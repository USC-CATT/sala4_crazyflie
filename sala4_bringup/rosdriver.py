#!/usr/bin/env python3
"""
cflib link driver for a SIMULATED Crazyflie: ``ros://`` URIs.

Import this module once and cflib accepts a new kind of Crazyflie URI:

    import cflib_ros_link            # registers the driver, nothing else to do
    ...
    with SyncCrazyflie("ros://crazyflie", cf=cf) as scf:
        ...                          # LogConfig, param.set_value, MotorRaw ...

There is no Crazyradio and no real firmware. This driver pretends to be the
firmware at the CRTP packet level, just enough for cflib's normal handshake:

  * link control + platform  -> reports protocol version (TOC v2)
  * log TOC / log blocks     -> streams stateEstimate.* / gyro.* / pm.vbat
                                taken from a ROS 2 nav_msgs/Odometry topic
  * param TOC / read / write -> a few stub parameters (motorPowerSet.enable, ...)
  * memory subsystem         -> reports 0 memories
  * port 0x09 (MotorRaw)     -> 4x uint16 PWM -> converted and published to
                                the Gazebo motor-speed topic

NOT emulated (packets are silently ignored): commander setpoints, high level
commander, console, memories, localization. Scripts that fly through
``cf.commander`` / ``MotionCommander`` will connect and log, but will not move
the simulated drone. Raw-motor scripts (MotorRaw) do.

URI options (query string, all optional):
  ros://crazyflie?odom=/crazyflie/odom&motors=/motor_raw
      odom=        nav_msgs/Odometry topic            (default /crazyflie/odom)
      motors=      motor speed topic                  (default /motor_raw)
      msg=         motor message type    (default actuator_msgs/msg/Actuators)
      imu=         optional sensor_msgs/Imu topic for gyro
      twist=       'body' (default) or 'world': frame of the odometry twist
      mass=        model mass in kg          (default 0.0282 = SDF body + 4 props)
      kf=          motorConstant from the SDF   (default 1.28192e-08)
      omega_hover= rotor speed that hovers the model; default sqrt(m*g/(4*kf))
      hover_cmd=   controller hover thrust command    (default 36000)
      motor_map=   Gazebo rotor index for M1..M4      (default 0,1,2,3)
      wait=        seconds to wait for the first odometry (default 10)
"""

import logging
import math
import queue
import struct
import threading
import time
import zlib
from typing import Callable, Dict, List, Optional
from urllib.parse import parse_qs, urlparse

import cflib.crtp
from cflib.crtp.crtpdriver import CRTPDriver
from cflib.crtp.crtpstack import CRTPPacket
from cflib.crtp.exceptions import WrongUriType

logger = logging.getLogger(__name__)

PORT_PARAM = 0x02
PORT_MEM = 0x04
PORT_LOG = 0x05
PORT_MOTOR_RAW = 0x09
PORT_PLATFORM = 0x0D
PORT_LINKCTRL = 0x0F

PROTOCOL_VERSION = 10  # >= 4: TOC v2. < 11: classic param protocol.

# ---- log TOC -----------------------------------------------------------------
LOG_TYPE_FLOAT = 0x07
# fetch type id -> struct format (matches cflib.crazyflie.log.LogTocElement.types)
_LOG_FMT = {1: "<B", 2: "<H", 3: "<L", 4: "<b", 5: "<h", 6: "<i", 7: "<f", 8: "<e"}

# Only variables that are really populated from ROS are advertised, so a script
# asking for anything else fails loudly instead of silently reading zeros.
LOG_VARIABLES = [
    "stateEstimate.x",
    "stateEstimate.y",
    "stateEstimate.z",
    "stateEstimate.vx",
    "stateEstimate.vy",
    "stateEstimate.vz",
    "stateEstimate.roll",
    "stateEstimate.pitch",
    "stateEstimate.yaw",
    "gyro.x",
    "gyro.y",
    "gyro.z",
    "pm.vbat",
]

# ---- param TOC (stubs, stored in memory) ---------------------------------------
PARAM_U8, PARAM_U16 = 0x08, 0x09
_PARAM_FMT = {PARAM_U8: "<B", PARAM_U16: "<H"}
PARAM_VARIABLES = [
    ("motorPowerSet", "enable", PARAM_U8),
    ("motorPowerSet", "m1", PARAM_U16),
    ("motorPowerSet", "m2", PARAM_U16),
    ("motorPowerSet", "m3", PARAM_U16),
    ("motorPowerSet", "m4", PARAM_U16),
    ("commander", "enHighLevel", PARAM_U8),
    ("stabilizer", "estimator", PARAM_U8),
    ("stabilizer", "controller", PARAM_U8),
    ("kalman", "resetEstimation", PARAM_U8),
]


def _pk(port: int, channel: int, data: bytes) -> CRTPPacket:
    pk = CRTPPacket()
    pk.set_header(port, channel)
    pk.data = bytes(data)
    return pk


def _toc_crc(descriptors) -> int:
    return zlib.crc32(repr(descriptors).encode()) & 0xFFFFFFFF


class _LogBlock:
    def __init__(self):
        self.variables = []  # list of (fetch_type, toc_id)
        self.period_s = 0.1
        self.running = False
        self.next_due = 0.0


class SimFirmware:
    """Pure-python CRTP firmware emulator (no ROS, no cflib link needed)."""

    def __init__(
        self,
        get_values: Callable[[], Dict[str, float]],
        send_motors: Callable[[int, int, int, int], None],
    ):
        self._get_values = get_values
        self._send_motors = send_motors
        self._lock = threading.RLock()
        self._t0 = time.monotonic()
        self._blocks: Dict[int, _LogBlock] = {}
        self.params: Dict[str, int] = {f"{g}.{n}": 0 for g, n, _ in PARAM_VARIABLES}
        self._log_crc = _toc_crc(LOG_VARIABLES)
        self._param_crc = _toc_crc(PARAM_VARIABLES)

    # ---- incoming packets from cflib ----------------------------------------
    def handle(self, pk: CRTPPacket) -> List[CRTPPacket]:
        port, ch, data = pk.port, pk.channel, bytes(pk.data)
        with self._lock:
            if port == PORT_LINKCTRL:
                if ch == 1:
                    return [_pk(PORT_LINKCTRL, 1, b"Bitcraze Crazyflie")]
            elif port == PORT_PLATFORM:
                if ch == 1 and data[:1] == b"\x00":
                    return [_pk(PORT_PLATFORM, 1, bytes([0, PROTOCOL_VERSION]))]
            elif port == PORT_MEM:
                if ch == 0 and data[:1] == b"\x01":  # CMD_INFO_NBR
                    return [_pk(PORT_MEM, 0, bytes([1, 0]))]  # zero memories
            elif port == PORT_LOG:
                return self._handle_log(ch, data)
            elif port == PORT_PARAM:
                return self._handle_param(ch, data)
            elif port == PORT_MOTOR_RAW:
                if len(data) >= 8:
                    self._send_motors(*struct.unpack("<HHHH", data[:8]))
            # everything else (commander, console, ...) is ignored
        return []

    # ---- log -------------------------------------------------------------------
    def _handle_log(self, ch: int, data: bytes) -> List[CRTPPacket]:
        if not data:
            return []
        cmd = data[0]
        if ch == 0:  # TOC access
            if cmd == 3:  # CMD_GET_INFO_V2
                body = struct.pack("<HI", len(LOG_VARIABLES), self._log_crc)
                return [_pk(PORT_LOG, 0, bytes([3]) + body)]
            if cmd == 1:  # CMD_TOC_INFO (v1)
                body = struct.pack("<BI", len(LOG_VARIABLES), self._log_crc)
                return [_pk(PORT_LOG, 0, bytes([1]) + body)]
            if cmd == 2:  # CMD_GET_ITEM_V2
                idx = data[1] | (data[2] << 8)
                group, name = LOG_VARIABLES[idx].split(".")
                body = bytes([LOG_TYPE_FLOAT]) + group.encode() + b"\0" + name.encode() + b"\0"
                return [_pk(PORT_LOG, 0, bytes([2, data[1], data[2]]) + body)]
            if cmd == 0:  # CMD_TOC_ELEMENT (v1)
                idx = data[1]
                group, name = LOG_VARIABLES[idx].split(".")
                body = bytes([LOG_TYPE_FLOAT]) + group.encode() + b"\0" + name.encode() + b"\0"
                return [_pk(PORT_LOG, 0, bytes([0, idx]) + body)]
        elif ch == 1:  # block control
            if cmd in (0, 6, 1, 7):  # create / append (v1, v2)
                block_id = data[1]
                v2 = cmd in (6, 7)
                step = 3 if v2 else 2
                if cmd in (0, 6) or block_id not in self._blocks:
                    self._blocks[block_id] = _LogBlock()
                block = self._blocks[block_id]
                body = data[2:]
                for i in range(0, len(body) - step + 1, step):
                    fetch = body[i] & 0x0F
                    toc_id = body[i + 1] | (body[i + 2] << 8) if v2 else body[i + 1]
                    block.variables.append((fetch, toc_id))
                return [_pk(PORT_LOG, 1, bytes([cmd, block_id, 0]))]
            if cmd == 3:  # start
                block_id, period = data[1], data[2]
                block = self._blocks.get(block_id)
                if block is None:
                    return [_pk(PORT_LOG, 1, bytes([3, block_id, 2]))]  # ENOENT
                block.period_s = max(period, 1) * 0.01
                block.running = True
                block.next_due = time.monotonic()
                return [_pk(PORT_LOG, 1, bytes([3, block_id, 0]))]
            if cmd == 4:  # stop
                block = self._blocks.get(data[1])
                if block:
                    block.running = False
                return [_pk(PORT_LOG, 1, bytes([4, data[1], 0]))]
            if cmd == 2:  # delete
                self._blocks.pop(data[1], None)
                return [_pk(PORT_LOG, 1, bytes([2, data[1], 0]))]
            if cmd == 5:  # reset
                self._blocks.clear()
                return [_pk(PORT_LOG, 1, bytes([5, 0, 0]))]
        return []

    def due_log_packets(self, now: float) -> List[CRTPPacket]:
        """Called periodically by the driver thread: log data for due blocks."""
        out: List[CRTPPacket] = []
        with self._lock:
            if not any(b.running for b in self._blocks.values()):
                return out
            values = self._get_values()
            stamp = int((now - self._t0) * 1000) & 0xFFFFFF
            for block_id, block in self._blocks.items():
                if not block.running or now < block.next_due:
                    continue
                block.next_due = max(block.next_due + block.period_s, now)
                payload = bytes([block_id]) + stamp.to_bytes(3, "little")
                for fetch, toc_id in block.variables:
                    payload += self._pack_log_value(fetch, LOG_VARIABLES[toc_id], values)
                out.append(_pk(PORT_LOG, 2, payload))
        return out

    @staticmethod
    def _pack_log_value(fetch: int, name: str, values: Dict[str, float]) -> bytes:
        fmt = _LOG_FMT[fetch]
        v = float(values.get(name, 0.0))
        try:
            if fmt in ("<f", "<e"):
                if fmt == "<e":
                    v = max(-65504.0, min(65504.0, v))
                return struct.pack(fmt, v)
            return struct.pack(fmt, int(round(v)))
        except (struct.error, OverflowError, ValueError):
            return struct.pack(fmt, 0)

    # ---- param -----------------------------------------------------------------
    def _handle_param(self, ch: int, data: bytes) -> List[CRTPPacket]:
        n = len(PARAM_VARIABLES)
        if ch == 0:  # TOC
            cmd = data[0] if data else None
            if cmd == 3:
                return [_pk(PORT_PARAM, 0, bytes([3]) + struct.pack("<HI", n, self._param_crc))]
            if cmd == 1:
                return [_pk(PORT_PARAM, 0, bytes([1]) + struct.pack("<BI", n, self._param_crc))]
            if cmd == 2:
                idx = data[1] | (data[2] << 8)
                group, name, code = PARAM_VARIABLES[idx]
                body = bytes([code]) + group.encode() + b"\0" + name.encode() + b"\0"
                return [_pk(PORT_PARAM, 0, bytes([2, data[1], data[2]]) + body)]
            if cmd == 0:
                idx = data[1]
                group, name, code = PARAM_VARIABLES[idx]
                body = bytes([code]) + group.encode() + b"\0" + name.encode() + b"\0"
                return [_pk(PORT_PARAM, 0, bytes([0, idx]) + body)]
        elif ch == 1 and len(data) >= 2:  # read
            idx = data[0] | (data[1] << 8)
            group, name, code = PARAM_VARIABLES[idx]
            value = struct.pack(_PARAM_FMT[code], self.params[f"{group}.{name}"])
            return [_pk(PORT_PARAM, 1, data[:2] + b"\x00" + value)]
        elif ch == 2 and len(data) >= 3:  # write
            idx = data[0] | (data[1] << 8)
            group, name, code = PARAM_VARIABLES[idx]
            fmt = _PARAM_FMT[code]
            (value,) = struct.unpack(fmt, data[2 : 2 + struct.calcsize(fmt)])
            self.params[f"{group}.{name}"] = value
            return [_pk(PORT_PARAM, 2, data)]  # echo = acknowledge
        return []


# ---- ROS backend ------------------------------------------------------------------
class RosBackend:
    """State/motor plumbing through roslink.RosLink (needs rclpy)."""

    def __init__(self, cfg: Dict[str, str]):
        from roslink import RosLink  # lazy: rclpy only needed for ros:// links

        mass = float(cfg.get("mass", 0.0282))
        kf = float(cfg.get("kf", 1.28192e-08))
        omega_hover = float(cfg.get("omega_hover", math.sqrt(mass * 9.80665 / (4.0 * kf))))
        self._values: Dict[str, float] = {}
        self._have_state = threading.Event()
        self._topic = cfg.get("odom", "/crazyflie/odom")
        self._link = RosLink(
            on_state=self._on_state,
            odom_topic=self._topic,
            motor_topic=cfg.get("motors", "/motor_raw"),
            omega_hover=omega_hover,
            hover_cmd=float(cfg.get("hover_cmd", 36000)),
            motor_msg_type=cfg.get("msg", "actuator_msgs/msg/Actuators"),
            imu_topic=cfg.get("imu"),
            twist_in_body=cfg.get("twist", "body") != "world",
            motor_map=[int(i) for i in cfg.get("motor_map", "0,1,2,3").split(",")],
        )
        self._link.start()

    def _on_state(self, data: Dict[str, float]):
        self._values.update(data)
        self._have_state.set()

    def wait_ready(self, timeout: float) -> bool:
        return self._have_state.wait(timeout)

    @property
    def topic(self) -> str:
        return self._topic

    def get_values(self) -> Dict[str, float]:
        return dict(self._values)

    def send_motors(self, m1: int, m2: int, m3: int, m4: int):
        self._link.send_motor_raw(m1, m2, m3, m4)

    def close(self):
        self._link.shutdown()


def _make_backend(cfg: Dict[str, str]):
    return RosBackend(cfg)


# ---- the cflib driver ---------------------------------------------------------------
class RosSimDriver(CRTPDriver):
    """cflib link driver for ``ros://`` URIs."""

    # MotorRaw checks this to avoid publishing motor commands twice.
    publishes_motors = True

    def __init__(self):
        CRTPDriver.__init__(self)
        self.needs_resending = False  # in-process link: never loses packets
        self.uri = ""
        self.in_queue: Optional[queue.Queue] = None
        self._backend = None
        self._fw: Optional[SimFirmware] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

    def connect(self, uri, radio_link_statistics_callback, link_error_callback):
        if not uri.startswith("ros://"):
            raise WrongUriType("Not a ros:// URI")
        parsed = urlparse(uri)
        cfg = {k: v[0] for k, v in parse_qs(parsed.query).items()}
        self.uri = uri

        self._backend = _make_backend(cfg)
        wait_s = float(cfg.get("wait", 10))
        if not self._backend.wait_ready(wait_s):
            topic = getattr(self._backend, "topic", "odometry")
            self._backend.close()
            raise Exception(
                f"No odometry received on {topic} within {wait_s:.0f}s. "
                "Is Gazebo running and the ros_gz bridge up? (ros2 topic echo it)"
            )

        self._fw = SimFirmware(self._backend.get_values, self._backend.send_motors)
        self.in_queue = queue.Queue()
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="RosSimDriver", daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            for pk in self._fw.due_log_packets(time.monotonic()):
                self.in_queue.put(pk)
            time.sleep(0.002)

    def send_packet(self, pk):
        if self._fw is None:
            return False
        for reply in self._fw.handle(pk):
            self.in_queue.put(reply)
        return True

    def receive_packet(self, wait=0):
        try:
            if wait == 0:
                return self.in_queue.get(False)
            if wait < 0:
                return self.in_queue.get(True)
            return self.in_queue.get(True, wait)
        except queue.Empty:
            return None

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._backend is not None:
            # make sure the simulated motors are not left spinning
            try:
                self._backend.send_motors(0, 0, 0, 0)
            except Exception:
                pass
            self._backend.close()
            self._backend = None

    def scan_interface(self, address=None):
        return []

    def get_status(self):
        return "ROS 2 simulated Crazyflie link"

    def get_name(self):
        return "ros"

    def get_help(self):
        return "ros://crazyflie?odom=/crazyflie/odom&motors=/motor_raw"


def _ensure_registered() -> None:
    if RosSimDriver not in cflib.crtp.CLASSES:
        cflib.crtp.CLASSES.insert(0, RosSimDriver)


def register() -> None:
    """Make cflib accept ros:// URIs. Safe to call repeatedly and in any order
    relative to cflib.crtp.init_drivers() (which is wrapped so that it cannot
    drop the driver, whatever the cflib version does with CLASSES)."""
    _ensure_registered()
    if not getattr(cflib.crtp.init_drivers, "_ros_wrapped", False):
        original = cflib.crtp.init_drivers

        def init_drivers(*args, **kwargs):
            result = original(*args, **kwargs)
            _ensure_registered()
            return result

        init_drivers._ros_wrapped = True
        cflib.crtp.init_drivers = init_drivers


register()