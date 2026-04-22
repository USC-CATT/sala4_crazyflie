#!/usr/bin/env python3
"""
Run the Python cascaded PID on the host and send raw PWM to all four motors.

This bypasses the firmware flight controller by enabling `motorPowerSet.enable`
and publishing motor commands on the raw motor CRTP port.
"""

# User-editable trajectory block.
# Edit this list directly to define the desired path.
# Each tuple is: (time_s, x_m, y_m, z_m, yaw_deg)
# time_s is relative to the start of the main control phase.
USE_SCRIPT_TRAJECTORY = True
USER_DEFINED_TRAJECTORY = [
    (0.0, 0.0, 0.0, 1.1, 0.0),
    (40000.0, 0.0, 0.0, 1.1, 0.0),
]
# after the last waypoint, there will be a default landing to z=0.05m in three seconds if --no-land is not specified, regardless of the last waypoint's z value

import argparse
import json
import logging
import math
import os
import sys
import time
from dataclasses import dataclass
from typing import Tuple

import numpy as np
from pynput import keyboard

# Ensure local imports work no matter where the script is launched from.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WAYPOINT_DATA_PATH = os.path.join(SCRIPT_DIR, "waypoint_data.json")
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

try:
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.log import LogConfig
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    from cflib.utils import uri_helper
except ModuleNotFoundError as exc:
    if exc.name == "cflib":
        raise SystemExit(
            "Missing dependency: cflib. Install it in this Python environment "
            f"(for example: `pip install cflib`) and run again.\n"
            f"Current interpreter: {sys.executable}"
        ) from exc
    raise

from controller.controller_mellinger import ControllerMellinger
from controller.controller_types import (
    AccData,
    Attitude,
    AttitudeRate,
    Axis3f,
    Control,
    GyroData,
    Position,
    Quaternion,
    SensorData,
    Setpoint,
    SetpointMode,
    StabMode,
    State,
    Velocity,
    quat2rpy,
)
from motorRaw import MotorRaw

logging.basicConfig(level=logging.ERROR)

# Taken from Crazyflie platform defaults / current local motor_control.py.
THRUST_MIN = 0.02136263065537499
THRUST_MAX = 0.2
VMOTOR2THRUST0 = -0.014058926705279723
VMOTOR2THRUST1 = 0.04265273261724981
VMOTOR2THRUST2 = 0.0018327760144017432
VMOTOR2THRUST3 = 0.0020576974784587178

IDLE_THRUST = 7000
UINT16_MAX = 65535
REDUCE_MULTIPLIER = 0.8


@dataclass(frozen=True)
class TrajectoryWaypoint:
    time_s: float
    x: float
    y: float
    z: float
    yaw_deg: float = 0.0


SCRIPT_TRAJECTORY = [
    TrajectoryWaypoint(time_s=t, x=x, y=y, z=z, yaw_deg=yaw)
    for (t, x, y, z, yaw) in USER_DEFINED_TRAJECTORY
]


@dataclass
class MotorThrust:
    motor_1: float = 0.0
    motor_2: float = 0.0
    motor_3: float = 0.0
    motor_4: float = 0.0


def _validate_trajectory() -> None:
    if not SCRIPT_TRAJECTORY:
        raise ValueError("SCRIPT_TRAJECTORY is empty")

    last_t = -1.0
    for waypoint in SCRIPT_TRAJECTORY:
        if waypoint.time_s < 0.0:
            raise ValueError("Trajectory times must be >= 0")
        if waypoint.time_s < last_t:
            raise ValueError("Trajectory times must be non-decreasing")
        last_t = waypoint.time_s


def _sample_trajectory(t_s: float) -> Tuple[float, float, float, float]:
    if len(SCRIPT_TRAJECTORY) == 1 or t_s <= SCRIPT_TRAJECTORY[0].time_s:
        p = SCRIPT_TRAJECTORY[0]
        return p.x, p.y, p.z, p.yaw_deg

    if t_s >= SCRIPT_TRAJECTORY[-1].time_s:
        p = SCRIPT_TRAJECTORY[-1]
        return p.x, p.y, p.z, p.yaw_deg

    for idx in range(len(SCRIPT_TRAJECTORY) - 1):
        p0 = SCRIPT_TRAJECTORY[idx]
        p1 = SCRIPT_TRAJECTORY[idx + 1]
        if p0.time_s <= t_s <= p1.time_s:
            dt = p1.time_s - p0.time_s
            alpha = 0.0 if dt <= 0.0 else (t_s - p0.time_s) / dt
            x = p0.x + (p1.x - p0.x) * alpha
            y = p0.y + (p1.y - p0.y) * alpha
            z = p0.z + (p1.z - p0.z) * alpha
            yaw = p0.yaw_deg + (p1.yaw_deg - p0.yaw_deg) * alpha
            return x, y, z, yaw

    p = SCRIPT_TRAJECTORY[-1]
    return p.x, p.y, p.z, p.yaw_deg


json_data = {
    "time_s": [],
    "loop_hz": 0.0,
    "sample_period_s": 0.0,
    "position_x": [],
    "setpoint_x": [],
    "position_y": [],
    "setpoint_y": [],
    "position_z": [],
    "setpoint_z": [],
}


class HostPIDPWMPositionController:
    def __init__(
        self,
        uri: str,
        target_x: float,
        target_y: float,
        target_z: float,
        target_yaw: float,
        loop_hz: float,
        run_seconds: float,
        land_z: float,
        land_seconds: float,
        do_land: bool,
    ):
        self.uri = uri
        self.loop_hz = loop_hz
        self.loop_period = 1.0 / loop_hz
        self.run_seconds = run_seconds
        self.land_z = land_z
        self.land_seconds = land_seconds
        self.do_land = do_land
        self._log_time_origin = None

        json_data["loop_hz"] = self.loop_hz
        json_data["sample_period_s"] = self.loop_period

        self.controller = ControllerMellinger()
        
        self.stabilizer_step = 1
        self.control = Control()

        self.cf_state = State(
            attitude=Attitude(),
            position=Position(),
            velocity=Velocity(),
            acc=Axis3f(),
        )
        self.cf_sensors = SensorData(gyro=GyroData(), acc=AccData())
        self.cf_vbat = 4.2
        self._have_state = False
        self._have_sensor = False

        if USE_SCRIPT_TRAJECTORY:
            _validate_trajectory()
            init_x, init_y, init_z, init_yaw = _sample_trajectory(0.0)
        else:
            init_x, init_y, init_z, init_yaw = target_x, target_y, target_z, target_yaw

        self.cf_setpoint = Setpoint()
        self.cf_setpoint.position = Position(x=init_x, y=init_y, z=init_z)
        self.cf_setpoint.velocity = Velocity(x=0.0, y=0.0, z=0.0)
        self.cf_setpoint.attitude = Attitude(roll=0.0, pitch=0.0, yaw=init_yaw)
        self.cf_setpoint.attitude_rate = AttitudeRate(roll=0.0, pitch=0.0, yaw=0.0)
        self.cf_setpoint.velocity_body = False
        self.cf_setpoint.mode = SetpointMode()
        self.cf_setpoint.mode.x = StabMode.MODE_ABS
        self.cf_setpoint.mode.y = StabMode.MODE_ABS
        self.cf_setpoint.mode.z = StabMode.MODE_ABS
        self.cf_setpoint.mode.roll = StabMode.MODE_DISABLE
        self.cf_setpoint.mode.pitch = StabMode.MODE_DISABLE
        self.cf_setpoint.mode.yaw = StabMode.MODE_ABS

        self.cf = Crazyflie(rw_cache="./cache")
        self.motor_raw = MotorRaw(crazyflie=self.cf)

        self.log_state = LogConfig(name="HostPIDState", period_in_ms=60)
        self.log_state.add_variable("stateEstimate.vx", "FP16")
        self.log_state.add_variable("stateEstimate.vy", "FP16")
        self.log_state.add_variable("stateEstimate.vz", "FP16")
        self.log_state.add_variable("stateEstimate.ax", "FP16")
        self.log_state.add_variable("stateEstimate.ay", "FP16")
        self.log_state.add_variable("stateEstimate.az", "FP16")
        self.log_state.add_variable("stateEstimate.x", "FP16")
        self.log_state.add_variable("stateEstimate.y", "FP16")
        self.log_state.add_variable("stateEstimate.z", "FP16")
        # self.log_state.add_variable("stateEstimate.pitch", "FP16")
        # self.log_state.add_variable("stateEstimate.roll", "FP16")
        # self.log_state.add_variable("stateEstimate.yaw", "FP16")
        self.log_state.add_variable("stateEstimate.qx", "FP16")
        self.log_state.add_variable("stateEstimate.qy", "FP16")
        self.log_state.add_variable("stateEstimate.qz", "FP16")
        self.log_state.add_variable("stateEstimate.qw", "FP16")

        self.log_sensor = LogConfig(name="HostPIDSensor", period_in_ms=60)
        self.log_sensor.add_variable("gyro.x", "float")
        self.log_sensor.add_variable("gyro.y", "float")
        self.log_sensor.add_variable("gyro.z", "float")
        self.log_sensor.add_variable("acc.x", "float")
        self.log_sensor.add_variable("acc.y", "float")
        self.log_sensor.add_variable("acc.z", "float")
        self.log_sensor.add_variable("pm.vbat", "FP16")
        self.killed = False
        
        self.m1_multiplier = 1.0
        self.m2_multiplier = 1.0
        self.m3_multiplier = 1.0
        self.m4_multiplier = 1.0

        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

    def kill(self):
        print("[KILLING DRONE]")
        self.killed = True
        self._stop_motors()

    def on_press(self, key):
        if key == keyboard.Key.space:
            self.kill()
        if key.char == "1":
            self.m1_multiplier = REDUCE_MULTIPLIER
        if key.char == "2":
            self.m2_multiplier = REDUCE_MULTIPLIER
        if key.char == "3":
            self.m3_multiplier = REDUCE_MULTIPLIER
        if key.char == "4":
            self.m4_multiplier = REDUCE_MULTIPLIER

    def run(self):
        cflib.crtp.init_drivers()
        print(f"Connecting to {self.uri}")

        with SyncCrazyflie(self.uri, cf=self.cf) as scf:
            logs_started = False
            try:
                self.cf.log.add_config(self.log_state)
                self.log_state.data_received_cb.add_callback(self._log_state_callback)
                self.cf.log.add_config(self.log_sensor)
                self.log_sensor.data_received_cb.add_callback(self._log_sensor_callback)

                self.log_state.start()
                self.log_sensor.start()
                logs_started = True

                scf.cf.param.set_value("motorPowerSet.enable", 1)
                self._wait_for_logs(timeout_s=3.0)
                self._spinup(duration_s=1.0)
                self._log_time_origin = time.monotonic()
                if USE_SCRIPT_TRAJECTORY:
                    traj_duration = SCRIPT_TRAJECTORY[-1].time_s
                    print(f"Following script trajectory for {traj_duration:.1f}s")
                    self._control_for(
                        duration_s=traj_duration, follow_script_trajectory=True
                    )
                else:
                    self._control_for(duration_s=self.run_seconds)

                if self.do_land and not self.killed:
                    print(
                        f"Landing target z={self.land_z:.2f} for {self.land_seconds:.1f}s"
                    )
                    self.cf_setpoint.position.z = self.land_z
                    self._control_for(
                        duration_s=self.land_seconds, follow_script_trajectory=False
                    )
            finally:
                self._stop_motors()
                if logs_started:
                    self.log_state.stop()
                    self.log_sensor.stop()
                with open(WAYPOINT_DATA_PATH, "w", encoding="utf-8") as json_file:
                    json.dump(json_data, json_file, indent=4)

    def _wait_for_logs(self, timeout_s: float):
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            if self._have_state and self._have_sensor:
                return
            time.sleep(0.01)
        raise RuntimeError("Timed out waiting for state/sensor logs")

    def _spinup(self, duration_s: float):
        print(f"Spinup for {duration_s:.1f}s")
        end = time.monotonic() + duration_s
        while time.monotonic() < end:
            self.motor_raw.send_motor_raw(
                IDLE_THRUST, IDLE_THRUST, IDLE_THRUST, IDLE_THRUST
            )
            time.sleep(0.03)

    def _control_for(self, duration_s: float, follow_script_trajectory: bool = False):
        print(
            "Control target "
            f"x={self.cf_setpoint.position.x:.2f}, "
            f"y={self.cf_setpoint.position.y:.2f}, "
            f"z={self.cf_setpoint.position.z:.2f}, "
            f"yaw={self.cf_setpoint.attitude.yaw:.1f} deg"
        )
        phase_start = time.monotonic()
        end = phase_start + duration_s
        next_tick = time.monotonic()
        next_print = time.monotonic()

        while time.monotonic() < end:
            if self.killed:
                return
            now = time.monotonic()
            if follow_script_trajectory:
                elapsed = now - phase_start
                x, y, z, yaw = _sample_trajectory(elapsed)
                self.cf_setpoint.position.x = x
                self.cf_setpoint.position.y = y
                self.cf_setpoint.position.z = z
                self.cf_setpoint.attitude.yaw = yaw

            if self._log_time_origin is None:
                self._log_time_origin = now
            json_data["time_s"].append(now - self._log_time_origin)
            json_data["position_x"].append(self.cf_state.position.x)
            json_data["setpoint_x"].append(self.cf_setpoint.position.x)
            json_data["position_y"].append(self.cf_state.position.y)
            json_data["setpoint_y"].append(self.cf_setpoint.position.y)
            json_data["position_z"].append(self.cf_state.position.z)
            json_data["setpoint_z"].append(self.cf_setpoint.position.z)

            self._control_step()
            next_tick += self.loop_period
            sleep_time = next_tick - time.monotonic()
            if sleep_time > 0.0:
                time.sleep(sleep_time)
            else:
                next_tick = time.monotonic()

            if time.monotonic() >= next_print:
                print(
                    "state "
                    f"x={self.cf_state.position.x:.2f} "
                    f"y={self.cf_state.position.y:.2f} "
                    f"z={self.cf_state.position.z:.2f} | "
                    f"sp=({self.cf_setpoint.position.x:.2f},"
                    f"{self.cf_setpoint.position.y:.2f},"
                    f"{self.cf_setpoint.position.z:.2f}) | "
                    f"thrust={self.control.thrust:.1f} "
                    f"roll={self.control.roll:.1f} "
                    f"pitch={self.control.pitch:.1f} "
                    f"yaw={self.control.yaw:.1f}"
                )

                next_print = time.monotonic() + 0.25

    def _control_step(self):
        if self.killed:
            return
        self.controller.controller_mellinger(
            self.control,
            self.cf_setpoint,
            self.cf_sensors,
            self.cf_state,
            self.stabilizer_step,
        )
        self.stabilizer_step += 1

        raw = MotorThrust()
        self._power_distributor(self.control, raw)
        compensated = MotorThrust()
        self._battery_compensator(raw, compensated)
        pwm = MotorThrust()
        self._power_distribution_cap(compensated, pwm)

        self.motor_raw.send_motor_raw(
            int(pwm.motor_1 * self.m1_multiplier),
            int(pwm.motor_2 * self.m2_multiplier),
            int(pwm.motor_3 * self.m3_multiplier),
            int(pwm.motor_4 * self.m4_multiplier),
        )

    def _power_distributor(self, control: Control, motor_thrust: MotorThrust):
        r = control.roll / 2.0
        p = control.pitch / 2.0
        motor_thrust.motor_1 = control.thrust - r + p + control.yaw
        motor_thrust.motor_2 = control.thrust - r - p - control.yaw
        motor_thrust.motor_3 = control.thrust + r - p + control.yaw
        motor_thrust.motor_4 = control.thrust + r + p - control.yaw

    def _battery_compensator(
        self, motor_thrust_uncapped: MotorThrust, motor_thrust_bat_comp: MotorThrust
    ):
        b = 0.01
        supply_voltage = 4.2
        supply_voltage = supply_voltage + b * (self.cf_vbat - supply_voltage)
        motor_thrust_bat_comp.motor_1 = self._compensate_voltage(
            i_thrust=motor_thrust_uncapped.motor_1, supply_voltage=supply_voltage
        )
        motor_thrust_bat_comp.motor_2 = self._compensate_voltage(
            i_thrust=motor_thrust_uncapped.motor_2, supply_voltage=supply_voltage
        )
        motor_thrust_bat_comp.motor_3 = self._compensate_voltage(
            i_thrust=motor_thrust_uncapped.motor_3, supply_voltage=supply_voltage
        )
        motor_thrust_bat_comp.motor_4 = self._compensate_voltage(
            i_thrust=motor_thrust_uncapped.motor_4, supply_voltage=supply_voltage
        )

    def _compensate_voltage(self, i_thrust: float, supply_voltage: float) -> float:
        if supply_voltage < 2.0:
            return 0.0

        thrust = (i_thrust / UINT16_MAX) * THRUST_MAX
        if thrust < THRUST_MIN:
            return 0.0

        p = -VMOTOR2THRUST2 / (3.0 * VMOTOR2THRUST3)
        q = p * p * p + (
            VMOTOR2THRUST2 * VMOTOR2THRUST1 - 3.0 * VMOTOR2THRUST3 * (VMOTOR2THRUST0 - thrust)
        ) / (6.0 * VMOTOR2THRUST3 * VMOTOR2THRUST3)
        r = VMOTOR2THRUST1 / (3.0 * VMOTOR2THRUST3)
        qrp = math.sqrt(q * q + (r - p * p) * (r - p * p) * (r - p * p))

        motor_voltage = self._cbrt(q + qrp) + self._cbrt(q - qrp) + p
        ratio = motor_voltage / supply_voltage
        return UINT16_MAX * ratio

    @staticmethod
    def _cbrt(x: float) -> float:
        return math.copysign(abs(x) ** (1.0 / 3.0), x)

    def _power_distribution_cap(
        self, motor_thrust_bat_comp: MotorThrust, motor_thrust_pwm: MotorThrust
    ):
        thrusts = [
            motor_thrust_bat_comp.motor_1,
            motor_thrust_bat_comp.motor_2,
            motor_thrust_bat_comp.motor_3,
            motor_thrust_bat_comp.motor_4,
        ]
        reduction = max(0.0, max(thrusts) - UINT16_MAX)
        motor_thrust_pwm.motor_1 = max(IDLE_THRUST, motor_thrust_bat_comp.motor_1 - reduction)
        motor_thrust_pwm.motor_2 = max(IDLE_THRUST, motor_thrust_bat_comp.motor_2 - reduction)
        motor_thrust_pwm.motor_3 = max(IDLE_THRUST, motor_thrust_bat_comp.motor_3 - reduction)
        motor_thrust_pwm.motor_4 = max(IDLE_THRUST, motor_thrust_bat_comp.motor_4 - reduction)

    def _stop_motors(self):
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            self.motor_raw.send_motor_raw(0, 0, 0, 0)
            time.sleep(0.01)

    def _log_state_callback(self, _timestamp, data, _logconf):
        self.cf_state.position.x = data["stateEstimate.x"]
        self.cf_state.position.y = data["stateEstimate.y"]
        self.cf_state.position.z = data["stateEstimate.z"]
        self.cf_state.velocity.x = data["stateEstimate.vx"]
        self.cf_state.velocity.y = data["stateEstimate.vy"]
        self.cf_state.velocity.z = data["stateEstimate.vz"]
        self.cf_state.acc.x = data["stateEstimate.ax"]
        self.cf_state.acc.y = data["stateEstimate.ay"]
        self.cf_state.acc.z = data["stateEstimate.az"]

        qx = data["stateEstimate.qx"]
        qy = data["stateEstimate.qy"]
        qz = data["stateEstimate.qz"]
        qw = data["stateEstimate.qw"]
        q = Quaternion(w=qw, x=qx, y=qy, z=qz)
        rpy = quat2rpy(q)
        self.cf_state.attitude_quaternion.x = qx
        self.cf_state.attitude_quaternion.y = qy
        self.cf_state.attitude_quaternion.z = qz
        self.cf_state.attitude_quaternion.w = qw

        self.cf_state.attitude.roll = np.degrees(rpy.x)
        self.cf_state.attitude.pitch = np.degrees(rpy.y)
        self.cf_state.attitude.yaw = np.degrees(rpy.z)
        
        self._have_state = True

    def _log_sensor_callback(self, _timestamp, data, _logconf):
        self.cf_sensors.gyro.x = data["gyro.x"]
        self.cf_sensors.gyro.y = data["gyro.y"]
        self.cf_sensors.gyro.z = data["gyro.z"]
        self.cf_sensors.acc.x = data["acc.x"]
        self.cf_sensors.acc.y = data["acc.y"]
        self.cf_sensors.acc.z = data["acc.z"]
        self.cf_vbat = data["pm.vbat"]
        self._have_sensor = True


def parse_args():
    parser = argparse.ArgumentParser(
        description="Host-side cascaded PID position hold with raw motor PWM output."
    )
    parser.add_argument(
        "--uri",
        default=uri_helper.uri_from_env(default="radio://0/80/2M/E7E7E7E7E7"),
        help="Crazyflie URI",
    )
    parser.add_argument(
        "--x",
        type=float,
        default=0.0,
        help="Desired X position (m), used when USE_SCRIPT_TRAJECTORY=False",
    )
    parser.add_argument(
        "--y",
        type=float,
        default=0.0,
        help="Desired Y position (m), used when USE_SCRIPT_TRAJECTORY=False",
    )
    parser.add_argument(
        "--z",
        type=float,
        default=0.6,
        help="Desired Z position (m), used when USE_SCRIPT_TRAJECTORY=False",
    )
    parser.add_argument(
        "--yaw",
        type=float,
        default=0.0,
        help="Desired yaw (deg), used when USE_SCRIPT_TRAJECTORY=False",
    )
    parser.add_argument(
        "--run-seconds",
        type=float,
        default=5.0,
        help="Main control duration before landing",
    )
    parser.add_argument(
        "--loop-hz", type=float, default=200.0, help="Host control loop rate"
    )
    parser.add_argument(
        "--land-z", type=float, default=0.05, help="Landing target z (m)"
    )
    parser.add_argument(
        "--land-seconds", type=float, default=3.0, help="Landing duration"
    )
    parser.add_argument(
        "--no-land",
        action="store_true",
        help="Skip landing phase and stop motors directly after run phase",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    controller = HostPIDPWMPositionController(
        uri=args.uri,
        target_x=args.x,
        target_y=args.y,
        target_z=args.z,
        target_yaw=args.yaw,
        loop_hz=args.loop_hz,
        run_seconds=args.run_seconds,
        land_z=args.land_z,
        land_seconds=args.land_seconds,
        do_land=not args.no_land,
    )
    try:
        controller.run()
    except KeyboardInterrupt:
        print("Interrupted, stopping motors")
        controller._stop_motors()
        sys.exit(130)
    except Exception as exc:
        msg = str(exc)
        if "Resource busy" in msg or "Couldn't load link driver" in msg:
            print("Fatal error: Crazyradio is busy.")
            print(
                "Close other tools using the radio (for example `cfclient`) and retry."
            )
            print(f"URI: {args.uri}")
            controller._stop_motors()
            sys.exit(1)
        print(f"Fatal error: {exc}")
        controller._stop_motors()
        raise


if __name__ == "__main__":
    main()
