#!/usr/bin/env python3
"""
Host-side augmented cascaded controller with raw PWM motor output.

This script bypasses the firmware flight controller by:
1) reading estimator + IMU logs from the Crazyflie
2) running a host-side controller
3) sending PWM values directly to all four motors

Augmentation is based on the simulator implementation in:
/home/gary/Downloads/Crazyflie-simulator-main/augmented_controller.py
"""

# User-editable trajectory block.
# Each tuple is: (time_s, x_m, y_m, z_m, yaw_deg)
# time_s is relative to the start of the main control phase.
USE_SCRIPT_TRAJECTORY = True
USER_DEFINED_TRAJECTORY = [
    (0.0, 0.0, 0.0, 0.04, 0.0),
    (3.0, 0.0, 0.0, 0.5, 0.0),
    (9.0, 0.3, 0.0, 0.5, 0.0),
    # (9.0, 0.3, 0.3, 0.9, 20.0),
    # (12.0, 0.0, 0.0, 0.8, 0.0),
]

# Augmentation tuning (ported from simulator augmented_controller.py).
AUG_ENABLED = True
AUG_MU = 0.15
AUG_V_LIMIT_XY = 0.2
AUG_START_TIME = 6.0
AUG_RAMP_TIME = 5.0
AUG_V_SLEW_LIMIT_XY = 0.25
AUG_MAX_ATT_DELTA_DEG = 1.0
AUG_DISABLE_IF_TILT_OVER_DEG = 22.0
AUG_DISABLE_IF_SPEED_OVER_MPS = 2.0
AUG_SIGN_XY = 1.0
USE_BASELINE_BEFORE_AUG_START = True

import argparse
import logging
import math
import os
import sys
import time
from dataclasses import dataclass
from typing import Dict, Tuple

# Ensure local imports work no matter where the script is launched from.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
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

from controller.attitude_controller import AttitudeController
from controller.controller_pid import ControllerPID
from controller.pid import constrain
from controller.pid_types import (
    POSITION_RATE,
    AccData,
    Attitude,
    AttitudeRate,
    Axis3f,
    Control,
    ControlMode,
    GyroData,
    Position,
    SensorData,
    Setpoint,
    SetpointMode,
    StabMode,
    State,
    Velocity,
    cap_angle,
)
from controller.position_controller import PositionController
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
GRAVITY = 9.81


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


class LSAugmentationXY:
    """Least-squares XY augmentation ported from simulator augmented_controller.py."""

    def __init__(
        self,
        dt: float,
        mu: float = AUG_MU,
        v_limit_xy: float = AUG_V_LIMIT_XY,
        start_time: float = AUG_START_TIME,
        ramp_time: float = AUG_RAMP_TIME,
        v_slew_limit_xy: float = AUG_V_SLEW_LIMIT_XY,
        enabled: bool = AUG_ENABLED,
    ):
        self.dt = float(dt)
        self.mu = float(mu)
        self.v_limit_xy = float(v_limit_xy)
        self.start_time = float(start_time)
        self.ramp_time = float(ramp_time)
        self.v_slew_limit_xy = float(v_slew_limit_xy)
        self.enabled = bool(enabled)

        self._xhat = [[0.0, 0.0], [0.0, 0.0]]  # axis -> [pos_hat, vel_hat]
        self._e_prev = [[0.0, 0.0], [0.0, 0.0]]
        self._ef = [[0.0, 0.0], [0.0, 0.0]]
        self._initialized = False
        self._last_v = [0.0, 0.0]

    def reset(self) -> None:
        self._xhat = [[0.0, 0.0], [0.0, 0.0]]
        self._e_prev = [[0.0, 0.0], [0.0, 0.0]]
        self._ef = [[0.0, 0.0], [0.0, 0.0]]
        self._initialized = False
        self._last_v = [0.0, 0.0]

    def _align_to_measurement(self, pos_xy: Tuple[float, float], vel_xy: Tuple[float, float]) -> None:
        # Keep the digital twin synced before augmentation starts to avoid kick transients.
        self._xhat[0][0], self._xhat[0][1] = pos_xy[0], vel_xy[0]
        self._xhat[1][0], self._xhat[1][1] = pos_xy[1], vel_xy[1]
        self._e_prev = [[0.0, 0.0], [0.0, 0.0]]
        self._ef = [[0.0, 0.0], [0.0, 0.0]]
        self._initialized = False

    def _slew_limit(self, target: float, prev: float) -> float:
        max_step = max(0.0, self.v_slew_limit_xy) * max(self.dt, 1e-6)
        low = prev - max_step
        high = prev + max_step
        return float(constrain(target, low, high))

    def _axis_virtual_input(self, axis: int, pos: float, vel: float, u_nom: float) -> float:
        dt = max(self.dt, 1e-6)
        mu = max(self.mu, 1e-6)

        # Digital twin update with xhat_dot = [vel, u_nom]^T.
        self._xhat[axis][0] += dt * vel
        self._xhat[axis][1] += dt * u_nom

        e0 = pos - self._xhat[axis][0]
        e1 = vel - self._xhat[axis][1]

        if self._initialized:
            e_dot0 = (e0 - self._e_prev[axis][0]) / dt
            e_dot1 = (e1 - self._e_prev[axis][1]) / dt
        else:
            e_dot0 = 0.0
            e_dot1 = 0.0

        # Stable FOH update: e_f += alpha * (e_dot - e_f), alpha = 1-exp(-dt/mu).
        alpha = 1.0 - math.exp(-dt / mu)
        self._ef[axis][0] += alpha * (e_dot0 - self._ef[axis][0])
        self._ef[axis][1] += alpha * (e_dot1 - self._ef[axis][1])

        self._e_prev[axis][0] = e0
        self._e_prev[axis][1] = e1

        # B=[0,1]^T -> v = -e_f[1].
        v = -self._ef[axis][1] * AUG_SIGN_XY
        return float(constrain(v, -self.v_limit_xy, self.v_limit_xy))

    def compute(
        self,
        sim_time: float,
        pos_xy: Tuple[float, float],
        vel_xy: Tuple[float, float],
        u_nom_xy: Tuple[float, float],
    ) -> Tuple[float, float]:
        if not self.enabled:
            self._align_to_measurement(pos_xy, vel_xy)
            self._initialized = True
            self._last_v = [0.0, 0.0]
            return 0.0, 0.0

        if sim_time < self.start_time:
            self._align_to_measurement(pos_xy, vel_xy)
            self._last_v = [0.0, 0.0]
            return 0.0, 0.0

        vx_est = self._axis_virtual_input(0, pos_xy[0], vel_xy[0], u_nom_xy[0])
        vy_est = self._axis_virtual_input(1, pos_xy[1], vel_xy[1], u_nom_xy[1])
        self._initialized = True

        if self.ramp_time > 0.0:
            ramp = constrain((sim_time - self.start_time) / self.ramp_time, 0.0, 1.0)
        else:
            ramp = 1.0
        vx_tgt = vx_est * ramp
        vy_tgt = vy_est * ramp

        vx = self._slew_limit(vx_tgt, self._last_v[0])
        vy = self._slew_limit(vy_tgt, self._last_v[1])
        self._last_v = [vx, vy]

        return self._last_v[0], self._last_v[1]

    def diagnostics(self) -> Dict[str, float]:
        return {
            "v_aug_x": self._last_v[0],
            "v_aug_y": self._last_v[1],
            "ef_x_pos": self._ef[0][0],
            "ef_x_vel": self._ef[0][1],
            "ef_y_pos": self._ef[1][0],
            "ef_y_vel": self._ef[1][1],
        }


class AugmentedCascadeController:
    """Hardware-side cascaded controller with XY LS augmentation."""

    def __init__(self, loop_hz: float):
        self.loop_hz = float(loop_hz)
        self.dt = 1.0 / self.loop_hz
        self.outer_div = max(1, int(round(self.loop_hz / POSITION_RATE)))
        self.outer_dt = self.outer_div / self.loop_hz
        self.tick = 0

        self.attitude_controller = AttitudeController(update_dt=self.dt)
        self.position_controller = PositionController()
        # Augmentation runs at the outer-loop cadence (typically 100 Hz).
        self.augmentor = LSAugmentationXY(dt=self.outer_dt)

        self.control = Control()
        self.attitude_desired = Attitude()
        self.rate_desired = Attitude()
        self.actuator_thrust = 0.0
        self._last_diag: Dict[str, float] = {
            "v_aug_x": 0.0,
            "v_aug_y": 0.0,
            "ef_x_pos": 0.0,
            "ef_x_vel": 0.0,
            "ef_y_pos": 0.0,
        }

    @staticmethod
    def _nominal_accel_from_attitude(
        roll_deg: float, pitch_deg: float, yaw_deg: float
    ) -> Tuple[float, float]:
        # Inverse of simulator mapping used in controller.py:
        # des_phi = (ax*sin(yaw)-ay*cos(yaw))/g
        # des_theta = (ax*cos(yaw)+ay*sin(yaw))/g
        phi = math.radians(roll_deg)
        theta = math.radians(pitch_deg)
        yaw = math.radians(yaw_deg)
        ax = GRAVITY * (phi * math.sin(yaw) + theta * math.cos(yaw))
        ay = GRAVITY * (-phi * math.cos(yaw) + theta * math.sin(yaw))
        return ax, ay

    @staticmethod
    def _attitude_delta_from_aug(
        v_aug_x: float, v_aug_y: float, yaw_deg: float
    ) -> Tuple[float, float]:
        yaw = math.radians(yaw_deg)
        d_roll_rad = (v_aug_x * math.sin(yaw) - v_aug_y * math.cos(yaw)) / GRAVITY
        d_pitch_rad = (v_aug_x * math.cos(yaw) + v_aug_y * math.sin(yaw)) / GRAVITY
        d_roll_deg = math.degrees(d_roll_rad)
        d_pitch_deg = math.degrees(d_pitch_rad)
        d_roll_deg = constrain(d_roll_deg, -AUG_MAX_ATT_DELTA_DEG, AUG_MAX_ATT_DELTA_DEG)
        d_pitch_deg = constrain(d_pitch_deg, -AUG_MAX_ATT_DELTA_DEG, AUG_MAX_ATT_DELTA_DEG)
        return d_roll_deg, d_pitch_deg

    def reset(self, state: State) -> None:
        self.tick = 0
        self.augmentor.reset()
        self.attitude_controller.reset_all_pid(
            state.attitude.roll, state.attitude.pitch, state.attitude.yaw
        )
        self.position_controller.reset_all_pid(
            state.position.x, state.position.y, state.position.z
        )
        self.attitude_desired.roll = state.attitude.roll
        self.attitude_desired.pitch = state.attitude.pitch
        self.attitude_desired.yaw = state.attitude.yaw
        self.rate_desired = Attitude()
        self.actuator_thrust = 0.0

    def step(
        self, setpoint: Setpoint, sensors: SensorData, state: State, sim_time: float
    ) -> Tuple[Control, Dict[str, float]]:
        self.tick += 1
        update_outer = (self.tick == 1) or ((self.tick % self.outer_div) == 0)

        # Match firmware-style yaw setpoint handling.
        if setpoint.mode.yaw == StabMode.MODE_VELOCITY:
            self.attitude_desired.yaw = cap_angle(
                self.attitude_desired.yaw + setpoint.attitude_rate.yaw * self.dt
            )
        elif setpoint.mode.yaw == StabMode.MODE_ABS:
            self.attitude_desired.yaw = setpoint.attitude.yaw
        self.attitude_desired.yaw = cap_angle(self.attitude_desired.yaw)

        if update_outer:
            thrust_nom, attitude_nom = self.position_controller.position_controller(
                setpoint, state
            )
            self.actuator_thrust = thrust_nom

            tilt_deg = max(abs(state.attitude.roll), abs(state.attitude.pitch))
            horiz_speed = math.hypot(state.velocity.x, state.velocity.y)
            apply_aug = (
                setpoint.mode.x == StabMode.MODE_ABS
                and setpoint.mode.y == StabMode.MODE_ABS
                and self.actuator_thrust > 0.0
                and tilt_deg < AUG_DISABLE_IF_TILT_OVER_DEG
                and horiz_speed < AUG_DISABLE_IF_SPEED_OVER_MPS
            )
            if apply_aug:
                # Convert nominal attitude command back to nominal XY acceleration.
                a_nom_x, a_nom_y = self._nominal_accel_from_attitude(
                    attitude_nom.roll, attitude_nom.pitch, self.attitude_desired.yaw
                )
                v_aug_x, v_aug_y = self.augmentor.compute(
                    sim_time=sim_time,
                    pos_xy=(state.position.x, state.position.y),
                    vel_xy=(state.velocity.x, state.velocity.y),
                    u_nom_xy=(a_nom_x, a_nom_y),
                )
            else:
                v_aug_x, v_aug_y = 0.0, 0.0
                self.augmentor.reset()
            self._last_diag = self.augmentor.diagnostics()
            if not apply_aug:
                self._last_diag["v_aug_x"] = 0.0
                self._last_diag["v_aug_y"] = 0.0

            # Inject augmented virtual input as attitude delta (equivalent XY accel add).
            d_roll, d_pitch = self._attitude_delta_from_aug(
                v_aug_x, v_aug_y, self.attitude_desired.yaw
            )
            roll_des = attitude_nom.roll + d_roll
            pitch_des = attitude_nom.pitch + d_pitch

            self.attitude_desired.roll = constrain(
                roll_des,
                -self.position_controller.r_limit,
                self.position_controller.r_limit,
            )
            self.attitude_desired.pitch = constrain(
                pitch_des,
                -self.position_controller.p_limit,
                self.position_controller.p_limit,
            )

        # Manual overrides for direct modes.
        if setpoint.mode.z == StabMode.MODE_DISABLE:
            self.actuator_thrust = setpoint.thrust

        if (
            setpoint.mode.x == StabMode.MODE_DISABLE
            or setpoint.mode.y == StabMode.MODE_DISABLE
        ):
            self.attitude_desired.roll = setpoint.attitude.roll
            self.attitude_desired.pitch = setpoint.attitude.pitch

        # Attitude loop.
        (
            self.rate_desired.roll,
            self.rate_desired.pitch,
            self.rate_desired.yaw,
        ) = self.attitude_controller.correct_attitude_pid(
            state.attitude.roll,
            state.attitude.pitch,
            state.attitude.yaw,
            self.attitude_desired.roll,
            self.attitude_desired.pitch,
            self.attitude_desired.yaw,
        )

        if setpoint.mode.roll == StabMode.MODE_VELOCITY:
            self.rate_desired.roll = setpoint.attitude_rate.roll
            self.attitude_controller.reset_roll_attitude_pid(state.attitude.roll)

        if setpoint.mode.pitch == StabMode.MODE_VELOCITY:
            self.rate_desired.pitch = setpoint.attitude_rate.pitch
            self.attitude_controller.reset_pitch_attitude_pid(state.attitude.pitch)

        # Rate loop (note: gyro.y sign follows existing implementation).
        self.attitude_controller.correct_rate_pid(
            sensors.gyro.x,
            -sensors.gyro.y,
            sensors.gyro.z,
            self.rate_desired.roll,
            self.rate_desired.pitch,
            self.rate_desired.yaw,
        )
        roll_out, pitch_out, yaw_out = self.attitude_controller.get_actuator_output()

        self.control.control_mode = ControlMode.CONTROL_MODE_LEGACY
        self.control.roll = roll_out
        self.control.pitch = pitch_out
        self.control.yaw = -yaw_out
        self.control.thrust = self.actuator_thrust

        # Safety/reset on zero thrust.
        if self.control.thrust == 0:
            self.control.roll = 0
            self.control.pitch = 0
            self.control.yaw = 0
            self.attitude_controller.reset_all_pid(
                state.attitude.roll, state.attitude.pitch, state.attitude.yaw
            )
            self.position_controller.reset_all_pid(
                state.position.x, state.position.y, state.position.z
            )
            self.augmentor.reset()
            self.attitude_desired.yaw = state.attitude.yaw

        return self.control, dict(self._last_diag)


class HostAugmentedPWMPositionController:
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
        self._control_time_origin = 0.0

        # Known-stable baseline controller (same path as host_pid_pwm_position.py).
        self.controller_pid = ControllerPID()
        self.controller_pid.init()
        self.stabilizer_step = 1
        self.control = Control()

        # Augmented controller used after startup gate.
        self.controller_aug = AugmentedCascadeController(loop_hz=loop_hz)
        self._aug_active = False
        self._last_diag: Dict[str, float] = {
            "v_aug_x": 0.0,
            "v_aug_y": 0.0,
            "ef_x_pos": 0.0,
            "ef_x_vel": 0.0,
            "ef_y_pos": 0.0,
            "ef_y_vel": 0.0,
        }

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

        self.log_state = LogConfig(name="HostAugState", period_in_ms=10)
        self.log_state.add_variable("stateEstimate.vx", "FP16")
        self.log_state.add_variable("stateEstimate.vy", "FP16")
        self.log_state.add_variable("stateEstimate.vz", "FP16")
        self.log_state.add_variable("stateEstimate.ax", "FP16")
        self.log_state.add_variable("stateEstimate.ay", "FP16")
        self.log_state.add_variable("stateEstimate.az", "FP16")
        self.log_state.add_variable("stateEstimate.x", "FP16")
        self.log_state.add_variable("stateEstimate.y", "FP16")
        self.log_state.add_variable("stateEstimate.z", "FP16")
        self.log_state.add_variable("stateEstimate.roll", "FP16")
        self.log_state.add_variable("stateEstimate.pitch", "FP16")
        self.log_state.add_variable("stateEstimate.yaw", "FP16")
        self.log_state.add_variable("pm.vbat", "FP16")

        self.log_sensor = LogConfig(name="HostAugSensor", period_in_ms=10)
        self.log_sensor.add_variable("gyro.x", "float")
        self.log_sensor.add_variable("gyro.y", "float")
        self.log_sensor.add_variable("gyro.z", "float")
        self.log_sensor.add_variable("acc.x", "float")
        self.log_sensor.add_variable("acc.y", "float")
        self.log_sensor.add_variable("acc.z", "float")

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
                self.controller_pid.init()
                self.stabilizer_step = 1
                self.controller_aug.reset(self.cf_state)
                self._aug_active = False
                self._spinup(duration_s=1.0)
                self._control_time_origin = time.monotonic()

                if USE_SCRIPT_TRAJECTORY:
                    traj_duration = SCRIPT_TRAJECTORY[-1].time_s
                    print(f"Following script trajectory for {traj_duration:.1f}s")
                    self._control_for(
                        duration_s=traj_duration, follow_script_trajectory=True
                    )
                else:
                    self._control_for(duration_s=self.run_seconds)

                if self.do_land:
                    print(f"Landing target z={self.land_z:.2f} for {self.land_seconds:.1f}s")
                    self.cf_setpoint.position.z = self.land_z
                    self._control_for(
                        duration_s=self.land_seconds, follow_script_trajectory=False
                    )
            finally:
                self._stop_motors()
                if logs_started:
                    self.log_state.stop()
                    self.log_sensor.stop()

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
            elapsed = time.monotonic() - phase_start
            if follow_script_trajectory:
                x, y, z, yaw = _sample_trajectory(elapsed)
                self.cf_setpoint.position.x = x
                self.cf_setpoint.position.y = y
                self.cf_setpoint.position.z = z
                self.cf_setpoint.attitude.yaw = yaw

            sim_time = time.monotonic() - self._control_time_origin
            self._control_step(sim_time=sim_time)

            next_tick += self.loop_period
            sleep_time = next_tick - time.monotonic()
            if sleep_time > 0.0:
                time.sleep(sleep_time)
            else:
                next_tick = time.monotonic()

            if time.monotonic() >= next_print:
                diag = self._last_diag
                mode = "AUG" if self._aug_active else "PID"
                print(
                    "state "
                    f"x={self.cf_state.position.x:.2f} "
                    f"y={self.cf_state.position.y:.2f} "
                    f"z={self.cf_state.position.z:.2f} | "
                    f"sp=({self.cf_setpoint.position.x:.2f},"
                    f"{self.cf_setpoint.position.y:.2f},"
                    f"{self.cf_setpoint.position.z:.2f}) | "
                    f"mode={mode} "
                    f"thrust={self.control.thrust:.1f} "
                    f"roll={self.control.roll:.1f} "
                    f"pitch={self.control.pitch:.1f} "
                    f"yaw={self.control.yaw:.1f} | "
                    f"v_aug=({diag['v_aug_x']:.2f},{diag['v_aug_y']:.2f})"
                )
                next_print = time.monotonic() + 0.25

    def _control_step(self, sim_time: float):
        use_aug = AUG_ENABLED and (
            (not USE_BASELINE_BEFORE_AUG_START) or (sim_time >= AUG_START_TIME)
        )
        if use_aug:
            if not self._aug_active:
                # Re-seed augmented controller from current state for bumpless switch.
                self.controller_aug.reset(self.cf_state)
                self._aug_active = True

            control, diag = self.controller_aug.step(
                setpoint=self.cf_setpoint,
                sensors=self.cf_sensors,
                state=self.cf_state,
                sim_time=sim_time,
            )
            self.control = control
            self._last_diag = diag
        else:
            self._aug_active = False
            self.controller_pid.controller_pid(
                self.control,
                self.cf_setpoint,
                self.cf_sensors,
                self.cf_state,
                self.stabilizer_step,
            )
            self.stabilizer_step += 1
            self._last_diag["v_aug_x"] = 0.0
            self._last_diag["v_aug_y"] = 0.0

        raw = MotorThrust()
        self._power_distributor(self.control, raw)
        compensated = MotorThrust()
        self._battery_compensator(raw, compensated)
        pwm = MotorThrust()
        self._power_distribution_cap(compensated, pwm)

        self.motor_raw.send_motor_raw(
            int(pwm.motor_1),
            int(pwm.motor_2),
            int(pwm.motor_3),
            int(pwm.motor_4),
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
            VMOTOR2THRUST2 * VMOTOR2THRUST1
            - 3.0 * VMOTOR2THRUST3 * (VMOTOR2THRUST0 - thrust)
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
        motor_thrust_pwm.motor_1 = max(
            IDLE_THRUST, motor_thrust_bat_comp.motor_1 - reduction
        )
        motor_thrust_pwm.motor_2 = max(
            IDLE_THRUST, motor_thrust_bat_comp.motor_2 - reduction
        )
        motor_thrust_pwm.motor_3 = max(
            IDLE_THRUST, motor_thrust_bat_comp.motor_3 - reduction
        )
        motor_thrust_pwm.motor_4 = max(
            IDLE_THRUST, motor_thrust_bat_comp.motor_4 - reduction
        )

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
        self.cf_state.attitude.pitch = data["stateEstimate.pitch"]
        self.cf_state.attitude.roll = data["stateEstimate.roll"]
        self.cf_state.attitude.yaw = data["stateEstimate.yaw"]
        self.cf_vbat = data["pm.vbat"]
        self._have_state = True

    def _log_sensor_callback(self, _timestamp, data, _logconf):
        self.cf_sensors.gyro.x = data["gyro.x"]
        self.cf_sensors.gyro.y = data["gyro.y"]
        self.cf_sensors.gyro.z = data["gyro.z"]
        self.cf_sensors.acc.x = data["acc.x"]
        self.cf_sensors.acc.y = data["acc.y"]
        self.cf_sensors.acc.z = data["acc.z"]
        self._have_sensor = True


def parse_args():
    parser = argparse.ArgumentParser(
        description="Host-side augmented cascaded controller with raw motor PWM output."
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
        "--run-seconds", type=float, default=5.0, help="Main control duration"
    )
    parser.add_argument("--loop-hz", type=float, default=500.0, help="Host control loop rate")
    parser.add_argument("--land-z", type=float, default=0.05, help="Landing target z (m)")
    parser.add_argument("--land-seconds", type=float, default=3.0, help="Landing duration")
    parser.add_argument(
        "--no-land",
        action="store_true",
        help="Skip landing phase and stop motors directly after run phase",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    runner = HostAugmentedPWMPositionController(
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
        runner.run()
    except KeyboardInterrupt:
        print("Interrupted, stopping motors")
        runner._stop_motors()
        sys.exit(130)
    except Exception as exc:
        msg = str(exc)
        if "Resource busy" in msg or "Couldn't load link driver" in msg:
            print("Fatal error: Crazyradio is busy.")
            print("Close other tools using the radio (for example `cfclient`) and retry.")
            print(f"URI: {args.uri}")
            runner._stop_motors()
            sys.exit(1)
        print(f"Fatal error: {exc}")
        runner._stop_motors()
        raise


if __name__ == "__main__":
    main()
