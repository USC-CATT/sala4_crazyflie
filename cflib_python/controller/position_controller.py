"""
Position PID controller implementation for Crazyflie

Based on position_controller_pid.c from Crazyflie firmware
Copyright (C) 2016 Bitcraze AB
"""

import math
from typing import Optional, Tuple

from .pid_constants import *
from .pid import (
    PIDObject,
    constrain,
    filter_reset,
    pid_init,
    pid_reset,
    pid_set_desired,
    pid_update,
)
from .controller_types import (
    POSITION_RATE,
    Attitude,
    Axis3f,
    Position,
    Setpoint,
    StabMode,
    State,
)


class PIDAxis:
    """PID axis with mode tracking"""

    def __init__(self):
        self.pid = PIDObject()
        self.previous_mode = StabMode.MODE_DISABLE
        self.setpoint = 0.0
        self.output = 0.0


class PositionController:
    """Position controller using cascaded PID loops"""

    def __init__(self):
        """Initialize position controller"""
        # Update rate
        self.dt = 1.0 / POSITION_RATE

        # Velocity PIDs
        self.pid_vx = PIDAxis()
        self.pid_vy = PIDAxis()
        self.pid_vz = PIDAxis()

        # Position PIDs
        self.pid_x = PIDAxis()
        self.pid_y = PIDAxis()
        self.pid_z = PIDAxis()

        # Thrust parameters
        if CONFIG_CONTROLLER_PID_IMPROVED_BARO_Z_HOLD:
            self.thrust_base = PID_VEL_THRUST_BASE_BARO_Z_HOLD
        else:
            self.thrust_base = PID_VEL_THRUST_BASE
        self.thrust_min = PID_VEL_THRUST_MIN

        # Limits
        self.r_limit = PID_VEL_ROLL_MAX
        self.p_limit = PID_VEL_PITCH_MAX
        self.rp_limit_overhead = 1.10

        self.x_vel_max = PID_POS_VEL_X_MAX
        self.y_vel_max = PID_POS_VEL_Y_MAX
        self.z_vel_max = PID_POS_VEL_Z_MAX
        self.vel_max_overhead = 1.10

        self.thrust_scale = 1000.0

        # Filter parameters
        self.pos_filt_enable = PID_POS_XY_FILT_ENABLE
        self.vel_filt_enable = PID_VEL_XY_FILT_ENABLE
        self.pos_filt_cutoff = PID_POS_XY_FILT_CUTOFF
        self.vel_filt_cutoff = PID_VEL_XY_FILT_CUTOFF
        self.pos_z_filt_enable = PID_POS_Z_FILT_ENABLE
        self.vel_z_filt_enable = PID_VEL_Z_FILT_ENABLE
        self.pos_z_filt_cutoff = PID_POS_Z_FILT_CUTOFF
        if CONFIG_CONTROLLER_PID_IMPROVED_BARO_Z_HOLD:
            self.vel_z_filt_cutoff = PID_VEL_Z_FILT_CUTOFF_BARO_Z_HOLD
        else:
            self.vel_z_filt_cutoff = PID_VEL_Z_FILT_CUTOFF

        # State variables for logging
        self.state_body_x = 0.0
        self.state_body_y = 0.0
        self.state_body_vx = 0.0
        self.state_body_vy = 0.0

        # Initialize PIDs
        self._init_pids()

    def _init_pids(self):
        """Initialize all PID controllers"""
        # Position X PID
        self.pid_x.pid = PIDObject(
            kp=PID_POS_X_KP,
            ki=PID_POS_X_KI,
            kd=PID_POS_X_KD,
            kff=PID_POS_X_KFF,
            dt=self.dt,
            sample_rate=POSITION_RATE,
            cutoff_freq=self.pos_filt_cutoff,
            enable_filter=self.pos_filt_enable,
        )

        # Position Y PID
        self.pid_y.pid = PIDObject(
            kp=PID_POS_Y_KP,
            ki=PID_POS_Y_KI,
            kd=PID_POS_Y_KD,
            kff=PID_POS_Y_KFF,
            dt=self.dt,
            sample_rate=POSITION_RATE,
            cutoff_freq=self.pos_filt_cutoff,
            enable_filter=self.pos_filt_enable,
        )

        # Position Z PID
        self.pid_z.pid = PIDObject(
            kp=PID_POS_Z_KP,
            ki=PID_POS_Z_KI,
            kd=PID_POS_Z_KD,
            kff=PID_POS_Z_KFF,
            dt=self.dt,
            sample_rate=POSITION_RATE,
            cutoff_freq=self.pos_z_filt_cutoff,
            enable_filter=self.pos_z_filt_enable,
        )

        # Velocity X PID
        self.pid_vx.pid = PIDObject(
            kp=PID_VEL_X_KP,
            ki=PID_VEL_X_KI,
            kd=PID_VEL_X_KD,
            kff=PID_VEL_X_KFF,
            dt=self.dt,
            sample_rate=POSITION_RATE,
            cutoff_freq=self.vel_filt_cutoff,
            enable_filter=self.vel_filt_enable,
        )

        # Velocity Y PID
        self.pid_vy.pid = PIDObject(
            kp=PID_VEL_Y_KP,
            ki=PID_VEL_Y_KI,
            kd=PID_VEL_Y_KD,
            kff=PID_VEL_Y_KFF,
            dt=self.dt,
            sample_rate=POSITION_RATE,
            cutoff_freq=self.vel_filt_cutoff,
            enable_filter=self.vel_filt_enable,
        )

        # Velocity Z PID
        if CONFIG_CONTROLLER_PID_IMPROVED_BARO_Z_HOLD:
            kp = PID_VEL_Z_KP_BARO_Z_HOLD
            ki = PID_VEL_Z_KI_BARO_Z_HOLD
            kd = PID_VEL_Z_KD_BARO_Z_HOLD
            kff = PID_VEL_Z_KFF_BARO_Z_HOLD
        else:
            kp = PID_VEL_Z_KP
            ki = PID_VEL_Z_KI
            kd = PID_VEL_Z_KD
            kff = PID_VEL_Z_KFF

        self.pid_vz.pid = PIDObject(
            kp=kp,
            ki=ki,
            kd=kd,
            kff=kff,
            dt=self.dt,
            sample_rate=POSITION_RATE,
            cutoff_freq=self.vel_z_filt_cutoff,
            enable_filter=self.vel_z_filt_enable,
        )

    def init(self):
        """Initialize controller (C-style compatibility)"""
        self._init_pids()

    def _run_pid(
        self, input_val: float, axis: PIDAxis, setpoint: float, dt: float
    ) -> float:
        """
        Run PID update for an axis

        Args:
            input_val: Current measured value
            axis: PID axis object
            setpoint: Desired setpoint
            dt: Time step

        Returns:
            PID output
        """
        axis.setpoint = setpoint
        pid_set_desired(axis.pid, axis.setpoint)
        return pid_update(axis.pid, input_val, False)

    def position_controller(
        self, setpoint: Setpoint, state: State
    ) -> Tuple[float, Attitude]:
        """
        Position controller - outer loop

        Args:
            setpoint: Desired setpoint
            state: Current state

        Returns:
            Tuple of (thrust, attitude_desired)
        """
        # Set output limits
        self.pid_x.pid.output_limit = self.x_vel_max * self.vel_max_overhead
        self.pid_y.pid.output_limit = self.y_vel_max * self.vel_max_overhead
        self.pid_z.pid.output_limit = max(self.z_vel_max, 0.5) * self.vel_max_overhead

        # Transform to body frame
        cosyaw = math.cos(math.radians(state.attitude.yaw))
        sinyaw = math.sin(math.radians(state.attitude.yaw))

        setp_body_x = setpoint.position.x * cosyaw + setpoint.position.y * sinyaw
        setp_body_y = -setpoint.position.x * sinyaw + setpoint.position.y * cosyaw

        self.state_body_x = state.position.x * cosyaw + state.position.y * sinyaw
        self.state_body_y = -state.position.x * sinyaw + state.position.y * cosyaw

        globalvx = setpoint.velocity.x
        globalvy = setpoint.velocity.y

        # Calculate setpoint velocities
        setpoint_velocity = Axis3f(
            x=setpoint.velocity.x, y=setpoint.velocity.y, z=setpoint.velocity.z
        )

        # X axis
        if setpoint.mode.x == StabMode.MODE_ABS:
            setpoint_velocity.x = self._run_pid(
                self.state_body_x, self.pid_x, setp_body_x, self.dt
            )
        elif not setpoint.velocity_body:
            setpoint_velocity.x = globalvx * cosyaw + globalvy * sinyaw

        # Y axis
        if setpoint.mode.y == StabMode.MODE_ABS:
            setpoint_velocity.y = self._run_pid(
                self.state_body_y, self.pid_y, setp_body_y, self.dt
            )
        elif not setpoint.velocity_body:
            setpoint_velocity.y = globalvy * cosyaw - globalvx * sinyaw

        # Z axis
        if setpoint.mode.z == StabMode.MODE_ABS:
            setpoint_velocity.z = self._run_pid(
                state.position.z, self.pid_z, setpoint.position.z, self.dt
            )

        # Call velocity controller
        return self.velocity_controller(setpoint_velocity, state)

    def velocity_controller(
        self, setpoint_velocity: Axis3f, state: State
    ) -> Tuple[float, Attitude]:
        """
        Velocity controller - inner loop

        Args:
            setpoint_velocity: Desired velocity
            state: Current state

        Returns:
            Tuple of (thrust, attitude_desired)
        """
        # Set output limits
        self.pid_vx.pid.output_limit = self.p_limit * self.rp_limit_overhead
        self.pid_vy.pid.output_limit = self.r_limit * self.rp_limit_overhead
        self.pid_vz.pid.output_limit = UINT16_MAX / 2 / self.thrust_scale

        # Transform velocity to body frame
        cosyaw = math.cos(math.radians(state.attitude.yaw))
        sinyaw = math.sin(math.radians(state.attitude.yaw))
        self.state_body_vx = state.velocity.x * cosyaw + state.velocity.y * sinyaw
        self.state_body_vy = -state.velocity.x * sinyaw + state.velocity.y * cosyaw

        # Create attitude output
        attitude = Attitude()

        # Roll and Pitch
        attitude.pitch = -self._run_pid(
            self.state_body_vx, self.pid_vx, setpoint_velocity.x, self.dt
        )
        attitude.roll = -self._run_pid(
            self.state_body_vy, self.pid_vy, setpoint_velocity.y, self.dt
        )

        # Constrain roll and pitch
        attitude.roll = constrain(attitude.roll, -self.r_limit, self.r_limit)
        attitude.pitch = constrain(attitude.pitch, -self.p_limit, self.p_limit)

        # Thrust
        thrust_raw = self._run_pid(
            state.velocity.z, self.pid_vz, setpoint_velocity.z, self.dt
        )

        # Scale thrust and add feed forward term
        thrust = thrust_raw * self.thrust_scale + self.thrust_base

        # Check for minimum thrust
        if thrust < self.thrust_min:
            thrust = self.thrust_min

        # Saturate thrust
        thrust = constrain(thrust, 0, UINT16_MAX)

        return thrust, attitude

    def reset_all_pid(self, x_actual: float, y_actual: float, z_actual: float):
        """
        Reset all PIDs

        Args:
            x_actual: Current X position
            y_actual: Current Y position
            z_actual: Current Z position
        """
        pid_reset(self.pid_x.pid, x_actual)
        pid_reset(self.pid_y.pid, y_actual)
        pid_reset(self.pid_z.pid, z_actual)
        pid_reset(self.pid_vx.pid, 0.0)
        pid_reset(self.pid_vy.pid, 0.0)
        pid_reset(self.pid_vz.pid, 0.0)

    def reset_all_filters(self):
        """Reset all filter states"""
        filter_reset(
            self.pid_x.pid, POSITION_RATE, self.pos_filt_cutoff, self.pos_filt_enable
        )
        filter_reset(
            self.pid_y.pid, POSITION_RATE, self.pos_filt_cutoff, self.pos_filt_enable
        )
        filter_reset(
            self.pid_z.pid,
            POSITION_RATE,
            self.pos_z_filt_cutoff,
            self.pos_z_filt_enable,
        )
        filter_reset(
            self.pid_vx.pid, POSITION_RATE, self.vel_filt_cutoff, self.vel_filt_enable
        )
        filter_reset(
            self.pid_vy.pid, POSITION_RATE, self.vel_filt_cutoff, self.vel_filt_enable
        )
        filter_reset(
            self.pid_vz.pid,
            POSITION_RATE,
            self.vel_z_filt_cutoff,
            self.vel_z_filt_enable,
        )


# C-style function interfaces for compatibility
_position_controller_instance: Optional[PositionController] = None


def position_controller_init():
    """Initialize position controller (C-style function)"""
    global _position_controller_instance
    _position_controller_instance = PositionController()


def position_controller(setpoint: Setpoint, state: State) -> Tuple[float, Attitude]:
    """Run position controller (C-style function)"""
    global _position_controller_instance
    if _position_controller_instance is not None:
        return _position_controller_instance.position_controller(setpoint, state)
    return 0.0, Attitude()


def velocity_controller(
    setpoint_velocity: Axis3f, state: State
) -> Tuple[float, Attitude]:
    """Run velocity controller (C-style function)"""
    global _position_controller_instance
    if _position_controller_instance is not None:
        return _position_controller_instance.velocity_controller(
            setpoint_velocity, state
        )
    return 0.0, Attitude()


def position_controller_reset_all_pid(
    x_actual: float, y_actual: float, z_actual: float
):
    """Reset all PIDs (C-style function)"""
    global _position_controller_instance
    if _position_controller_instance is not None:
        _position_controller_instance.reset_all_pid(x_actual, y_actual, z_actual)


def position_controller_reset_all_filters():
    """Reset all filters (C-style function)"""
    global _position_controller_instance
    if _position_controller_instance is not None:
        _position_controller_instance.reset_all_filters()
