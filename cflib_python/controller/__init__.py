"""
Crazyflie PID Controller Package

Python implementation of the Crazyflie firmware PID controller.
Based on the Crazyflie firmware by Bitcraze AB.

This package provides:
- PID controller base class
- Attitude controller (inner and outer loops)
- Position controller (velocity and position control)
- Main controller integrating all components
- Data types and structures
- Default constants and parameters

Usage:
    from controller import ControllerPID, Setpoint, State, Control, SensorData

    # Create controller
    controller = ControllerPID()
    controller.init()

    # Create inputs
    setpoint = Setpoint()
    state = State()
    sensors = SensorData()
    control = Control()

    # Run controller
    controller.controller_pid(control, setpoint, sensors, state, stabilizer_step=0)
"""

from .attitude_controller import (
    AttitudeController,
    attitude_controller_correct_attitude_pid,
    attitude_controller_correct_rate_pid,
    attitude_controller_get_actuator_output,
    attitude_controller_get_yaw_max_delta,
    attitude_controller_init,
    attitude_controller_reset_all_pid,
    attitude_controller_reset_pitch_attitude_pid,
    attitude_controller_reset_roll_attitude_pid,
    attitude_controller_test,
)
from .pid_constants import *
from .controller_pid import (
    ControllerPID,
    controller_pid,
    controller_pid_init,
    controller_pid_test,
)
from .pid import (
    PIDObject,
    constrain,
    filter_reset,
    pid_init,
    pid_reset,
    pid_set_desired,
    pid_set_integral_limit,
    pid_update,
)
from .position_controller import (
    PositionController,
    position_controller,
    position_controller_init,
    position_controller_reset_all_filters,
    position_controller_reset_all_pid,
    velocity_controller,
)
from .controller_types import (
    ATTITUDE_RATE,
    POSITION_RATE,
    AccData,
    Attitude,
    AttitudeRate,
    Axis3f,
    Control,
    ControlMode,
    GyroData,
    Position,
    Quaternion,
    SensorData,
    Setpoint,
    SetpointMode,
    StabMode,
    State,
    Velocity,
    cap_angle,
    degrees,
    quat2rpy,
    radians,
    rate_do_execute,
)

__all__ = [
    # Main controller
    "ControllerPID",
    "controller_pid_init",
    "controller_pid_test",
    "controller_pid",
    # Attitude controller
    "AttitudeController",
    "attitude_controller_init",
    "attitude_controller_test",
    "attitude_controller_correct_rate_pid",
    "attitude_controller_correct_attitude_pid",
    "attitude_controller_reset_roll_attitude_pid",
    "attitude_controller_reset_pitch_attitude_pid",
    "attitude_controller_reset_all_pid",
    "attitude_controller_get_actuator_output",
    "attitude_controller_get_yaw_max_delta",
    # Position controller
    "PositionController",
    "position_controller_init",
    "position_controller",
    "velocity_controller",
    "position_controller_reset_all_pid",
    "position_controller_reset_all_filters",
    # PID base
    "PIDObject",
    "pid_init",
    "pid_set_desired",
    "pid_update",
    "pid_reset",
    "pid_set_integral_limit",
    "filter_reset",
    "constrain",
    # Types
    "State",
    "Setpoint",
    "SetpointMode",
    "Control",
    "SensorData",
    "Attitude",
    "AttitudeRate",
    "Position",
    "Velocity",
    "Axis3f",
    "Quaternion",
    "GyroData",
    "AccData",
    "StabMode",
    "ControlMode",
    "ATTITUDE_RATE",
    "POSITION_RATE",
    "rate_do_execute",
    "cap_angle",
    "degrees",
    "radians",
    "quat2rpy",
]

__version__ = "1.0.0"
__author__ = "Based on Bitcraze AB Crazyflie Firmware"
