"""
Main PID controller implementation for Crazyflie

Based on controller_pid.c from Crazyflie firmware
Copyright (C) 2011-2012 Bitcraze AB
"""

from typing import Optional

from .attitude_controller import AttitudeController
from .position_controller import PositionController
from .controller_types import (
    ATTITUDE_RATE,
    POSITION_RATE,
    Attitude,
    Control,
    ControlMode,
    Quaternion,
    SensorData,
    Setpoint,
    StabMode,
    State,
    cap_angle,
    degrees,
    quat2rpy,
    radians,
    rate_do_execute,
)


class ControllerPID:
    """Main PID controller for Crazyflie"""

    def __init__(self):
        """Initialize PID controller"""
        self.attitude_desired = Attitude()
        self.rate_desired = Attitude()
        self.actuator_thrust = 0.0

        # Logging variables
        self.cmd_thrust = 0.0
        self.cmd_roll = 0.0
        self.cmd_pitch = 0.0
        self.cmd_yaw = 0.0
        self.r_roll = 0.0
        self.r_pitch = 0.0
        self.r_yaw = 0.0
        self.accelz = 0.0

        # Sub-controllers
        self.attitude_controller: Optional[AttitudeController] = None
        self.position_controller: Optional[PositionController] = None

        # Update rate
        self.attitude_update_dt = 1.0 / ATTITUDE_RATE

    def init(self):
        """Initialize the controller and sub-controllers"""
        self.attitude_controller = AttitudeController(self.attitude_update_dt)
        self.position_controller = PositionController()

    def test(self) -> bool:
        """
        Test if controller is properly initialized

        Returns:
            True if initialized
        """
        if self.attitude_controller is None or self.position_controller is None:
            return False
        return self.attitude_controller.test()

    def controller_pid(
        self,
        control: Control,
        setpoint: Setpoint,
        sensors: SensorData,
        state: State,
        stabilizer_step: int,
    ):
        """
        Main PID controller update

        Args:
            control: Control output (modified in place)
            setpoint: Desired setpoint
            sensors: Sensor data
            state: Current state estimate
            stabilizer_step: Current stabilizer step for rate limiting
        """
        if self.attitude_controller is None or self.position_controller is None:
            return

        control.control_mode = ControlMode.CONTROL_MODE_LEGACY

        # Update at ATTITUDE_RATE
        if rate_do_execute(ATTITUDE_RATE, stabilizer_step):
            # Rate-controlled YAW is moving YAW angle setpoint
            if setpoint.mode.yaw == StabMode.MODE_VELOCITY:
                self.attitude_desired.yaw = cap_angle(
                    self.attitude_desired.yaw
                    + setpoint.attitude_rate.yaw * self.attitude_update_dt
                )

                yaw_max_delta = self.attitude_controller.get_yaw_max_delta()
                if yaw_max_delta != 0.0:
                    delta = cap_angle(self.attitude_desired.yaw - state.attitude.yaw)
                    # Keep the yaw setpoint within +/- yawMaxDelta from current yaw
                    if delta > yaw_max_delta:
                        self.attitude_desired.yaw = state.attitude.yaw + yaw_max_delta
                    elif delta < -yaw_max_delta:
                        self.attitude_desired.yaw = state.attitude.yaw - yaw_max_delta

            elif setpoint.mode.yaw == StabMode.MODE_ABS:
                self.attitude_desired.yaw = setpoint.attitude.yaw

            elif setpoint.mode.quat == StabMode.MODE_ABS:
                # Convert quaternion to RPY
                setpoint_quat = Quaternion(
                    w=setpoint.attitude_quaternion.w,
                    x=setpoint.attitude_quaternion.x,
                    y=setpoint.attitude_quaternion.y,
                    z=setpoint.attitude_quaternion.z,
                )
                rpy = quat2rpy(setpoint_quat)
                self.attitude_desired.yaw = degrees(rpy.z)

            self.attitude_desired.yaw = cap_angle(self.attitude_desired.yaw)

        # Update position controller at POSITION_RATE
        if rate_do_execute(POSITION_RATE, stabilizer_step):
            thrust, attitude_desired = self.position_controller.position_controller(
                setpoint, state
            )
            self.actuator_thrust = thrust
            # Update attitude desired from position controller
            self.attitude_desired.roll = attitude_desired.roll
            self.attitude_desired.pitch = attitude_desired.pitch
            # Note: yaw is handled separately above

        # Update attitude controller at ATTITUDE_RATE
        if rate_do_execute(ATTITUDE_RATE, stabilizer_step):
            # Switch between manual and automatic position control
            if setpoint.mode.z == StabMode.MODE_DISABLE:
                self.actuator_thrust = setpoint.thrust

            if (
                setpoint.mode.x == StabMode.MODE_DISABLE
                or setpoint.mode.y == StabMode.MODE_DISABLE
            ):
                self.attitude_desired.roll = setpoint.attitude.roll
                self.attitude_desired.pitch = setpoint.attitude.pitch

            # Correct attitude PID
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

            # For roll and pitch, if velocity mode, overwrite rateDesired with setpoint
            # value. Also reset the PID to avoid error buildup
            if setpoint.mode.roll == StabMode.MODE_VELOCITY:
                self.rate_desired.roll = setpoint.attitude_rate.roll
                self.attitude_controller.reset_roll_attitude_pid(state.attitude.roll)

            if setpoint.mode.pitch == StabMode.MODE_VELOCITY:
                self.rate_desired.pitch = setpoint.attitude_rate.pitch
                self.attitude_controller.reset_pitch_attitude_pid(state.attitude.pitch)

            # Correct rate PID (note: gyro.y is negated)
            self.attitude_controller.correct_rate_pid(
                sensors.gyro.x,
                -sensors.gyro.y,
                sensors.gyro.z,
                self.rate_desired.roll,
                self.rate_desired.pitch,
                self.rate_desired.yaw,
            )

            # Get actuator output
            roll_out, pitch_out, yaw_out = (
                self.attitude_controller.get_actuator_output()
            )

            control.roll = roll_out
            control.pitch = pitch_out
            control.yaw = -yaw_out  # Negate yaw output

            # Update logging variables
            self.cmd_thrust = control.thrust
            self.cmd_roll = control.roll
            self.cmd_pitch = control.pitch
            self.cmd_yaw = control.yaw
            self.r_roll = radians(sensors.gyro.x)
            self.r_pitch = -radians(sensors.gyro.y)
            self.r_yaw = radians(sensors.gyro.z)
            self.accelz = sensors.acc.z

        # Set thrust
        control.thrust = self.actuator_thrust

        # Handle zero thrust case (safety)
        if control.thrust == 0:
            control.thrust = 0
            control.roll = 0
            control.pitch = 0
            control.yaw = 0

            self.cmd_thrust = control.thrust
            self.cmd_roll = control.roll
            self.cmd_pitch = control.pitch
            self.cmd_yaw = control.yaw

            # Reset all PIDs
            self.attitude_controller.reset_all_pid(
                state.attitude.roll, state.attitude.pitch, state.attitude.yaw
            )
            self.position_controller.reset_all_pid(
                state.position.x, state.position.y, state.position.z
            )

            # Reset the calculated YAW angle for rate control
            self.attitude_desired.yaw = state.attitude.yaw

    def get_logging_data(self):
        """
        Get logging data

        Returns:
            Dictionary with logging variables
        """
        return {
            "cmd_thrust": self.cmd_thrust,
            "cmd_roll": self.cmd_roll,
            "cmd_pitch": self.cmd_pitch,
            "cmd_yaw": self.cmd_yaw,
            "r_roll": self.r_roll,
            "r_pitch": self.r_pitch,
            "r_yaw": self.r_yaw,
            "accelz": self.accelz,
            "actuator_thrust": self.actuator_thrust,
            "attitude_desired_roll": self.attitude_desired.roll,
            "attitude_desired_pitch": self.attitude_desired.pitch,
            "attitude_desired_yaw": self.attitude_desired.yaw,
            "rate_desired_roll": self.rate_desired.roll,
            "rate_desired_pitch": self.rate_desired.pitch,
            "rate_desired_yaw": self.rate_desired.yaw,
        }


# C-style function interfaces for compatibility
_controller_pid_instance: Optional[ControllerPID] = None


def controller_pid_init():
    """Initialize controller (C-style function)"""
    global _controller_pid_instance
    _controller_pid_instance = ControllerPID()
    _controller_pid_instance.init()


def controller_pid_test() -> bool:
    """Test if controller is initialized (C-style function)"""
    global _controller_pid_instance
    if _controller_pid_instance is None:
        return False
    return _controller_pid_instance.test()


def controller_pid(
    control: Control,
    setpoint: Setpoint,
    sensors: SensorData,
    state: State,
    stabilizer_step: int,
):
    """Run controller (C-style function)"""
    global _controller_pid_instance
    if _controller_pid_instance is not None:
        _controller_pid_instance.controller_pid(
            control, setpoint, sensors, state, stabilizer_step
        )
