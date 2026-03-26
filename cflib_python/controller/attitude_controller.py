"""
Attitude PID controller implementation for Crazyflie

Based on attitude_pid_controller.c from Crazyflie firmware
Copyright (C) 2011-2012 Bitcraze AB
"""

import math
from typing import Optional, Tuple

from .pid_constants import *
from .pid import (
    PIDObject,
    pid_init,
    pid_reset,
    pid_set_desired,
    pid_set_integral_limit,
    pid_update,
)
from .controller_types import ATTITUDE_RATE


class AttitudeController:
    """Attitude controller using cascaded PID loops"""

    def __init__(self, update_dt: float = 1.0 / ATTITUDE_RATE):
        """
        Initialize attitude controller

        Args:
            update_dt: Update time step (default: 1/500 Hz)
        """
        self.update_dt = update_dt
        self.is_init = False

        # Attitude rate PIDs
        self.pid_roll_rate = PIDObject(
            kp=PID_ROLL_RATE_KP,
            ki=PID_ROLL_RATE_KI,
            kd=PID_ROLL_RATE_KD,
            kff=PID_ROLL_RATE_KFF,
            dt=update_dt,
            sample_rate=ATTITUDE_RATE,
            cutoff_freq=ATTITUDE_ROLL_RATE_LPF_CUTOFF_FREQ,
            enable_filter=ATTITUDE_RATE_LPF_ENABLE,
        )

        self.pid_pitch_rate = PIDObject(
            kp=PID_PITCH_RATE_KP,
            ki=PID_PITCH_RATE_KI,
            kd=PID_PITCH_RATE_KD,
            kff=PID_PITCH_RATE_KFF,
            dt=update_dt,
            sample_rate=ATTITUDE_RATE,
            cutoff_freq=ATTITUDE_PITCH_RATE_LPF_CUTOFF_FREQ,
            enable_filter=ATTITUDE_RATE_LPF_ENABLE,
        )

        self.pid_yaw_rate = PIDObject(
            kp=PID_YAW_RATE_KP,
            ki=PID_YAW_RATE_KI,
            kd=PID_YAW_RATE_KD,
            kff=PID_YAW_RATE_KFF,
            dt=update_dt,
            sample_rate=ATTITUDE_RATE,
            cutoff_freq=ATTITUDE_YAW_RATE_LPF_CUTOFF_FREQ,
            enable_filter=ATTITUDE_RATE_LPF_ENABLE,
        )

        # Attitude PIDs
        self.pid_roll = PIDObject(
            kp=PID_ROLL_KP,
            ki=PID_ROLL_KI,
            kd=PID_ROLL_KD,
            kff=PID_ROLL_KFF,
            dt=update_dt,
            sample_rate=ATTITUDE_RATE,
            cutoff_freq=ATTITUDE_LPF_CUTOFF_FREQ,
            enable_filter=ATTITUDE_LPF_ENABLE,
        )

        self.pid_pitch = PIDObject(
            kp=PID_PITCH_KP,
            ki=PID_PITCH_KI,
            kd=PID_PITCH_KD,
            kff=PID_PITCH_KFF,
            dt=update_dt,
            sample_rate=ATTITUDE_RATE,
            cutoff_freq=ATTITUDE_LPF_CUTOFF_FREQ,
            enable_filter=ATTITUDE_LPF_ENABLE,
        )

        self.pid_yaw = PIDObject(
            kp=PID_YAW_KP,
            ki=PID_YAW_KI,
            kd=PID_YAW_KD,
            kff=PID_YAW_KFF,
            dt=update_dt,
            sample_rate=ATTITUDE_RATE,
            cutoff_freq=ATTITUDE_LPF_CUTOFF_FREQ,
            enable_filter=ATTITUDE_LPF_ENABLE,
        )

        # Set integral limits
        pid_set_integral_limit(self.pid_roll_rate, PID_ROLL_RATE_INTEGRATION_LIMIT)
        pid_set_integral_limit(self.pid_pitch_rate, PID_PITCH_RATE_INTEGRATION_LIMIT)
        pid_set_integral_limit(self.pid_yaw_rate, PID_YAW_RATE_INTEGRATION_LIMIT)

        pid_set_integral_limit(self.pid_roll, PID_ROLL_INTEGRATION_LIMIT)
        pid_set_integral_limit(self.pid_pitch, PID_PITCH_INTEGRATION_LIMIT)
        pid_set_integral_limit(self.pid_yaw, PID_YAW_INTEGRATION_LIMIT)

        # Output values
        self.roll_output = 0
        self.pitch_output = 0
        self.yaw_output = 0

        # Yaw max delta parameter
        self.yaw_max_delta = YAW_MAX_DELTA

        self.is_init = True

    def init(self, update_dt: float):
        """
        Initialize controller (alternative initialization method)

        Args:
            update_dt: Update time step
        """
        self.__init__(update_dt)

    def test(self) -> bool:
        """
        Test if controller is initialized

        Returns:
            True if initialized
        """
        return self.is_init

    def correct_rate_pid(
        self,
        roll_rate_actual: float,
        pitch_rate_actual: float,
        yaw_rate_actual: float,
        roll_rate_desired: float,
        pitch_rate_desired: float,
        yaw_rate_desired: float,
    ):
        """
        Update rate PIDs (inner loop)

        Args:
            roll_rate_actual: Current roll rate (deg/s)
            pitch_rate_actual: Current pitch rate (deg/s)
            yaw_rate_actual: Current yaw rate (deg/s)
            roll_rate_desired: Desired roll rate (deg/s)
            pitch_rate_desired: Desired pitch rate (deg/s)
            yaw_rate_desired: Desired yaw rate (deg/s)
        """
        pid_set_desired(self.pid_roll_rate, roll_rate_desired)
        self.roll_output = self._saturate_signed_int16(
            pid_update(self.pid_roll_rate, roll_rate_actual, False)
        )

        pid_set_desired(self.pid_pitch_rate, pitch_rate_desired)
        self.pitch_output = self._saturate_signed_int16(
            pid_update(self.pid_pitch_rate, pitch_rate_actual, False)
        )

        pid_set_desired(self.pid_yaw_rate, yaw_rate_desired)
        self.yaw_output = self._saturate_signed_int16(
            pid_update(self.pid_yaw_rate, yaw_rate_actual, False)
        )

    def correct_attitude_pid(
        self,
        euler_roll_actual: float,
        euler_pitch_actual: float,
        euler_yaw_actual: float,
        euler_roll_desired: float,
        euler_pitch_desired: float,
        euler_yaw_desired: float,
    ) -> Tuple[float, float, float]:
        """
        Update attitude PIDs (outer loop)

        Args:
            euler_roll_actual: Current roll (degrees)
            euler_pitch_actual: Current pitch (degrees)
            euler_yaw_actual: Current yaw (degrees)
            euler_roll_desired: Desired roll (degrees)
            euler_pitch_desired: Desired pitch (degrees)
            euler_yaw_desired: Desired yaw (degrees)

        Returns:
            Tuple of (roll_rate_desired, pitch_rate_desired, yaw_rate_desired)
        """
        # Update PID for roll axis
        pid_set_desired(self.pid_roll, euler_roll_desired)
        roll_rate_desired = pid_update(self.pid_roll, euler_roll_actual, False)

        # Update PID for pitch axis
        pid_set_desired(self.pid_pitch, euler_pitch_desired)
        pitch_rate_desired = pid_update(self.pid_pitch, euler_pitch_actual, False)

        # Update PID for yaw axis (with angle wrapping)
        pid_set_desired(self.pid_yaw, euler_yaw_desired)
        yaw_rate_desired = pid_update(self.pid_yaw, euler_yaw_actual, True)

        return roll_rate_desired, pitch_rate_desired, yaw_rate_desired

    def reset_roll_attitude_pid(self, roll_actual: float):
        """
        Reset roll attitude PID

        Args:
            roll_actual: Current roll value
        """
        pid_reset(self.pid_roll, roll_actual)

    def reset_pitch_attitude_pid(self, pitch_actual: float):
        """
        Reset pitch attitude PID

        Args:
            pitch_actual: Current pitch value
        """
        pid_reset(self.pid_pitch, pitch_actual)

    def reset_all_pid(self, roll_actual: float, pitch_actual: float, yaw_actual: float):
        """
        Reset all PIDs

        Args:
            roll_actual: Current roll
            pitch_actual: Current pitch
            yaw_actual: Current yaw
        """
        pid_reset(self.pid_roll, roll_actual)
        pid_reset(self.pid_pitch, pitch_actual)
        pid_reset(self.pid_yaw, yaw_actual)
        pid_reset(self.pid_roll_rate, 0.0)
        pid_reset(self.pid_pitch_rate, 0.0)
        pid_reset(self.pid_yaw_rate, 0.0)

    def get_actuator_output(self) -> Tuple[int, int, int]:
        """
        Get actuator output values

        Returns:
            Tuple of (roll, pitch, yaw) as int16 values
        """
        return self.roll_output, self.pitch_output, self.yaw_output

    def get_yaw_max_delta(self) -> float:
        """
        Get yaw max delta parameter

        Returns:
            Yaw max delta value
        """
        return self.yaw_max_delta

    def set_yaw_max_delta(self, value: float):
        """
        Set yaw max delta parameter

        Args:
            value: New yaw max delta
        """
        self.yaw_max_delta = value

    @staticmethod
    def _saturate_signed_int16(value: float) -> int:
        """
        Saturate float to signed int16 range

        Args:
            value: Float value

        Returns:
            Saturated int16 value
        """
        INT16_MAX = 32767
        if value > INT16_MAX:
            return INT16_MAX
        elif value < -INT16_MAX:
            return -INT16_MAX
        else:
            return int(value)


# C-style function interfaces for compatibility
_attitude_controller_instance: Optional[AttitudeController] = None


def attitude_controller_init(update_dt: float):
    """Initialize attitude controller (C-style function)"""
    global _attitude_controller_instance
    _attitude_controller_instance = AttitudeController(update_dt)


def attitude_controller_test() -> bool:
    """Test if controller is initialized (C-style function)"""
    global _attitude_controller_instance
    if _attitude_controller_instance is None:
        return False
    return _attitude_controller_instance.test()


def attitude_controller_correct_rate_pid(
    roll_rate_actual: float,
    pitch_rate_actual: float,
    yaw_rate_actual: float,
    roll_rate_desired: float,
    pitch_rate_desired: float,
    yaw_rate_desired: float,
):
    """Update rate PIDs (C-style function)"""
    global _attitude_controller_instance
    if _attitude_controller_instance is not None:
        _attitude_controller_instance.correct_rate_pid(
            roll_rate_actual,
            pitch_rate_actual,
            yaw_rate_actual,
            roll_rate_desired,
            pitch_rate_desired,
            yaw_rate_desired,
        )


def attitude_controller_correct_attitude_pid(
    euler_roll_actual: float,
    euler_pitch_actual: float,
    euler_yaw_actual: float,
    euler_roll_desired: float,
    euler_pitch_desired: float,
    euler_yaw_desired: float,
) -> Tuple[float, float, float]:
    """Update attitude PIDs (C-style function)"""
    global _attitude_controller_instance
    if _attitude_controller_instance is not None:
        return _attitude_controller_instance.correct_attitude_pid(
            euler_roll_actual,
            euler_pitch_actual,
            euler_yaw_actual,
            euler_roll_desired,
            euler_pitch_desired,
            euler_yaw_desired,
        )
    return 0.0, 0.0, 0.0


def attitude_controller_reset_roll_attitude_pid(roll_actual: float):
    """Reset roll attitude PID (C-style function)"""
    global _attitude_controller_instance
    if _attitude_controller_instance is not None:
        _attitude_controller_instance.reset_roll_attitude_pid(roll_actual)


def attitude_controller_reset_pitch_attitude_pid(pitch_actual: float):
    """Reset pitch attitude PID (C-style function)"""
    global _attitude_controller_instance
    if _attitude_controller_instance is not None:
        _attitude_controller_instance.reset_pitch_attitude_pid(pitch_actual)


def attitude_controller_reset_all_pid(
    roll_actual: float, pitch_actual: float, yaw_actual: float
):
    """Reset all PIDs (C-style function)"""
    global _attitude_controller_instance
    if _attitude_controller_instance is not None:
        _attitude_controller_instance.reset_all_pid(
            roll_actual, pitch_actual, yaw_actual
        )


def attitude_controller_get_actuator_output() -> Tuple[int, int, int]:
    """Get actuator output (C-style function)"""
    global _attitude_controller_instance
    if _attitude_controller_instance is not None:
        return _attitude_controller_instance.get_actuator_output()
    return 0, 0, 0


def attitude_controller_get_yaw_max_delta() -> float:
    """Get yaw max delta (C-style function)"""
    global _attitude_controller_instance
    if _attitude_controller_instance is not None:
        return _attitude_controller_instance.get_yaw_max_delta()
    return 0.0
