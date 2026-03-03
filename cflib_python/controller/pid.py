"""
PID controller implementation for Crazyflie

Based on the Crazyflie firmware PID controller.
Copyright (C) 2011-2012 Bitcraze AB
"""

import math
from typing import Optional


class PIDObject:
    """PID controller with feedforward and low-pass filtering"""

    def __init__(
        self,
        desired: float = 0.0,
        kp: float = 0.0,
        ki: float = 0.0,
        kd: float = 0.0,
        kff: float = 0.0,
        dt: float = 0.002,
        sample_rate: int = 500,
        cutoff_freq: float = 20.0,
        enable_filter: bool = True,
    ):
        """
        Initialize PID controller

        Args:
            desired: Desired setpoint
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            kff: Feedforward gain
            dt: Time step (seconds)
            sample_rate: Update rate (Hz)
            cutoff_freq: Low-pass filter cutoff frequency (Hz)
            enable_filter: Enable/disable low-pass filtering
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kff = kff
        self.dt = dt

        self.desired = desired
        self.error = 0.0
        self.prev_error = 0.0
        self.integ = 0.0
        self.deriv = 0.0

        # Output components
        self.outP = 0.0
        self.outI = 0.0
        self.outD = 0.0
        self.outFF = 0.0

        # Limits
        self.i_limit = float("inf")
        self.output_limit = float("inf")

        # Low-pass filter parameters
        self.enable_d_filter = enable_filter
        self.filter_cutoff = cutoff_freq
        self.filter_sample_rate = sample_rate

        # Initialize filter state
        self._init_filter()

    def _init_filter(self):
        """Initialize low-pass filter coefficients"""
        if self.enable_d_filter and self.filter_cutoff > 0:
            # Calculate filter coefficient for first-order low-pass filter
            # a = dt / (dt + 1/(2*pi*fc))
            omega = 2.0 * math.pi * self.filter_cutoff
            dt = 1.0 / self.filter_sample_rate
            self.filter_alpha = omega * dt / (omega * dt + 1.0)
        else:
            self.filter_alpha = 1.0

        self.filter_state = 0.0

    def set_desired(self, desired: float):
        """Set the desired setpoint"""
        self.desired = desired

    def set_integral_limit(self, limit: float):
        """Set the integral windup limit"""
        self.i_limit = limit

    def set_output_limit(self, limit: float):
        """Set the output limit"""
        self.output_limit = limit

    def reset(self, actual: float):
        """
        Reset PID state

        Args:
            actual: Current value to reset integral to
        """
        self.integ = actual
        self.error = 0.0
        self.prev_error = 0.0
        self.deriv = 0.0
        self.outP = 0.0
        self.outI = 0.0
        self.outD = 0.0
        self.outFF = 0.0
        self.filter_state = 0.0

    def update(self, measured: float, update_error: bool = True) -> float:
        """
        Update PID controller

        Args:
            measured: Current measured value
            update_error: If True, wraps error to [-180, 180] for angles

        Returns:
            PID output value
        """
        # Calculate error
        self.error = self.desired - measured

        # Wrap error for angles if requested (yaw control)
        if update_error:
            while self.error > 180.0:
                self.error -= 360.0
            while self.error < -180.0:
                self.error += 360.0

        # Proportional term
        self.outP = self.kp * self.error

        # Integral term with anti-windup
        self.integ += self.error * self.dt

        # Clamp integral
        if self.integ > self.i_limit:
            self.integ = self.i_limit
        elif self.integ < -self.i_limit:
            self.integ = -self.i_limit

        self.outI = self.ki * self.integ

        # Derivative term with low-pass filter
        raw_deriv = (self.error - self.prev_error) / self.dt

        if self.enable_d_filter:
            # Apply first-order low-pass filter
            self.filter_state = (
                self.filter_alpha * raw_deriv
                + (1.0 - self.filter_alpha) * self.filter_state
            )
            self.deriv = self.filter_state
        else:
            self.deriv = raw_deriv

        self.outD = self.kd * self.deriv

        # Feedforward term
        self.outFF = self.kff * self.desired

        # Save error for next iteration
        self.prev_error = self.error

        # Calculate total output
        output = self.outP + self.outI + self.outD + self.outFF

        # Apply output limit
        if output > self.output_limit:
            output = self.output_limit
        elif output < -self.output_limit:
            output = -self.output_limit

        return output

    def get_p_out(self) -> float:
        """Get proportional output"""
        return self.outP

    def get_i_out(self) -> float:
        """Get integral output"""
        return self.outI

    def get_d_out(self) -> float:
        """Get derivative output"""
        return self.outD

    def get_ff_out(self) -> float:
        """Get feedforward output"""
        return self.outFF


def pid_init(
    pid: PIDObject,
    desired: float,
    kp: float,
    ki: float,
    kd: float,
    kff: float,
    dt: float,
    sample_rate: int,
    cutoff_freq: float,
    enable_filter: bool,
):
    """
    Initialize a PID object (C-style function interface)

    Args:
        pid: PID object to initialize
        desired: Initial setpoint
        kp: Proportional gain
        ki: Integral gain
        kd: Derivative gain
        kff: Feedforward gain
        dt: Time step
        sample_rate: Update rate (Hz)
        cutoff_freq: Filter cutoff frequency
        enable_filter: Enable filtering
    """
    pid.kp = kp
    pid.ki = ki
    pid.kd = kd
    pid.kff = kff
    pid.dt = dt
    pid.desired = desired
    pid.filter_cutoff = cutoff_freq
    pid.filter_sample_rate = sample_rate
    pid.enable_d_filter = enable_filter
    pid._init_filter()


def pid_set_desired(pid: PIDObject, desired: float):
    """Set desired setpoint (C-style function)"""
    pid.set_desired(desired)


def pid_update(pid: PIDObject, measured: float, update_error: bool) -> float:
    """Update PID (C-style function)"""
    return pid.update(measured, update_error)


def pid_reset(pid: PIDObject, actual: float):
    """Reset PID (C-style function)"""
    pid.reset(actual)


def pid_set_integral_limit(pid: PIDObject, limit: float):
    """Set integral limit (C-style function)"""
    pid.set_integral_limit(limit)


def filter_reset(pid: PIDObject, sample_rate: int, cutoff_freq: float, enable: bool):
    """Reset filter parameters"""
    pid.filter_sample_rate = sample_rate
    pid.filter_cutoff = cutoff_freq
    pid.enable_d_filter = enable
    pid._init_filter()


def constrain(value: float, min_val: float, max_val: float) -> float:
    """Constrain value between min and max"""
    if value < min_val:
        return min_val
    elif value > max_val:
        return max_val
    return value
