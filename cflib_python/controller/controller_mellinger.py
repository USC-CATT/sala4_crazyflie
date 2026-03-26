"""
Main Mellinger controller implementation for Crazyflie

Based on mellinger.c from Crazyflie firmware
"""

from typing import Optional
import math

from .mellinger_constants import *

from .controller_types import (
    ATTITUDE_RATE,
    Control,
    ControlMode,
    Quaternion,
    SensorData,
    Setpoint,
    StabMode,
    Vector,
    State,
    degrees,
    quat2rpy,
    radians,
    rate_do_execute,
    vsub,
    vnormalize,
    clamp,
    vcross,
    vdot,
    mcolumn,
    mcolumns,
    mvmul,
    quat2rotmat
)


class ControllerMellinger:
    """Main Mellinger controller for Crazyflie"""

    def __init__(self):
        """Initialize Mellinger controller"""

        # Logging variables
        self.cmd_thrust = 0.0
        self.cmd_roll = 0.0
        self.cmd_pitch = 0.0
        self.cmd_yaw = 0.0
        self.r_roll = 0.0
        self.r_pitch = 0.0
        self.r_yaw = 0.0
        self.accelz = 0.0
        self.prev_omega_roll = float('nan')
        self.prev_omega_pitch = float('nan')
        self.prev_setpoint_omega_roll = float('nan')
        self.prev_setpoint_omega_pitch = float('nan')


        self.i_error_x = 0
        self.i_error_y = 0
        self.i_error_z = 0
        self.i_error_m_x = 0
        self.i_error_m_y = 0
        self.i_error_m_z = 0

    def reset(self):
        self.cmd_thrust = 0.0
        self.cmd_roll = 0.0
        self.cmd_pitch = 0.0
        self.cmd_yaw = 0.0
        self.r_roll = 0.0
        self.r_pitch = 0.0
        self.r_yaw = 0.0
        self.accelz = 0.0

        self.i_error_x = 0
        self.i_error_y = 0
        self.i_error_z = 0
        self.i_error_m_x = 0
        self.i_error_m_y = 0
        self.i_error_m_z = 0
        self.prev_omega_roll = float('nan')
        self.prev_omega_pitch = float('nan')
        self.prev_setpoint_omega_roll = float('nan')
        self.prev_setpoint_omega_pitch = float('nan')

        

    def test(self):
        """
        Test if controller is properly initialized

        Returns:
            True if initialized
        """
        return True

    def controller_mellinger(
        self,
        control: Control,
        setpoint: Setpoint,
        sensors: SensorData,
        state: State,
        stabilizer_step: int,
    ):
        """
        Main Mellinger controller update

        Args:
            control: Control output (modified in place)
            setpoint: Desired setpoint
            sensors: Sensor data
            state: Current state estimate
            stabilizer_step: Current stabilizer step for rate limiting
        """
        r_error = Vector()
        v_error  = Vector()
        target_thrust  = Vector()
        z_axis  = Vector()
        current_thrust = 0.0
        x_axis_desired = Vector()
        y_axis_desired = Vector()
        x_c_des = Vector()
        eR = Vector()
        ew = Vector()
        M = Vector()
        dt= 0.0
        desiredYaw = 0.0

        control.control_mode = ControlMode.CONTROL_MODE_LEGACY

        if not rate_do_execute(ATTITUDE_RATE, stabilizer_step):
          return


        dt = float(1.0/ATTITUDE_RATE)
        setpointPos = Vector(setpoint.position.x, setpoint.position.y, setpoint.position.z)
        setpointVel = Vector(setpoint.velocity.x, setpoint.velocity.y, setpoint.velocity.z)
        statePos = Vector(state.position.x, state.position.y, state.position.z)
        stateVel = Vector(state.velocity.x, state.velocity.y, state.velocity.z)

        # Position Error (ep)
        r_error = vsub(setpointPos, statePos)

        # Velocity Error (ev)
        v_error = vsub(setpointVel, stateVel)

        # Integral Error
        self.i_error_z += r_error.z * dt
        self.i_error_z = clamp(self.i_error_z, -I_RANGE_Z, I_RANGE_Z)

        self.i_error_x += r_error.x * dt
        self.i_error_x = clamp(self.i_error_x, -I_RANGE_XY, I_RANGE_XY)

        self.i_error_y += r_error.y * dt
        self.i_error_y = clamp(self.i_error_y, -I_RANGE_XY, I_RANGE_XY)

        # Desired thrust [F_des]
        if (setpoint.mode.x == StabMode.MODE_ABS) :
          target_thrust.x = MASS * setpoint.acceleration.x                       + KP_XY * r_error.x + KD_XY * v_error.x + KI_XY * self.i_error_x
          target_thrust.y = MASS * setpoint.acceleration.y                       + KP_XY * r_error.y + KD_XY * v_error.y + KI_XY * self.i_error_y
          target_thrust.z = MASS * (setpoint.acceleration.z + GRAVITY_MAGNITUDE) + KP_Z  * r_error.z + KD_Z  * v_error.z + KI_Z  * self.i_error_z
        else:
          target_thrust.x = -math.sin(radians(setpoint.attitude.pitch))
          target_thrust.y = -math.sin(radians(setpoint.attitude.roll))
          # In case of a timeout, the commander tries to level, ie. x/y are disabled, but z will use the previous setting
          # In that case we ignore the last feedforward term for acceleration
          if (setpoint.mode.z == StabMode.MODE_ABS):
            target_thrust.z = MASS * GRAVITY_MAGNITUDE + KP_Z  * r_error.z + KD_Z  * v_error.z + KI_Z  * self.i_error_z
          else:
            target_thrust.z = 1



        # Rate-controlled YAW is moving YAW angle setpoint
        if (setpoint.mode.yaw == StabMode.MODE_VELOCITY):
          desiredYaw = state.attitude.yaw + setpoint.attitude_rate.yaw * dt
        elif (setpoint.mode.yaw == StabMode.MODE_ABS):
          desiredYaw = setpoint.attitude.yaw
        elif (setpoint.mode.quat == StabMode.MODE_ABS):
          setpoint_quat = Quaternion(x=setpoint.attitude_quaternion.x, y=setpoint.attitude_quaternion.y, z=setpoint.attitude_quaternion.z, w=setpoint.attitude_quaternion.w)
          rpy = quat2rpy(setpoint_quat)
          desiredYaw = degrees(rpy.z)


        # Z-Axis [zB]
        q = Quaternion(x=state.attitude_quaternion.x, y=state.attitude_quaternion.y, z=state.attitude_quaternion.z, w=state.attitude_quaternion.w)
        R = quat2rotmat(q)
        z_axis = mcolumn(R, 2)

        # yaw correction (only if position control is not used)
        if (setpoint.mode.x != StabMode.MODE_ABS):
          x_yaw = mcolumn(R, 0)
          x_yaw.z = 0
          x_yaw = vnormalize(x_yaw)
          y_yaw = vcross(Vector(0, 0, 1), x_yaw)
          R_yaw_only = mcolumns(x_yaw, y_yaw, Vector(0, 0, 1))
          target_thrust = mvmul(R_yaw_only, target_thrust)


        # Current thrust [F]
        current_thrust = vdot(target_thrust, z_axis)

        # Calculate axis [zB_des]
        z_axis_desired = vnormalize(target_thrust)

        # [xC_des]
        # x_axis_desired = z_axis_desired x [sin(yaw), cos(yaw), 0]^T
        x_c_des.x = math.cos(radians(desiredYaw))
        x_c_des.y = math.sin(radians(desiredYaw))
        x_c_des.z = 0
        # [yB_des]
        y_axis_desired = vnormalize(vcross(z_axis_desired, x_c_des))
        # [xB_des]
        x_axis_desired = vcross(y_axis_desired, z_axis_desired)

        x = q.x
        y = q.y
        z = q.z
        w = q.w
        eR.x = (-1 + 2*pow(x, 2) + 2*pow(y, 2))*y_axis_desired.z + z_axis_desired.y - 2*(x*y_axis_desired.x*z + y*y_axis_desired.y*z - x*y*z_axis_desired.x + pow(x, 2)*z_axis_desired.y + pow(z, 2)*z_axis_desired.y - y*z*z_axis_desired.z) +    2*w*(-(y*y_axis_desired.x) - z*z_axis_desired.x + x*(y_axis_desired.y + z_axis_desired.z))
        eR.y = x_axis_desired.z - z_axis_desired.x - 2*(pow(x, 2)*x_axis_desired.z + y*(x_axis_desired.z*y - x_axis_desired.y*z) - (pow(y, 2) + pow(z, 2))*z_axis_desired.x + x*(-(x_axis_desired.x*z) + y*z_axis_desired.y + z*z_axis_desired.z) + w*(x*x_axis_desired.y + z*z_axis_desired.y - y*(x_axis_desired.x + z_axis_desired.z)))
        eR.z = y_axis_desired.x - 2*(y*(x*x_axis_desired.x + y*y_axis_desired.x - x*y_axis_desired.y) + w*(x*x_axis_desired.z + y*y_axis_desired.z)) + 2*(-(x_axis_desired.z*y) + w*(x_axis_desired.x + y_axis_desired.y) + x*y_axis_desired.z)*z - 2*y_axis_desired.x*pow(z, 2) + x_axis_desired.y*(-1 + 2*pow(x, 2) + 2*pow(z, 2))

        # Account for Crazyflie coordinate system
        eR.y = -eR.y

        # [ew]
        err_d_roll = 0.0
        err_d_pitch = 0.0

        stateAttitudeRateRoll = radians(sensors.gyro.x)
        stateAttitudeRatePitch = -radians(sensors.gyro.y)
        stateAttitudeRateYaw = radians(sensors.gyro.z)

        ew.x = radians(setpoint.attitude_rate.roll) - stateAttitudeRateRoll
        ew.y = -radians(setpoint.attitude_rate.pitch) - stateAttitudeRatePitch
        ew.z = radians(setpoint.attitude_rate.yaw) - stateAttitudeRateYaw
        if (self.prev_omega_roll == self.prev_omega_roll): #d part initialized
          err_d_roll = ((radians(setpoint.attitude_rate.roll) - self.prev_setpoint_omega_roll) - (stateAttitudeRateRoll - self.prev_omega_roll)) / dt
          err_d_pitch = (-(radians(setpoint.attitude_rate.pitch) - self.prev_setpoint_omega_pitch) - (stateAttitudeRatePitch - self.prev_omega_pitch)) / dt

        self.prev_omega_roll = stateAttitudeRateRoll
        self.prev_omega_pitch = stateAttitudeRatePitch
        self.prev_setpoint_omega_roll = radians(setpoint.attitude_rate.roll)
        self.prev_setpoint_omega_pitch = radians(setpoint.attitude_rate.pitch)

        # Integral Error
        self.i_error_m_x += (-eR.x) * dt
        self.i_error_m_x = clamp(self.i_error_m_x, -I_RANGE_M_XY, I_RANGE_M_XY)

        self.i_error_m_y += (-eR.y) * dt
        self.i_error_m_y = clamp(self.i_error_m_y, -I_RANGE_M_XY, I_RANGE_M_XY)

        self.i_error_m_z += (-eR.z) * dt
        self.i_error_m_z = clamp(self.i_error_m_z, -I_RANGE_M_Z, I_RANGE_M_Z)

        # Moment:
        M.x = -KR_XY * eR.x + KW_XY * ew.x + KI_M_XY * self.i_error_m_x + KD_OMEGA_RP * err_d_roll
        M.y = -KR_XY * eR.y + KW_XY * ew.y + KI_M_XY * self.i_error_m_y + KD_OMEGA_RP * err_d_pitch
        M.z = -KR_Z  * eR.z + KW_Z  * ew.z + KI_M_Z  * self.i_error_m_z

        # Output
        if (setpoint.mode.z == StabMode.MODE_DISABLE):
          control.thrust = setpoint.thrust
        else:
          control.thrust = MASS_THRUST * current_thrust


        self.cmd_thrust = control.thrust
        self.r_roll = radians(sensors.gyro.x)
        self.r_pitch = -radians(sensors.gyro.y)
        self.r_yaw = radians(sensors.gyro.z)
        self.accelz = sensors.acc.z

        if (control.thrust > 0):
          control.roll = clamp(M.x, -32000, 32000)
          control.pitch = clamp(M.y, -32000, 32000)
          control.yaw = clamp(-M.z, -32000, 32000)

          self.cmd_roll = control.roll
          self.cmd_pitch = control.pitch
          self.cmd_yaw = control.yaw

        else:
          control.roll = 0
          control.pitch = 0
          control.yaw = 0

          self.cmd_roll = control.roll
          self.cmd_pitch = control.pitch
          self.cmd_yaw = control.yaw

          self.reset()
        
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
        }


# C-style function interfaces for compatibility
_controller_mellinger_instance: Optional[ControllerMellinger] = None


def controller_mellinger_init():
    """Initialize controller"""
    global _controller_mellinger_instance
    _controller_mellinger_instance = ControllerMellinger()


def controller_mellinger_test():
    """Test if controller is initialized"""
    global _controller_mellinger_instance
    if _controller_mellinger_instance is None:
        return False
    return _controller_mellinger_instance.test()


def controller_mellinger(
    control: Control,
    setpoint: Setpoint,
    sensors: SensorData,
    state: State,
    stabilizer_step: int,
):
    """Run controller (C-style function)"""
    global _controller_mellinger_instance
    if _controller_mellinger_instance is not None:
        _controller_mellinger_instance.controller_mellinger(
            control, setpoint, sensors, state, stabilizer_step
        )
