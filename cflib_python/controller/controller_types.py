"""
Data types and structures for Crazyflie PID controller

Based on stabilizer_types.h from Crazyflie firmware
Copyright (C) 2011-2012 Bitcraze AB
"""

from dataclasses import dataclass, field
from enum import IntEnum
from typing import List

from numpy import sqrt


class StabMode(IntEnum):
    """Stabilization mode"""

    MODE_DISABLE = 0
    MODE_ABS = 1
    MODE_VELOCITY = 2


class ControlMode(IntEnum):
    """Control mode"""

    CONTROL_MODE_LEGACY = 0
    CONTROL_MODE_FORCE = 1


@dataclass
class Axis3f:
    """3D axis float values"""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class Quaternion:
    """Quaternion representation"""

    w: float = 1.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class Attitude:
    """Attitude in Euler angles (degrees)"""

    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


@dataclass
class AttitudeRate:
    """Attitude rates (deg/s)"""

    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


@dataclass
class GyroData:
    """Gyroscope data (deg/s)"""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class AccData:
    """Accelerometer data (g)"""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class SensorData:
    """Sensor data from IMU"""

    gyro: GyroData = field(default_factory=GyroData)
    acc: AccData = field(default_factory=AccData)


@dataclass
class Position:
    """Position in 3D space (meters)"""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class Velocity:
    """Velocity in 3D space (m/s)"""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

@dataclass
class Acceleration:
    """Acceleration in 3D space (m/s^2)"""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class State:
    """Complete state of the Crazyflie"""

    attitude: Attitude = field(default_factory=Attitude)
    attitude_quaternion: Quaternion = field(default_factory=Quaternion)
    position: Position = field(default_factory=Position)
    velocity: Velocity = field(default_factory=Velocity)
    acc: Axis3f = field(default_factory=Axis3f)


@dataclass
class SetpointMode:
    """Mode for each control axis"""

    x: StabMode = StabMode.MODE_DISABLE
    y: StabMode = StabMode.MODE_DISABLE
    z: StabMode = StabMode.MODE_DISABLE
    roll: StabMode = StabMode.MODE_DISABLE
    pitch: StabMode = StabMode.MODE_DISABLE
    yaw: StabMode = StabMode.MODE_DISABLE
    quat: StabMode = StabMode.MODE_DISABLE


@dataclass
class Setpoint:
    """Setpoint for the controller"""

    position: Position = field(default_factory=Position)
    velocity: Velocity = field(default_factory=Velocity)
    attitude: Attitude = field(default_factory=Attitude)
    acceleration: Acceleration = field(default_factory=Acceleration)
    attitude_rate: AttitudeRate = field(default_factory=AttitudeRate)
    attitude_quaternion: Quaternion = field(default_factory=Quaternion)
    thrust: float = 0.0
    mode: SetpointMode = field(default_factory=SetpointMode)
    velocity_body: bool = False  # True if velocity is in body frame


@dataclass
class Control:
    """Control output"""

    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    thrust: float = 0.0
    control_mode: ControlMode = ControlMode.CONTROL_MODE_LEGACY

@dataclass
class Vector:
    """Vector"""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

@dataclass
class Mat33:
    m: List[List[float]]
    def __post_init__(self):
        """Validate that the input is exactly 3x3."""
        if len(self.m) != 3 or any(len(row) != 3 for row in self.m):
            raise ValueError("Matrix must be exactly 3x3")


class StabilizerStep(IntEnum):
    """Stabilizer step counter for rate limiting"""

    STEP_0 = 0
    STEP_1 = 1
    STEP_2 = 2
    STEP_3 = 3
    STEP_4 = 4


# Rate constants
ATTITUDE_RATE = 500  # Hz
POSITION_RATE = 100  # Hz


def rate_do_execute(rate: int, step: int) -> bool:
    """
    Check if a rate should execute on this step

    Args:
        rate: Target rate in Hz
        step: Current stabilizer step

    Returns:
        True if should execute
    """
    if rate == ATTITUDE_RATE:
        return True
    elif rate == POSITION_RATE:
        # Execute every 5th step (500Hz / 5 = 100Hz)
        return (step % 5) == 0
    return False


def degrees(radians: float) -> float:
    """Convert radians to degrees"""
    import math

    return radians * 180.0 / math.pi


def radians(degrees: float) -> float:
    """Convert degrees to radians"""
    import math

    return degrees * math.pi / 180.0


def quat2rpy(q: Quaternion) -> Axis3f:
    """
    Convert quaternion to roll-pitch-yaw (radians)

    Args:
        q: Quaternion

    Returns:
        Roll, pitch, yaw as Axis3f in radians
    """
    import math

    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return Axis3f(x=roll, y=pitch, z=yaw)


def cap_angle(angle: float) -> float:
    """
    Wrap angle to [-180, 180] range

    Args:
        angle: Angle in degrees

    Returns:
        Wrapped angle
    """
    result = angle
    while result > 180.0:
        result -= 360.0
    while result < -180.0:
        result += 360.0
    return result

def vscl(s: float, v: Vector):
	return Vector(s * v.x , s * v.y, s * v.z)

# negate a vector.
def vneg(v: Vector):
	return Vector(-v.x, -v.y, -v.z)

# divide a vector by a scalar.
# does not perform divide-by-zero check.
def vdiv(v: Vector, s: float):
	return vscl(1.0/s, v)

# add two vectors.
def vadd(a: Vector,  b: Vector):
	return Vector(a.x + b.x, a.y + b.y, a.z + b.z)

# subtract a vector from another vector.
def vsub(a: Vector, b: Vector):
	return vadd(a, vneg(b))

# vector dot product.
def vdot(a: Vector, b: Vector):
	return a.x * b.x + a.y * b.y + a.z * b.z

def vmag2(v: Vector):
	return vdot(v, v)


def vmag(v: Vector) :
	return sqrt(vmag2(v))

def vnormalize(v: Vector):
	return vdiv(v, vmag(v))

def clamp(t, min_val, max_val):
    return min(max(t, min_val), max_val)


def vcross( a: Vector, b: Vector):
	return Vector(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x)

def mcolumn(m: Mat33, col: int):
	return Vector(m.m[0][col], m.m[1][col], m.m[2][col])
	
def mcolumns(a: Vector,  b: Vector, c: Vector):
	m = Mat33([[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]])
	m.m[0][0] = a.x
	m.m[1][0] = a.y
	m.m[2][0] = a.z
	m.m[0][1] = b.x
	m.m[1][1] = b.y
	m.m[2][1] = b.z
	m.m[0][2] = c.x
	m.m[1][2] = c.y
	m.m[2][2] = c.z
	return m

def mvmul(a: Mat33, v: Vector):
	x = a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z
	y = a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z
	z = a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z
	return Vector(x, y, z)
	
def quat2rotmat(q: Quaternion):
	x = q.x
	y = q.y
	z = q.z
	w = q.w
	m = Mat33([[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]])
	m.m[0][0] = 1 - 2*y*y - 2*z*z
	m.m[0][1] = 2*x*y - 2*z*w
	m.m[0][2] = 2*x*z + 2*y*w
	m.m[1][0] = 2*x*y + 2*z*w
	m.m[1][1] = 1 - 2*x*x - 2*z*z
	m.m[1][2] = 2*y*z - 2*x*w
	m.m[2][0] = 2*x*z - 2*y*w
	m.m[2][1] = 2*y*z + 2*x*w
	m.m[2][2] = 1 - 2*x*x - 2*y*y
	return m
