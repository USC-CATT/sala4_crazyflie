#!/usr/bin/env python3
"""
ROS 2 replacement for the Crazyradio link (for use with Gazebo, no hardware).

It does the same two jobs the cflib link does in augmented_pid_pwm_position.py:

  1. STATE IN:  subscribes to a nav_msgs/Odometry topic (optionally a
     sensor_msgs/Imu topic for gyro) and calls `on_state(dict)` with the SAME
     keys the cflib LogConfig produces ("stateEstimate.x", "gyro.x", ...),
     in the same units (m, m/s, deg, deg/s). The existing
     `_log_state_callback` can therefore be reused unchanged.

  2. MOTORS OUT: `send_motor_raw(m1, m2, m3, m4)` takes the same 0..65535 PWM
     values as MotorRaw and publishes motor speeds (rad/s) to Gazebo.

Conventions copied from the Crazyflie firmware (check these in sim!):
  * stateEstimate.pitch has the OPPOSITE sign of the standard ROS/aerospace
    pitch, so pitch is negated here.
  * velocities are in the WORLD frame. Gazebo/ROS odometry twists are usually
    in the BODY (child) frame, so they are rotated by the orientation unless
    `twist_in_body=False`.
"""

import math
import threading
from typing import Callable, Dict, Optional, Sequence

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Imu

UINT16_MAX = 65535.0
THRUST_MAX = 0.2
SUPPLY_V = 4.2
# Same voltage -> thrust polynomial the script uses for battery compensation.
V0 = -0.014058926705279723
V1 = 0.04265273261724981
V2 = 0.0018327760144017432
V3 = 0.0020576974784587178

PITCH_SIGN = -1.0  # Crazyflie stateEstimate.pitch is inverted


def _quat_to_rpy_deg(x, y, z, w):
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def _rotate(q, v):
    """Rotate vector v by quaternion q=(x,y,z,w): body -> world."""
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def _rotate_inv(q, v):
    """world -> body."""
    return _rotate((-q[0], -q[1], -q[2], q[3]), v)


class RosLink:
    def __init__(
        self,
        on_state: Callable[[Dict[str, float]], None],
        odom_topic: str,
        motor_topic: str,
        omega_hover: float,
        hover_cmd: float = 36000.0,
        motor_msg_type: str = "actuator_msgs/msg/Actuators",
        imu_topic: Optional[str] = None,
        twist_in_body: bool = True,
        motor_map: Sequence[int] = (0, 1, 2, 3),
        vbat: float = 4.2,
        node_name: str = "host_pid_link",
    ):
        """
        omega_hover: rotor speed (rad/s) at which the 4 rotors exactly hold the
                     model's weight in the SDF, sqrt(m*g / (4*kf)).
        hover_cmd:   the controller's hover thrust command (position
                     controller's thrust_base, 36000 by default on Crazyflie).
        motor_map:   motor_map[i] = index of the Gazebo rotor driven by
                     Crazyflie motor i+1. Reorder if the SDF numbers rotors
                     differently.
        """
        self.on_state = on_state
        self.twist_in_body = twist_in_body
        self.omega_hover = float(omega_hover)
        self.hover_cmd = float(hover_cmd)
        self.motor_map = list(motor_map)
        self.vbat = vbat
        self._imu_gyro = None  # rad/s, body frame
        self.motor_rpm = [0.0, 0.0, 0.0, 0.0]

        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            rclpy.init()
        self.node = Node(node_name)
        self.node.create_subscription(
            Odometry, odom_topic, self._on_odom, qos_profile_sensor_data
        )
        if imu_topic:
            self.node.create_subscription(
                Imu, imu_topic, self._on_imu, qos_profile_sensor_data
            )

        self._motor_cls = get_message(motor_msg_type)
        self._motor_pub = self.node.create_publisher(self._motor_cls, motor_topic, 10)

        self._thread = threading.Thread(
            target=rclpy.spin, args=(self.node,), daemon=True
        )

    def start(self):
        self._thread.start()

    def shutdown(self):
        try:
            self.node.destroy_node()
            # only tear down the ROS context if we created it (MotorRaw or
            # another node in the same process may be using it)
            if self._owns_rclpy and rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    # ---- state in -------------------------------------------------------
    def _on_imu(self, msg: Imu):
        a = msg.angular_velocity
        self._imu_gyro = (a.x, a.y, a.z)

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        q = (o.x, o.y, o.z, o.w)
        lv = msg.twist.twist.linear
        av = msg.twist.twist.angular

        if self.twist_in_body:
            v_world = _rotate(q, (lv.x, lv.y, lv.z))
            w_body = (av.x, av.y, av.z)
        else:
            v_world = (lv.x, lv.y, lv.z)
            w_body = _rotate_inv(q, (av.x, av.y, av.z))
        if self._imu_gyro is not None:
            w_body = self._imu_gyro

        roll, pitch, yaw = _quat_to_rpy_deg(*q)
        self.on_state(
            {
                "stateEstimate.x": p.x,
                "stateEstimate.y": p.y,
                "stateEstimate.z": p.z,
                "stateEstimate.vx": v_world[0],
                "stateEstimate.vy": v_world[1],
                "stateEstimate.vz": v_world[2],
                "stateEstimate.roll": roll,
                "stateEstimate.pitch": PITCH_SIGN * pitch,
                "stateEstimate.yaw": yaw,
                "gyro.x": math.degrees(w_body[0]),
                "gyro.y": math.degrees(w_body[1]),
                "gyro.z": math.degrees(w_body[2]),
                "pm.vbat": self.vbat,
            }
        )

    # ---- motors out -----------------------------------------------------
    def _pwm_to_omega(self, pwm: float) -> float:
        if pwm <= 0.0:
            return 0.0
        v = min(pwm, UINT16_MAX) / UINT16_MAX * SUPPLY_V
        thrust = V0 + v * (V1 + v * (V2 + v * V3))
        if thrust <= 0.0:
            return 0.0
        # The script's battery compensator targets thrust = cmd/65535*THRUST_MAX,
        # so this recovers the original (pre-compensation) thrust command.
        cmd_equiv = thrust / THRUST_MAX * UINT16_MAX
        # Rotor thrust ~ omega^2, anchored so hover_cmd -> omega_hover.
        return self.omega_hover * math.sqrt(cmd_equiv / self.hover_cmd)

    def send_motor_raw(self, m1, m2, m3, m4):
        omegas = [self._pwm_to_omega(float(m)) for m in (m1, m2, m3, m4)]
        self.motor_rpm = [omega * 60.0 / (2.0 * math.pi) for omega in omegas]
        out = [0.0] * 4
        for i, rotor_idx in enumerate(self.motor_map):
            out[rotor_idx] = omegas[i]

        msg = self._motor_cls()
        if hasattr(msg, "header"):
            msg.header.stamp = self.node.get_clock().now().to_msg()
        if hasattr(msg, "velocity"):  # actuator_msgs/Actuators
            msg.velocity = out
        elif hasattr(msg, "angular_velocities"):  # mav_msgs/Actuators
            msg.angular_velocities = out
        else:
            raise RuntimeError(
                "Motor message has neither 'velocity' nor 'angular_velocities'"
            )
        self._motor_pub.publish(msg)