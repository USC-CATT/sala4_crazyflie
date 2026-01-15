#!/usr/bin/env python3

"""
Simple node to
"""

import numpy as np
import rclpy
import tf_transformations
from crazyflie_interfaces.srv import Arm
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist

WAYPOINTS = np.array([[0.5, 0.0, 1.0], [0.0, 0.5, 1.5]])  # In Meters


class WaypointFollowing(Node):
    def __init__(self):
        super().__init__("waypoint_following")
        self.declare_parameter("robot_prefix", "/crazyflie")
        robot_prefix = self.get_parameter("robot_prefix").value
        
        self.declare_parameter("delay", 5.0)
        self.delay = self.get_parameter("delay").value
        self.declare_parameter("max_turn_rate", 0.5)
        max_turn_rate = self.get_parameter("max_turn_rate").value
        self.declare_parameter("max_forward_speed", 0.5)
        max_forward_speed = self.get_parameter("max_forward_speed").value
        
        self.odom_subscriber = self.create_subscription(
            Odometry, robot_prefix + "/odom", self.odom_subscribe_callback, 10
        )
        
        self.srv = self.create_service(
            Trigger, robot_prefix + "/stop_waypoint_following", self.stop_waypoint_following_cb
        )
        
        self.position = [0.0, 0.0, 0.0]
        self.angles = [0.0, 0.0, 0.0]
        self.ranges = [0.0, 0.0, 0.0, 0.0]

        self.position_update = False

        self.twist_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        # Timer to wait and kick off waypoint following
        self.timer = self.create_timer(0.01, self.timer_callback)
        
        self.wait_for_start = True
        self.start_clock = self.get_clock().now().nanoseconds * 1e-9
        msg = Twist()
        msg.linear.z = 0.5
        self.twist_publisher.publish(msg)

    def timer_callback(self):
        # wait for the delay to pass and then start waypoint following
        if self.wait_for_start:
            if (
                self.get_clock().now().nanoseconds * 1e-9 - self.start_clock
                > self.delay
            ):
                self.get_logger().info("Starting waypoint following")
                self.wait_for_start = False
            else:
                return

        # initialize variables
        velocity_x = 0.0
        velocity_y = 0.0
        yaw_rate = 0.0

        msg = Twist()
        msg.linear.x = velocity_x
        msg.linear.y = velocity_y
        msg.angular.z = yaw_rate
        self.twist_publisher.publish(msg)
        
    def stop_waypoint_following_cb(self, request, response):
        self.get_logger().info("Stopping waypoint following")
        self.timer.cancel()
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = -0.2
        msg.angular.z = 0.0
        self.twist_publisher.publish(msg)

        response.success = True

        return response
        
    def odom_subscribe_callback(self, msg):
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        self.position[2] = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.angles[0] = euler[0]
        self.angles[1] = euler[1]
        self.angles[2] = euler[2]
        self.position_update = True
        

def main(args=None):
    rclpy.init(args=args)
    waypoint_following = WaypointFollowing()
    rclpy.spin(waypoint_following)
    waypoint_following.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
