#!/usr/bin/env python3

"""
Simple node to arm the crazyflies once all of them are ready
"""

import rclpy
from crazyflie_interfaces.srv import Arm
from rclpy.node import Node

class Arming(Node):
    def __init__(self):
        super().__init__("arming")
        self.declare_parameter("robot_prefix", "/crazyflie")
        robot_prefix = self.get_parameter("robot_prefix").value

        self.armService = self.create_client(Arm, "all/arm")

        self.get_logger().info("Waiting for /all/arm service...")
        while not self.armService.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("/all/arm service not available, waiting again...")

        self.get_logger().info("/all/arm service is ready!")

        # Give a take off command but wait for the delay to start the wall following
        self.wait_for_start = True
        self.start_clock = self.get_clock().now().nanoseconds * 1e-9

        self.create_timer(2.0, self.arm_callback)  # Arms after 2 seconds
        self.armed = False

    def arm_callback(self):
        """Arm after timer"""
        if not self.armed:
            self.get_logger().info("Arming crazyflies")
            req = Arm.Request()
            req.arm = True

            future = self.armService.call_async(req)
            future.add_done_callback(self.arm_response_callback)

    def arm_response_callback(self, future):
        try:
            self.get_logger().info("Crazyflies armed successfully!")
            self.armed = True
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    arming = Arming()
    rclpy.spin(arming)
    arming.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
