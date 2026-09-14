#!/usr/bin/env python3
"""
Subsystem handling Gazebo Odometry logging to JSON (Columnar Array Format)
"""
import os
import json
import logging
import time
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from datetime import datetime
import inspect

logger = logging.getLogger(__name__)

class GZOdomNode(Node):

    def __init__(self, callback, name="gz_odom_node"):
        super().__init__(name)
        # Subscribe to Gazebo odometry topic
        self._odom_sub = self.create_subscription(
            Odometry, "/crazyflie/odom", callback, 10
        )


class GZOdom:
    """Handles subscribing to Crazyflie odometry and logging data to a JSON file."""

    def __init__(
        self,
        use_json_log: bool = True,
    ):
        self.use_json_log = use_json_log
        self.path = "/home/catt/crazyflie/crazyflie-ros/ros2_ws/src/sala4_crazyflie/cflib_python/"
        # Columnar structure with independent value arrays
        self.data_log = {
            "time_s": [],
            "position_x": [],
            "position_y": [],
            "position_z": [],
        }

        # Initialize ROS 2 if not already initialized
        if not rclpy.ok():
            rclpy.init()

        # Create ROS 2 subscriber node
        self._node = GZOdomNode(callback=self._incoming_odom)

    def _incoming_odom(self, msg: Odometry):
        """Callback triggered whenever new odometry data arrives from Gazebo."""
        pose = msg.pose.pose
        twist = msg.twist.twist
        sim_time_s = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)

        if self.use_json_log:
            # Append timestamp
            self.data_log["time_s"].append(sim_time_s)
            
            # Append position coordinates
            self.data_log["position_x"].append(float(pose.position.x))
            self.data_log["position_y"].append(float(pose.position.y))
            self.data_log["position_z"].append(float(pose.position.z))
            

    def save_log(self):

        stack = inspect.stack()
        date_path = datetime.now().strftime("%Y/%m/%d")
        folder_path = f"{self.path}{date_path}"

        if len(stack) > 1:
            filepath = stack[-1].filename
            subdir = os.path.join(folder_path, "Gazebologs")
            base_name = f"Gz_simlog"
        else:
            subdir = folder_path
            base_name = "Gz_simlog_test"

        # Find the next available filename: base_name1.json, base_name2.json, ...
        counter = 1
        while True:
            log_name = os.path.join(subdir, f"{base_name}{counter}.json")
            if not os.path.exists(log_name):
                break
            counter += 1
        # print(f"Saving Gazebo odometry log to: {log_name}")

        # Create the FULL directory tree for this specific file
        os.makedirs(os.path.dirname(log_name), exist_ok=True)

        with open(log_name, "w") as f:
            json.dump(self.data_log, f, indent=4)

    def destroy(self):
        """Clean shutdown helper."""
        self.save_log()
        self._node.destroy_node()

# --- ENTRY POINT FOR ROS 2 EXECUTION ---
def main(args=None):
    rclpy.init(args=args)
    gz_odom = GZOdom()
    try:
        # Keep the node running in the background
        rclpy.spin(gz_odom._node)
    except KeyboardInterrupt:
        pass
    finally:
        # Guarantees save_log() triggers when you stop the launch file (Ctrl+C)
        gz_odom.destroy()
        # rclpy.shutdown()

if __name__ == "__main__":
    main()