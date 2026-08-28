#!/usr/bin/env python3
"""
Subsystem handling Gazebo Odometry logging to JSON (Columnar Array Format)
"""

import json
import logging
import time
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

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
        log_filename: str = "odom_raw_log.json",
    ):
        self.use_json_log = use_json_log
        self.log_filename = log_filename
        
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
        """Explicitly saves logged odometry data to the specified JSON file."""
        if self.use_json_log:
            with open(self.log_filename, "w") as f:
                json.dump(self.data_log, f, indent=4)
            logger.info(f"Saved odometry log to {self.log_filename}")

    def destroy(self):
        """Clean shutdown helper."""
        self.save_log()
        self._node.destroy_node()