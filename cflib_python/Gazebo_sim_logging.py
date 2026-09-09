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
        stack = inspect.stack()
        folder_path = datetime.now().strftime("%Y/%m/%d")
        if len(stack) > 1:
            filepath = stack[-1].filename
            parentfile = os.path.splitext(os.path.basename(filepath))[0]
            log_filename = os.path.join(folder_path,f"{parentfile}_odomlog1.json")
        else:
            log_filename = os.path.join(folder_path, "odomlog_test1.json")
        os.makedirs(folder_path, exist_ok=True)
        
        if(log_filename)

        if self.use_json_log:
            with open(log_filename, "a") as f:
                json.dump(self.data_log, f, indent=4)
            logger.info(f"Saved odometry log to {log_filename}")

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
        rclpy.shutdown()

if __name__ == "__main__":
    main()