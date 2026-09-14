#!/usr/bin/env python
"""
Subsystem handling raw motor data communication
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from actuator_msgs.msg import Actuators
import threading
import collections
import logging
import struct
import json
import time
import inspect
import os
from datetime import datetime  # <-- was missing entirely

from cflib.crtp.crtpstack import CRTPPacket, CRTPPort
from cflib.utils.callbacks import Caller

logger = logging.getLogger(__name__)

MotorRawPacket = collections.namedtuple("motorRawPacket", ["m1", "m2", "m3", "m4"])
MOTOR_RAW_PORT = 0x09


class MotorRawNode(Node):
    def __init__(self, name="motor_raw_node"):
        super().__init__(name)
        self._motor_raw_pub = self.create_publisher(Actuators, "motor_raw", 10)

    def publish_motor_raw(self, m1, m2, m3, m4):
        msg = Actuators()
        m1 = m1 / 65535.0 * 26180.0
        m2 = m2 / 65535.0 * 26180.0
        m3 = m3 / 65535.0 * 26180.0
        m4 = m4 / 65535.0 * 26180.0
        print(f"Publishing motor raw: {m1}, {m2}, {m3}, {m4}")
        msg.velocity = [float(m1), float(m2), float(m3), float(m4)]
        self._motor_raw_pub.publish(msg)


class MotorRaw:
    """
    Handle localization-related data communication with the Crazyflie
    """
    SETPOINT_CH = 0
    test = False

    def __init__(self, crazyflie=None, test=False, using_json_log=False):
        self.start_time = time.time()
        self.using_json_log = using_json_log
        self.test = test
        self._closed = False
        self.data_json = {"velocities": [], "start_time": self.start_time}  # instance-level, not shared

        if not test:
            self._cf = crazyflie
            self.receivedLocationPacket = Caller()
            self._cf.add_port_callback(MOTOR_RAW_PORT, self._incoming)

        if not rclpy.ok():
            rclpy.init()
        self._node = MotorRawNode()

    def save_log(self):
        if self.using_json_log:
            return  # keeping your original logic: skip file write if using_json_log

        stack = inspect.stack()
        folder_path = datetime.now().strftime("%Y/%m/%d")

        if len(stack) > 1:
            filepath = stack[-1].filename
            parentfile = os.path.splitext(os.path.basename(filepath))[0]
            if not parentfile:
                parentfile = "unknown_script"
            subdir = os.path.join(folder_path, "motorlogs")
            base_name = f"{parentfile}_motorlog"
        else:
            subdir = folder_path
            base_name = "motorlog_test"

        # Find the next available filename: base_name1.json, base_name2.json, ...
        counter = 1
        while True:
            log_name = os.path.join(subdir, f"{base_name}{counter}.json")
            if not os.path.exists(log_name):
                break
            counter += 1

        # Create the FULL directory tree for this specific file
        os.makedirs(os.path.dirname(log_name), exist_ok=True)

        with open(log_name, "w") as f:
            json.dump(self.data_json, f, indent=4)

    def close(self):
        """Call this explicitly when shutting down (e.g. node destroy / finally block)."""
        if self._closed:
            return
        self._closed = True
        try:
            self.save_log()
        except Exception:
            logger.exception("Failed to save motor log on close()")

    def __del__(self):
        # Safety net only — don't rely on this for correctness.
        try:
            self.close()
        except Exception:
            pass

    def _incoming(self, packet):
        print(packet.data)
        if len(packet.data) < 1:
            logger.warning(
                f"Packet received with incorrect length (length is {len(packet.data)})"
            )
            return
        return

    def send_motor_raw(self, m1, m2, m3, m4):
        self._node.publish_motor_raw(m1, m2, m3, m4)
        self.data_json["velocities"].append({
            "m1": m1, "m2": m2, "m3": m3, "m4": m4,
            "timestamp": time.time() - self.start_time
        })
        if not self.test:
            pk = CRTPPacket()
            pk.port = MOTOR_RAW_PORT
            pk.channel = self.SETPOINT_CH
            pk.data = struct.pack("<HHHH", m1, m2, m3, m4)
            self._cf.send_packet(pk)