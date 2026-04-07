#!/usr/bin/env python
"""
Subsytem handling raw motor data communication
"""
#ROS2 Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import threading
import collections
import logging
import struct

from cflib.crtp.crtpstack import CRTPPacket, CRTPPort
from cflib.utils.callbacks import Caller

logger = logging.getLogger(__name__)


MotorRawPacket = collections.namedtuple("motorRawPacket", ["m1", "m2", "m3", "m4"])
MOTOR_RAW_PORT = 0x09


class MotorRawNode(Node):
    def __init__(self, name="motor_raw_node"):
        super().__init__(name)
        self._motor_raw_pub = self.create_publisher(Int32MultiArray, "motor_raw", 10)
    def publish_motor_raw(self, m1, m2, m3, m4):
        msg = Int32MultiArray()
        msg.data = [m1, m2, m3, m4]
        self._motor_raw_pub.publish(msg)

class MotorRaw:
    """
    Handle localization-related data communication with the Crazyflie
    """
    # Implemented channels
    SETPOINT_CH = 0
    def __init__(self, crazyflie=None):
        self._cf = crazyflie

        self.receivedLocationPacket = Caller()
        self._cf.add_port_callback(MOTOR_RAW_PORT, self._incoming)

        #ROS2 Initialization
        if not rclpy.ok():
            rclpy.init()
        self._node = MotorRawNode()
        # Spin in background so it doesn't block
        # self._ros_thread = threading.Thread(target=rclpy.spin, args=(self._node,), daemon=True)
        # self._ros_thread.start()

    def _incoming(self, packet):
        """
        Callback for data received from the copter.
        """
        print(packet.data)
        if len(packet.data) < 1:
            logger.warning(
                f"Packet received with incorrect length (length is {len(packet.data)})"
            )
            return
        return

    def send_motor_raw(self, m1, m2, m3, m4):
        self._node.publish_motor_raw(m1, m2, m3, m4)
        pk = CRTPPacket()
        pk.port = MOTOR_RAW_PORT
        pk.channel = self.SETPOINT_CH
        pk.data = struct.pack("<HHHH", m1, m2, m3, m4)
        self._cf.send_packet(pk)
