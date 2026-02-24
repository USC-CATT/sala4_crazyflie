#!/usr/bin/env python
"""
Subsytem handling raw motor data communication
"""

import collections
import logging
import struct

from cflib.crtp.crtpstack import CRTPPacket, CRTPPort
from cflib.utils.callbacks import Caller

logger = logging.getLogger(__name__)

# A generic location packet contains type and data. When received the data
# may be decoded by the lib.
MotorRawPacket = collections.namedtuple("motorRawPacket", ["m1", "m2", "m3", "m4"])
MOTOR_RAW_PORT = 0x09


class MotorRaw:
    """
    Handle localization-related data communication with the Crazyflie
    """

    # Implemented channels
    SETPOINT_CH = 0

    def __init__(self, crazyflie=None):
        """
        Initialize the Extpos object.
        """
        self._cf = crazyflie

        self.receivedLocationPacket = Caller()
        self._cf.add_port_callback(MOTOR_RAW_PORT, self._incoming)

    def _incoming(self, packet):
        """
        Callback for data received from the copter.
        """
        print(packet.data)
        if len(packet.data) < 1:
            logger.warning(
                "Localization packet received with incorrect"
                + "length (length is {})".format(len(packet.data))
            )
            return
        return

    def send_motor_raw(self, m1, m2, m3, m4):
        """
        Send the current Crazyflie X, Y, Z position. This is going to be
        forwarded to the Crazyflie's position estimator.
        """

        pk = CRTPPacket()
        pk.port = MOTOR_RAW_PORT
        pk.channel = self.SETPOINT_CH
        pk.data = struct.pack("<HHHH", m1, m2, m3, m4)
        self._cf.send_packet(pk)
