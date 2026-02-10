"""
Simple example that connects to the first Crazyflie found, ramps up/down
the motors and disconnects.
"""

import logging
import time

import cflib
from cflib.crazyflie import Crazyflie
from cflib.utils import uri_helper

uri = uri_helper.uri_from_env(default="radio://0/80/2M/E7E7E7E7E7")

logging.basicConfig(level=logging.ERROR)


class MotorRampExample:
    """Example that connects to a Crazyflie and ramps the motors up/down and
    the disconnects"""

    def __init__(self, link_uri):
        """Initialize and run the example with the specified link_uri"""

        self._cf = Crazyflie(rw_cache="./cache")

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        self._cf.open_link(link_uri)

        print("Connecting to %s" % link_uri)

    def _connected(self, link_uri):
        """This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""

        # Arm the Crazyflie
        self._cf.platform.send_arming_request(True)
        time.sleep(1.0)

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print("Connection to %s failed: %s" % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print("Connection to %s lost: %s" % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print("Disconnected from %s" % link_uri)

    def ramp_motors(self):
        pitch = 0
        roll = 5
        yawrate = 0

        # Unlock startup thrust protection
        self._cf.platform.send_arming_request(True)
        time.sleep(0.75)

        start_time = time.time()
        duration = 1.25

        while time.time() < start_time + duration:
            self._cf.commander.send_zdistance_setpoint(roll, pitch, yawrate, 0.5)
            time.sleep(0.1)
        for _ in range(30):
            # Continuously send the zero setpoint until the drone is recognized as landed
            # to prevent the supervisor from intervening due to missing regular setpoints
            self._cf.commander.send_position_setpoint(0.0, 0.0, 0.1, 0.0)
            # Sleeping before closing the link makes sure the last
            # packet leaves before the link is closed, since the
            # message queue is not flushed before closing
            time.sleep(0.1)
        for _ in range(5):
            # Continuously send the zero setpoint until the drone is recognized as landed
            # to prevent the supervisor from intervening due to missing regular setpoints
            self._cf.commander.send_position_setpoint(0.0, 0.0, 0.0, 0.0)
            # Sleeping before closing the link makes sure the last
            # packet leaves before the link is closed, since the
            # message queue is not flushed before closing
            time.sleep(0.1)
        self._cf.commander.send_notify_setpoint_stop()

    def disconnect(self):
        self._cf.close_link()


if __name__ == "__main__":
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    me = MotorRampExample(uri)

    me.ramp_motors()

    me.disconnect()
