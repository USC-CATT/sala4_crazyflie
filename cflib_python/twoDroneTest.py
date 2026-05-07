import json
import struct
import sys
import threading
import time
from typing import cast

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crtp.crtpstack import CRTPPacket
from cflib.crtp.tcpdriver import CRTPPacket
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger
from motorRaw import MotorRaw

URI = uri_helper.uri_from_env(default="radio://0/80/2M/E7E7E7E701")
URI2 = uri_helper.uri_from_env(default="radio://0/80/2M/E7E7E7E702")

if len(sys.argv) > 1:
    URI = sys.argv[1]

json_data = {"packets": [], "start_unix_time": 0.0, "start_drone_time": 0.0}

packets = []

first_packet = False


def run_drone(link_uri):
    cf = Crazyflie(rw_cache="./cache")
    with SyncCrazyflie(link_uri, cf=cf) as scf:
        scf.cf.param.set_value(complete_name="motorPowerSet.enable", value=1)
        motorRaw = MotorRaw(crazyflie=scf.cf)
        min = 8000
        motorRaw.send_motor_raw(min, min, min, min)
        time.sleep(2)
        max = 10000
        current = 9000
        inc = 50
        start = time.time()
        duration = 6
        while time.time() < start + duration:
            # pass
            print(current)
            if current > max or current < min:
                inc *= -1

            current += inc
            motorRaw.send_motor_raw(current, current, current, current)
            time.sleep(0.005)
        motorRaw.send_motor_raw(0, 0, 0, 0)


if __name__ == "__main__":
    cflib.crtp.init_drivers()
    t2 = threading.Thread(target=run_drone, args=(URI2,))
    t1 = threading.Thread(target=run_drone, args=(URI,))
    t2.start()
    t1.start()
    t1.join()
    t2.join()
