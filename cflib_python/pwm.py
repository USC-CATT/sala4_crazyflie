#!/usr/bin/env python3


import logging
import sys
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger

URI = uri_helper.uri_from_env(default="radio://0/80/2M/E7E7E7E7E7")

if len(sys.argv) > 1:
    URI = sys.argv[1]

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


if __name__ == "__main__":
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    cf = Crazyflie(rw_cache="./cache")
    with SyncCrazyflie(URI, cf=cf) as scf:
        # Arm the Crazyflie
        # scf.cf.platform.send_arming_request(True)
        time.sleep(1.0)

        scf.cf.param.set_value(complete_name="motorPowerSet.enable", value=1)
        # motor power in range of 6,000-60,000
        print("starting motors")
        scf.cf.param.set_value(complete_name="motorPowerSet.m1", value=9000)
        scf.cf.param.set_value(complete_name="motorPowerSet.m2", value=9000)
        scf.cf.param.set_value(complete_name="motorPowerSet.m3", value=9000)
        scf.cf.param.set_value(complete_name="motorPowerSet.m4", value=9000)
        # while True:
        #     i = 6000
        #     print("up")
        #     while i < 10000:
        #         i+= 1
        #         scf.cf.param.set_value(complete_name="motorPowerSet.m1", value=i)
        #         scf.cf.param.set_value(complete_name="motorPowerSet.m2", value=i)
        #         scf.cf.param.set_value(complete_name="motorPowerSet.m3", value=i)
        #         scf.cf.param.set_value(complete_name="motorPowerSet.m4", value=i)
        #         # print(i)
        #         time.sleep(0.005)
        #     print("down")
        #     while i > 6000:
        #         i-= 1
        #         scf.cf.param.set_value(complete_name="motorPowerSet.m1", value=i)
        #         scf.cf.param.set_value(complete_name="motorPowerSet.m2", value=i)
        #         scf.cf.param.set_value(complete_name="motorPowerSet.m3", value=i)
        #         scf.cf.param.set_value(complete_name="motorPowerSet.m4", value=i)
        #         # print(i)
        #         time.sleep(0.005)
                
                

        time.sleep(0.5)
        scf.cf.high_level_commander.land(0.02, 1)
