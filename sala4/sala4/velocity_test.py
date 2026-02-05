#!/usr/bin/env python3
#
# Adapted from crazyswarm2/crazyflie_examples

from pathlib import Path

import numpy as np
import rclpy
from crazyflie_py import Crazyswarm
from crazyflie_py.crazyflie import CrazyflieServer
from crazyflie_py.uav_trajectory import Trajectory


def cmdVelStreamed(timeHelper, cf, rate=100.0):
    start_time = timeHelper.time()
    while not timeHelper.isShutdown():
        t = timeHelper.time() - start_time
        if t > 2:
            break
        # cf.cmdFullState(
        #     pos=[0.0,0.0,0.2], vel=[0.0, 0.0, 0.0], acc=np.zeros(3), yaw=1.4, omega=[0.0,0.0, 5.0]
        # )
        cf.cmdVel(roll=0.0,pitch=0.0,yawRate=10.0,thrust=45000.0)
        timeHelper.sleepForRate(rate)


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]
    print("Arming crazyflie")
    cf.arm()

    rate = 30.0
    Z = 0.5
    print("Attempting takeoff")
    # cf.takeoff(targetHeight=0.5, duration=1.5)
    # cf.goTo([0.0,0.0,0.2],0.0,1.0)
    timeHelper.sleep(3.0)

    print("Streaming velocity")

    cmdVelStreamed(
        timeHelper,
        cf,
        rate,
    )
    print("Finished streaming velocity")

    cf.notifySetpointsStop()
    # cf.land(targetHeight=0.01, duration=Z + 1.0)
    timeHelper.sleep(Z + 2.0)
