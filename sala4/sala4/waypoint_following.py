#!/usr/bin/env python3
# 
# Adapted from crazyswarm2/crazyflie_examples

from pathlib import Path

from crazyflie_py import Crazyswarm
from crazyflie_py.crazyflie import CrazyflieServer
from crazyflie_py.uav_trajectory import Trajectory
import rclpy
import numpy as np


def executeTrajectory(timeHelper, cf, trajpath, rate=100, offset=np.zeros(3)):
    traj = Trajectory()
    traj.loadcsv(trajpath)
    while not timeHelper.isShutdown():
        cf.cmdPosition(np.array([1.0,0.0,1.0]))
        
        timeHelper.sleepForRate(rate)


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]
    print("Arming crazyflie")
    cf.arm()

    rate = 30.0
    Z = 0.5
    pos = np.array(cf.initialPosition) + np.array([0, 0, Z])
    
    print("Attempting takeoff")
    cf.takeoff(targetHeight=Z, duration=Z+1.0)
    cf.goTo(pos, 0, 1.0)
    timeHelper.sleep(Z+2.0)

    executeTrajectory(timeHelper, cf,
                      Path(__file__).parent / 'data/figure8.csv',
                      rate,
                      offset=np.array([0, 0, 0.5]))

    cf.notifySetpointsStop()
    cf.land(targetHeight=0.03, duration=Z+1.0)
    timeHelper.sleep(Z+2.0)