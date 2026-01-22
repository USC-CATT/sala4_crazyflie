#!/usr/bin/env python3
#
# Adapted from crazyswarm2/crazyflie_examples

from pathlib import Path

from crazyflie_py import Crazyswarm
from crazyflie_py.crazyflie import CrazyflieServer
from crazyflie_py.uav_trajectory import Trajectory
import rclpy
import numpy as np


WAYPOINTS = [
    [[0.7, 0.1, 1.0], 0.0, 3.0],
    [[0.5, 0.6, 0.5], 1.0, 3.0],
    [[-0.5, 0.1, 1.0], 2.0, 3.0],
]
WAYPOINT_DELAY = 0.5


def executeTrajectory(timeHelper, cf, trajpath, rate=100, offset=np.zeros(3)):
    traj = Trajectory()
    traj.loadcsv(trajpath)
    for i, waypoint in enumerate(WAYPOINTS):
        print(f"Going to waypoint {i}")
        cf.goTo(waypoint[0], waypoint[1], waypoint[2])
        timeHelper.sleep(waypoint[2] + WAYPOINT_DELAY)


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
