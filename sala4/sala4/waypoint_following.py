#!/usr/bin/env python3
#
# Adapted from crazyswarm2/crazyflie_examples

from pathlib import Path

import numpy as np
import rclpy
from crazyflie_py import Crazyswarm
from crazyflie_py.crazyflie import CrazyflieServer
from crazyflie_py.uav_trajectory import Trajectory

WAYPOINTS = [
    [[0.3, 0.1, 0.0], 0.0, 3.0],
    [[0.2, 0.3, 0.0], 1.0, 3.0],
    [[-0.3, 0.0, 0.0], 2.0, 3.0],
    [[0.0, 0.0, 0.0], 4.0, 3.0],
]
WAYPOINT_DELAY = 0.5
TOTAL_WAYPOINTS = 4


def executeTrajectory(timeHelper, cf, trajpath, rate=100, offset=np.zeros(3)):
    traj = Trajectory()
    traj.loadcsv(trajpath)
    for i, waypoint in enumerate(WAYPOINTS):
        print(f"Going to waypoint {i}")
        cf.goTo(waypoint[0], waypoint[1], waypoint[2])
        timeHelper.sleep(waypoint[2] + WAYPOINT_DELAY)


def executeTrajectoryStreamed(timeHelper, cf, trajpath, rate=100.0, offset=np.zeros(3)):
    start_time = timeHelper.time()
    total_duration = sum(waypoint[2] for waypoint in WAYPOINTS)
    current_index = 0
    current_duration = WAYPOINTS[0][2]
    while not timeHelper.isShutdown():
        t = timeHelper.time() - start_time
        if t > current_duration and current_index < TOTAL_WAYPOINTS - 1:
            current_index += 1
            current_duration += WAYPOINTS[current_index][2]
        if t > total_duration:
            break
        cf.cmdPosition(
            pos=np.array(cf.initialPosition)
            + offset
            + np.array(WAYPOINTS[current_index][0]),
            yaw=WAYPOINTS[current_index][1],
        )

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
    cf.takeoff(targetHeight=Z, duration=Z + 1.0)
    cf.goTo(pos, 0, 1.0)
    timeHelper.sleep(Z + 2.0)

    executeTrajectoryStreamed(
        timeHelper,
        cf,
        Path(__file__).parent / "data/figure8.csv",
        rate,
        offset=np.array([0, 0, 0.5]),
    )

    cf.notifySetpointsStop()
    cf.land(targetHeight=0.01, duration=Z + 1.0)
    timeHelper.sleep(Z + 2.0)
