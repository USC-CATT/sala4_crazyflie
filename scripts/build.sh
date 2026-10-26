#!/bin/bash
cd  ~/crazyflie/crazyflie-ros/ros2_ws/
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DBUILD_TESTING=ON --symlink-install
cd ~/crazyflie/crazyflie-ros/ros2_ws/src/sala4_crazyflie
