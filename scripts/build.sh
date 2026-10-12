#!/bin/bash
cd  ~/crazyflie/crazyflie-ros/ros2_ws/
rm -rf build/sala4
rm -rf build/sala4_bringup
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DBUILD_TESTING=ON --symlink-install
cd ~/crazyflie/crazyflie-ros/ros2_ws/src/sala4_crazyflie
