#!/bin/bash
old_dir=$PWD
cd  ~/crazyflie/crazyflie-ros/ros2_ws/build
rm -rf sala4
rm -rf sala4_bringup
cd  ~/crazyflie/crazyflie-ros/ros2_ws/
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DBUILD_TESTING=ON --symlink-install
cd $old_dir
