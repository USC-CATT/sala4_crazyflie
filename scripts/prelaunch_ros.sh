#!/usr/bin/env bash
export PYTHONPATH=~crazyflie/crazyflie-firmware/build:$PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/crazyflie/crazyflie-ros/ros2_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH="/home/$USER/crazyflie/crazyflie-ros/simulation_ws/crazyflie-simulation/simulator_files/gazebo/"
echo "ROS2 is ready."