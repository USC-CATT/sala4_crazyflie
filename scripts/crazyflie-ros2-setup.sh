#!/bin/bash
mkdir ~/crazyflie
cd ~/crazyflie
mkdir crazyflie-ros
cd crazyflie-ros
mkdir simulation_ws
mkdir ros2_ws
cd ros2_ws
mkdir src
cd ~/crazyflie/crazyflie-ros/simulation_ws
git clone https://github.com/bitcraze/crazyflie-simulation.git
cd ~/crazyflie/crazyflie-ros/ros2_ws/src
git clone https://github.com/knmcguire/crazyflie_ros2_multiranger.git
git clone https://github.com/knmcguire/ros_gz_crazyflie
git clone https://github.com/IMRCLab/crazyswarm2 --recursive
sudo apt-get install libboost-program-options-dev libusb-1.0-0-dev python3-colcon-common-extensions
sudo apt-get install ros-humble-motion-capture-tracking ros-humble-tf-transformations
sudo apt-get install ros-humble-ros-gzharmonic ros-humble-teleop-twist-keyboard
pip3 install cflib transform3D
cd  ~/crazyflie/crazyflie-ros/ros2_ws/
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DBUILD_TESTING=ON
echo "setup complete"
