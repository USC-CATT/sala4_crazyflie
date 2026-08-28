#!/usr/bin/env bash
export PYTHONPATH=~/crazyflie/crazyflie-firmware/build:$PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/crazyflie/crazyflie-ros/ros2_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH="/home/$USER/crazyflie/crazyflie-ros/simulation_ws/crazyflie-simulation/simulator_files/gazebo/"
export EGL_PLATFORM=surfaceless

#trying to run headless
# export LIBGL_ALWAYS_SOFTWARE=1
# export GALLIUM_DRIVER=llvmpipe
# export MESA_GL_VERSION_OVERRIDE=4.5
# export LIBGL_DRI3_DISABLE=1
# if [ $1 == "-s" ]; then
#     # export LIBGL_ALWAYS_SOFTWARE=1
#     export LIBGL_DRI3_DISABLE=1
#     echo "ROS2 headless is ready"
# else
#     echo "ROS2 is ready."
# fi