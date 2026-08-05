## Andrew's Log 01/27:
- Setup second computer
- Running Ubuntu 22.0.4
- following [This docker tutorial](https://docs.docker.com/engine/install/linux-postinstall/) to make airtaxisim work
- if docker does not work
1. ```sudo groupadd docker``` if on new computer
2. ```sudo usermod -aG docker $USER```
3. ```newgrp docker #most important line```
## Andrew's Log 02/03:
- following [This gazebo tutorial](https://gazebosim.org/api/sim/9/install.html) this to install gazebo on second computer
- created bash file to setup computer to catch up to other computer in the ~ directory
- ```bash ./crazyflie-ros2-setup.sh```
- everything needs to be in the src folder if it is crazyflie/ros
- rvis does not work yet I think
## Andrew's Log 02/10:
- /home/catt/crazyflie/crazyflie-ros/ros2_ws/install/crazyflie_description/share/crazyflie_description/local_setup.bash 
- just delete crazyflie_description if it appears delete
- followed this tutorial [here](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/building-and-flashing/build/#build-python-bindings) to have correct python bindings
- make sure to have swig installed ```sudo apt install swig```
- to run the server backend for crazyflie ```pip3 install nicegui==1.4.37``` this is the sweetspot
- the server backend now works
- look in crazyswarm crazyflie launch.py
## Andrew's Log 02/17:
- created bridge for crazyflie interface message to gazebo
## Andrew's Log 02/24:
- understanding of services are things to get published to so trying to ros bridge them by creating a service is not how they work
- we will work on creating a subscriber to this and then translating it to cmd_full_state
- if you want ros service list ```ros2 service list```
- if you want ros topic list ```ros2 topic list```
- if you want the type of topic ```ros2 topic type [topic name]``` or service ```ros2 service type [service name]``` 
- if you want the way it is formated ```ros2 interface show [name of the type]```
- next thing to do is change physics and implement goto and then maybe merge control services with crazyflie_server.py
## Andrew's Log 03/03
- keep things in the launch folder also any folder it was originally in
- to run our sim ```ros2 launch sala4_bringup crazyflie_simulation.launch.py```
- to run our test tragectory ```ros2 run sala4 trajectory_following```
- next steps are kinda from before but also to merge crazyflie_server.py with control_services.py -> sala4_control_services.py
## Andrew's Log 03/31
- changed motorRaw.py to create a ros2 topic that sends the 4 pwm from motorRaw.send_motor_raw()
- next step is to interface with gazebo make it more disconnected by making two initalizations one original and the second just ros
## Andrew's log 4/7/2026
- Changed model.sdf so that it would at least subscribe to motor_speed and that we could change each motor individually
- changed the ros_gz bridge and that made crazyflie/gazebo/command/motor_speed exist
## Andrew's log 4/14/2026
- downloaded quadcopter world and quadcopter1.sdf so that we can make motor speed work to make each motor spin
- got rid of command/gazebo/twist on teh model.sdf and now motor_speed topic works
- if you make changes to model.sdf in sala4_crazyflie you have ```cp ~/crazyflie/crazyflie-ros/ros2_ws/src/sala4_crazyflie/model/model.sdf ~/crazyflie/crazyflie-ros/simulation_ws/crazyflie-simulation/simulator_files/gazebo/crazyflie/model.sdf ```

## Zach's Log 4/16/2026
- copied `MotorRawNodeTest.py` over to `MotorRawNodeSineTest.py` to get a short demo of the drone taking off and landing
- minimum value to get the drone to take off appears to be `58115` when using `motors.send_motor_raw()`.

## Andrew's Log 7/20/2026
- looking at how plotting works it seems that `waypoint_data.json` is created by the `waypoint_logging.py` that is using a logger from cffirmware that json file is then transformed with `plot_data.py`
- to make a comparsion we must then create a simular `json` file to `waypoint_data.json`
- so would have to make a logger for the simulation that creates that `json` either by highjacking the existing file or creating a new one

## Andrew's Log 7/21/2026
- looking into how to make the platform and how to make a robot in gazebo this [link](https://gazebosim.org/docs/latest/library_howtos/) is start of making a robot but shows a car not something with pistons that we would need
- although this [youtube link](https://www.youtube.com/watch?v=wnerg4OZ1HI) shows it is possible
- want to make a stewart platform which can give us 6 degrees of freedom which may be a lot for us so other options can be explored
- [ssh key authorization tutorial](https://code.visualstudio.com/docs/remote/troubleshooting#_configuring-key-based-authentication)

## Andrew's Log 7/22/2026
- github for stewart platform [link](https://github.com/HadiYd/stewart_platform_learning/tree/main/src/stewart_platform)

## Andrew's log 8/5/2026
- tried `-s` tag in `crazyflie_simulation.launch.py` in the worlds command as well as `--headless-rendering` worked on 7/29/2026 but broke for some reason
- installed `nvidia-driver-470-server` this is a special driver to maybe fix headless rendering for ubuntu servers [source](https://github.com/google-deepmind/dm_control/issues/214) <- this did not work
- notes for gazebo topic `gz topic -l` is for the list of topics in the gazebo `gz` is main command line for gazebo
- added `prelaunch_ros.sh` option -s for headless rendering
- `source ~/.bashrc` so for changes to the bashrc
- changed `~/.bashrc` to auto prelaunch ros and make it so that primary rendering is nvidia
- all this work for nothing because all the versions are incompatable and stuff so no x11 forwarding but
- run like normal first??? then do headless maybe it makes it work it just worked an I feel like i did nothing but okay
- ROS2 Notes for getting the type a ros2 topic is `ros2 topic info /crazyflie/odom` 
- to get the output and template for the