# Oct. 23

- ROS Workspace is completely set up, including Gazebo Harmonic and Crazyswarm ROS libraries.
- Successfully ran simulation in RViz and Gazebo, for simple wall mapping
- Attempted to run ROS with the live drone, but encountered an issue where the drone's roll appears to drift heavily
  - Observing using the `cfclient` plotter, it looks like the roll increases at a steady rate and then resets after a certain amount of seconds, or after a certain increment of 9 degrees.
  - From [this github issue](https://github.com/orgs/bitcraze/discussions/1160), it appears that this could be an issue with the flow deck's mounting
    - Testing the crazyflie with the flow deck and turning it off and on to recalibrate produced the same ramping Pose in the plotter each time
    - Testing the crazyflie withOUT the flow deck, and turning it off and on again to recalibrate produced the same ramping pose each time, but a different ramping pose from the plot with the flow deck.
  - This indicates to me that there could be some conflicting pose estimation between the decks, which is evidenced by [this article](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/sensor-to-control/state_estimators/) which leads us to believe that we need to change to the Kalman estimator in code
  - Another issue was that there was a bad array access within the example code, which didnt seem to be fixed after changing the code. Need to investigate that further

# Oct. 24

Questions to ask Dr. Culbertson:

- whether or not ros can support realtime - building digital twin (strict timing constraints), and if that timing is simulated
- understanding abstraction of gazebo models and how the models interpret commands sent to the actual robot/drone (does it use the firmware on the drone itself?)
  Found PID values in firmware:
- https://github.com/bitcraze/crazyflie-firmware/blob/master/src/platform/interface/platform_defaults_cf21bl.h
  as well as assorted gazebo models and worlds here
- https://github.com/knmcguire/ros_gz_crazyflie
  began setting up firmware compiling toolchain
- installed and set up Docker and [Toolbelt](https://github.com/bitcraze/toolbelt?tab=readme-ov-file)
- Alias created for 'tb' in bashrc
  building pipeline:
- adapted from [here](https://github.com/bitcraze/crazyflie-firmware/blob/master/docs/building-and-flashing/build.md)
- inside `~/crazyflie/crazyflie-firmware` run `tb compile` to compile firmware
- run `make cf2_defconfig` to make function definitions
- run `make bindings_python` to make python firmware binding file located at `./build/cffirmware.py`
  this cffirmware.py file imports the `_cffirmware.cpython[assorted version data].so` shared library.

# Nov. 13

- encountering some weird things where the drone flies around like crazy when the flow deck is attached
- the following forum post leads me to believe that there might be some conflict between it and the lighthouse
- https://forum.bitcraze.io/viewtopic.php?t=4197
- could be useful: https://www.bitcraze.io/documentation/system/platform/cf2-expansiondecks/#compatibility-matrixes
- https://github.com/bitcraze/crazyflie-firmware/issues/806

# Jan. 22
- We discovered that editing crazyflie_cpp in crazyswarm2 (which is two submodules deep) allows us to modify the controls of the drone while using the crazyswarm2 python library. it needs to be built before it works but adding a return to the sendFullState function disables the fullstate functionality of the drone.
- this can be used to potentially convert the fullstate from a modeAbs to modeVel control to have velocity and acceleration be used instead of position
- Waypoint follower now works