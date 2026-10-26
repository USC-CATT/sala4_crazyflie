# Setting up this repo on a new machine

## Pre-requisites

* Ubuntu 22.04 Jammy Jellyfish
* ROS 2 Humble Hawksbill
* Gazebo Harmonic
* Python 3.10
* swig
* nicegui 1.4.37
/
## If you dont have the pre-reqs installed

1. Start by downloading Ubuntu 22.04 from the [official source](https://releases.ubuntu.com/jammy/)
2. Install that to a bootable drive, follow [instructions](https://ubuntu.com/desktop/docs/en/latest/tutorial/install-ubuntu-desktop/) from Ubuntu if you're unsure how to do that.
3. Boot the machine of interest using your bootable drive and follow the instructions to install.
4. Reboot the machine and follow the setup instructions for Ubuntu
5. Follow the [intructions from ROS](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) to install Ros 2 Humble Hawksbill    
6. Follow the [instructions from Gazebo](https://gazebosim.org/docs/harmonic/install_ubuntu/#binary-installation-on-ubuntu) to install Gazebo Harmonic (they recommend using Fortress when paired with Ros 2 Humble, but we need to use Harmonic due to a different dependency that I can't remember at the time of writing)
7. Check Python version is 3.10 with `python --version`. If not 3.10, you mightneed to update it to be 3.10, or create a virtual environment which uses python 3.10. It looks like 3.10 is the default for Ubuntu 22.04 though, so you shouldbe fine.
8. Install swig with `sudo apt install swig`
9. Install pip with `sudo apt install python3-pip`
10. Install nicegui with `pip install nicegui==1.4.37`
11. Follow the [Github instructions](https://docs.github.com/en/authentication/connecting-to-github-with-ssh) to setup ssh keys and connect them to your github account

## Starting from cloning this repo

0. Navigate to the top-level directory youwant this repo to live in
1. Clone the repo with `git clone git@github.com:USC-CATT/sala4_crazyflie.git`

### Notes from installation with Andrew
Starting inside `sala4_crazyflie`
```
./scripts/crazyflie-ros2-setup.sh
```

Then move `sala4_crazyflie`
```
mv ~/path/to/sala4_crazyflie 
~/crazyflie/crazyflie-ros/ros2_ws/src/sala4_crazyflie
```
```
cd ~/crazyflie/crazyflie-ros/ros2_ws/src/sala4_crazyflie
```

Now run `build.sh`
```
source scripts/build.sh
```

Hopefully that ran without errors (we had a bunch when we tried...), next should 
be to do the pre-launch routine

```
source scripts/prelaunch_ros.sh
```

That should run without issue. Then to launch the simulation we run
```
ros2 launch sala4_bringup crazyflie_simulation.launch.py
```

### Issues we ran into
We had an issue where we couldn't access `ros-humble-motion-capture-tracking`, 
even though we explicitly try to install it in `crazyflie-ros2-setup.sh`. It 
turns out some things were getting installed at the user level, and some at the 
root level. 

We ended up fixing it by removing the user version of python, as follows
```
pip3 freeze --user > ~/pip3-backup.txt
mv ~/.local/lib/python3.10 ~/.local/lib/python3.10.bak
rm -rf ~/.cache/pip
```

We also had to install `rowan` and `ros-humble-crazyflie*` as follows:
```
pip3 install rowan
sudo apt-get install ros-hgumble-crazyflie*
```

We then ran `build.sh` again, and it seemed to work.
