# Building firmware
To build the crazyflie firmware, there are two steps:
1. Run `docker run --rm -u 1000 -v /var/run/docker.sock:/var/run/docker.sock -it -v /home/gary/crazyflie/crazyflie-firmware:/module bitcraze/builder tools/build/make cf21bl_defconfig` in the root `crazyflie-firmware` folder to generate config files for the brushless drone
2. Run `docker run --rm -u 1000 -v /var/run/docker.sock:/var/run/docker.sock -it -v /home/gary/crazyflie/crazyflie-firmware:/module bitcraze/builder tools/build/make` to compile the code. This is the same command with the last `cf21bl_defconfig` parameter removed.

# Flashing firmware
To flash the firmware to the crazyflie drone, ensure that the drone is turned ON and that the crazyradio is not in use.

Then, in `crazyflie-firmware`, run the following command:

```cfloader flash build/cf21bl.bin stm32-fw -w radio://0/80/2M/E7E7E7E7E7```