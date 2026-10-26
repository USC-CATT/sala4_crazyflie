# Getting ready to fly

Notes from session with Noah

## Drone identification
In our lab, the Drone # is written on the propellers

The address is on the underside of the props
i.e. `E7x4 01` means the address is `E7E7E7E701` for drone #1

## LED Indications
When you first power on the drone, we expect the lighthouse LED to signal RED, 
then YELLOW, then GREEN.

You can refer to the Bitcraze docs for the standarad LED states, but the normal 
expectation is 2 BLUE lights on motors 2 and 3 (the back two), and 1 steady 
flashing light on M1 (the heartbeat pattern). The lighthouse LEDs should show 
GREEN and BLUE (signal is good and we have power).

If the M1 LED gives you 2 blinks with a long pause, there has been a 
catastrophic error, and you need to restart the drone.

## Setup Procedure

Place drone in center of space (use tape on ground for easy calibration. The L 
indicates the origin, with the long end facing in the positive X-direction. The 
fron of the drone should face positive X)

On the computer, run 
```
conda activate py310
cfclient
```

Check `View > Tabs > Lighthouse Positioning` to adjust which tabs are available.  
You may want `Plotter` and `Parameters`. 

Set the address for the drone:
`0xE7E7E7E702` for drone 2.

Press `Scan`, then `Connect`.

In the Lighthouse positioning tab, does the drone look like it's stable? If not, 
you need to calibrate.

When looking at the Lighthouse beacons, a flashing blue LED indicates it is 
searching, and Green means good. 

Note: lighthouses need to be at roughly the same height

## Calibration

`Manage Geometry` to begin calibration. Follow the instructions on screen toi 
get started.

Place drone at origin, facing the x-direction (may be done already). 

`Start Measurement`

Note: red tape is exactly 1 meter, sop just place it at the end of the red tape 
for the 1 meter measurement.

Follow the remainder of the instructions on screen.

When complete, use `Estimate Geometry` to see the coordinates of the lighthouse 
beacons relative to the origin.

Then, write to the crazyflie so it knows where it is.

## Running python scripts.

See [these instructions](./cflib_python/instructions.md) to run various python 
scripts, including those which will make the drone fly.

Note: we built in a kill function tied to the space bar, but it only works if 
you are clicked in to the terminal that is running the script.

The scripts which can run a demo are:
`augmented_pid_pwm_position.py`
`pid_pwm_position.py`

To change the path the demo follows, look for the variable 
`USER_DEFINED_TRAJECTORY`, and change the waypoints. Each row of the trajectory 
is in the form

```
[ time, x, y, y, psi]
```
Where time is the time the drone should be at that waypoint, measured from the 
start of the script (t=0 is when the script start running), (x, y, z) are the 
spatial coordinates of the waypoint as defined in the frame calibrated 
previously, and `psi` is the yaw heading angle of the drone in degrees, measured 
with psi=0 pointing in the positive x direction, and positive yaw turning the 
drone to the right.

Once you have edited the waypoints as desired, save the file, and run with
```
./cflib/scriptname.py
```
assuming you are located in the `sala4_crazyflie` directory when starting.
