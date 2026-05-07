# Trajectory Plotting

`host_pid_pwm_position.py` stores the position and setpoint of the drone at each control iteration and stores it in a json object, and then saves that object to a file:

`waypoint_data.json` is created at the end of the waypoint running in `host_pid_pwm_position.py` and contains the x,y,z points of the position and setpoint of the drone

`plot_data.py` contains all logic necessary to plot two drone trajectories or just one. It has the following parameters:

`--augmented_file=[path]` a path to the augmented file to plot

`--pid_file=[path]`       a path to the PID file to plot. if you give either augmented OR pid, you need to pass both, otherwise it will fail.

> _Not passing any parameters has it use the default `waypoint_data.json` file._


`--cutoff=[seconds]`      a cutoff in seconds that cuts the end of the data file after this point

`--offset=[seconds]`      an offset value in seconds that helps with aligning augmented/pid files. negative values cut from the start of the PID file, positive values cut from the start of the augmented file

`--start=[seconds]`       a start value that cuts from the beginning of both pid and augmented files

