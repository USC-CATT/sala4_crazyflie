# Trajectory Plotting

`host_pid_pwm_position.py` stores the position and setpoint of the drone at each control iteration and stores it in a json object, and then saves that object to a file:
`waypoint_data.json` is created at the end of the waypoint running in `host_pid_pwm_position.py` and contains the x,y,z points of the position and setpoint of the drone
`plotter.py` contains a utility class to set up a 3d plot with the drone trajectories
`plotter_test.py` contains the logic to step through the `waypoint_data.json` file and add each point to the plot. to plot the trajectory generated in `host_pid_pwm_position` just run `python3 plotter_test.py`
