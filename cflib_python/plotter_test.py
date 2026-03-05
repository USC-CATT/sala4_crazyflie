import json
import time

import numpy as np
from plotter import TrajectoryPlot

trajectory_plotter = TrajectoryPlot()

data = {}


with open("waypoint_data.json", "r", encoding="utf-8") as file:
    data = json.load(file)
    print(len(data["setpoint_x"]))
    i = 0
    N = 10
    
    c_sp_x = np.convolve(data["setpoint_x"],  np.ones((N,))/N,  mode='same')
    c_sp_y = np.convolve(data["setpoint_y"],  np.ones((N,))/N,  mode='same')
    c_sp_z = np.convolve(data["setpoint_z"],  np.ones((N,))/N,  mode='same')
    
    c_x = np.convolve(data["position_x"],  np.ones((N,))/N,  mode='same')
    c_y = np.convolve(data["position_y"],  np.ones((N,))/N,  mode='same')
    c_z = np.convolve(data["position_z"],  np.ones((N,))/N,  mode='same')

    while i < len(c_sp_x):
        
        trajectory_plotter.addStateAndSetpoint(
            state={
                "x": c_x[i],
                "y": c_y[i],
                "z": c_z[i],
            },
            setpoint={
                "x": c_sp_x[i],
                "y": c_sp_y[i],
                "z": c_sp_z[i],
            }
        )
        i += 1
        time.sleep(.01)
    time.sleep(23)