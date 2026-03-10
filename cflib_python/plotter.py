import matplotlib.pyplot as plt
import numpy as np
import time
import threading
import os
from collections import deque
import json


data = {}
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WAYPOINT_DATA_PATH = os.path.join(SCRIPT_DIR, "waypoint_data.json")

try:
    with open(WAYPOINT_DATA_PATH, "r", encoding="utf-8") as file:
        data = json.load(file)
except FileNotFoundError:
    data = {}


class TrajectoryPlot:
    def __init__(self):
        self.xs = []
        self.ys = []
        self.zs = []
        self.xs_setpoint = []
        self.ys_setpoint = []
        self.zs_setpoint = []
        
        self.lock = threading.Lock()
        self.running = True
        
        self.data_thread = threading.Thread(target=self._plot_loop, daemon=True)
        self.data_thread.start()
    
    def _data_loop(self):
        pass
        
    def _plot_loop(self):
        plt.ion()
        plt.show()
        
        self.fig = plt.figure(figsize=(10, 10))
        self.ax = self.fig.add_subplot(projection="3d")
        self.line = self.ax.plot([], [], [], "b-")[0]
        self.line2 = self.ax.plot([], [], [], "g-")[0]
        
        self.ax.set_xlim(-1, 1)
        self.ax.set_ylim(-1, 1)
        self.ax.set_zlim(0, 1)
        while self.running:
            with self.lock:
                xs = list(self.xs)
                ys = list(self.ys)
                zs = list(self.zs)
                xs_setpoint = list(self.xs_setpoint)
                ys_setpoint = list(self.ys_setpoint)
                zs_setpoint = list(self.zs_setpoint)

            if xs:
                self.line.set_data(xs, ys)
                self.line.set_3d_properties(zs)
                self.line2.set_data(xs_setpoint, ys_setpoint)
                self.line2.set_3d_properties(zs_setpoint)

            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            time.sleep(0.05)  # ~20 fps
        
    def addStateAndSetpoint(self, state, setpoint):
        with self.lock:
            self.xs.append(state["x"])
            self.ys.append(state["y"])
            self.zs.append(state["z"])
            self.xs_setpoint.append(setpoint["x"])
            self.ys_setpoint.append(setpoint["y"])
            self.zs_setpoint.append(setpoint["z"])
        pass
    def setPaths(self, x, y, z, x_s, y_s, z_s):
        with self.lock:
            self.xs.clear()
            self.ys.clear()
            self.zs.clear()
            self.xs_setpoint.clear()
            self.ys_setpoint.clear()
            self.zs_setpoint.clear()
            self.xs = x
            self.ys = y
            self.zs = z
            self.xs_setpoint = x_s
            self.ys_setpoint = y_s
            self.zs_setpoint = z_s
