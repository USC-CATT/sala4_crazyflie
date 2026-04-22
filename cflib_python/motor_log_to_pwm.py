#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from motorRaw import MotorRaw
import time
import pandas as pd
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

def main():
    pd.set_option('display.max_rows', None)
    rclpy.init()
    motors = MotorRaw(crazyflie=None, test=True, using_json_log=True)
    df = pd.read_json(os.path.join(SCRIPT_DIR, "motor_raw_log.json"))
    entries = df["velocities"].to_list()
    for i, entry in enumerate(df["velocities"]):
        print(f"m1: {entry['m1']}, m2: {entry['m2']}, m3: {entry['m3']}, m4: {entry['m4']}, timestamp: {entry['timestamp']}")
        motors.send_motor_raw(entry['m1'], entry['m2'], entry['m3'], entry['m4'])
        if i + 1 < len(entries) - 1:
            dt = entries[i + 1]['timestamp'] - entry['timestamp']
        else:
            dt = 0
        time.sleep(dt)
    motors.send_motor_raw(0, 0, 0, 0)
    rclpy.shutdown()

if __name__ == '__main__':
    main()