#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from motorRaw import MotorRaw
import time

def main():
    rclpy.init()
    motors = MotorRaw(crazyflie=None, test=True)

    for i in range(50000,65535,25):
        motors.send_motor_raw(i, i, i, i)
        time.sleep(1)
    motors.send_motor_raw(0, 0, 0, 0)
    rclpy.shutdown()

if __name__ == '__main__':
    main()