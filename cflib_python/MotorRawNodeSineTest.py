#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from motorRaw import MotorRaw
import time
import math

def main():
    rclpy.init()
    motors = MotorRaw(crazyflie=None, test=True)

    tmax = 9;
    dt = 0.01;
    N = int(tmax/dt);

    t1 = 3
    t2 = 2*t1

    T = 2*t1
    w = 2*math.pi/T
    A = 0.1
    m = 58114
    e = 0.3
    g = 9.81

    

    for i in range(N):
        t = i*dt
        if t < t1:
            u = A*w**2*m/g*math.cos(w*t) + m
            u = int(u)
        elif t < t2:
            u = int(m)
        else:
            # u = -A*w**2*m/g*math.cos(w*(t-t2)) + m
            u = -A*w**2*m/g*math.sin(2*w*(t-t2)) + m
            u = int(u)

        #u = A*math.exp(-(e*(t-10))**2)+m
        #u = int(m)
        motors.send_motor_raw(u, u, u, u)
        time.sleep(dt)
    motors.send_motor_raw(0, 0, 0, 0)
    rclpy.shutdown()

if __name__ == '__main__':
    main()