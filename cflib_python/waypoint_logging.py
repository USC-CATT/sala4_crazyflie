import logging
import time

import math
import json
from tkinter.constants import FALSE
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M/E7E7E7E7E7'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


def is_close(range):
    MIN_DISTANCE = 0.2  # m

    if range is None:
        return False
    else:
        return range < MIN_DISTANCE


WAYPOINTS = [
    [[0.8, 0.3, 0.0], 0.0, 2.0],
    [[0.4, 0.6, 0.0], 45.0, 2.0],
    [[-0.5, 0.0, 0.0], 0.0, 2.0],
    [[0.0, 0.0, 0.0], -45.0, 2.0],
]
WAYPOINT_DELAY = 2
TOTAL_WAYPOINTS = 4

setpoint_vx = 0.0
setpoint_vy = 0.0
setpoint_timestamp = time.time()

json_data = {"packets": [], "start_unix_time": 0.0, "start_drone_time": 0.0}

packets = []

first_packet = False


def log_stab_callback(timestamp, data, logconf):
    # return
    global first_packet, json_data
    if first_packet is False:
        first_packet = True
        json_data["start_drone_time"] = timestamp
    data["setpoint_vx"] = setpoint_vx
    data["setpoint_vy"] = setpoint_vy
    data["packet_timestamp"] = timestamp                    # raw timestamp received from drone when logged, drones timestamp
    data["command_timestamp"] = int(setpoint_timestamp * 1000)   # timestamp from when we sent the command to the drone
    data["log_processed_timestamp"] = int(time.time()  * 1000)   # timestamp from when we receive a packet and have processed it
    packets.append(data)
    print('%s\n\n' % (data))
    

def fly_straight(mc, vx, vy, duration):
    dt = 1.0 / 50  # 20 ms
    steps = int(duration * 50)

    for _ in range(steps):
        global setpoint_vx, setpoint_vy, setpoint_timestamp
        setpoint_vx = vx
        setpoint_vy = vy
        setpoint_timestamp = time.time()
        mc.start_linear_motion(vx, vy, 0, 0)
        time.sleep(dt)

    mc.stop()




def fly_arc(mc, omega, radius, angle, clockwise):
    """
    omega : angular speed (rad/s)
    radius : radius of the arc (m)
    angle : total angle drone will cover (rad)
    clockwise : True for CW, False for CCW
    """
    direction = -1 if clockwise else 1
    duration = angle / omega
    dt = 1.0 / 50
    steps = int(duration * 50)

    for k in range(steps):
        alpha = direction * omega * k * dt
        vx = -omega * radius * math.sin(alpha)
        vy =  omega * radius * math.cos(alpha)
        global setpoint_vx, setpoint_vy, setpoint_timestamp
        setpoint_vx = vx
        setpoint_vy = vy
        setpoint_timestamp = time.time()
        mc.start_linear_motion(vx, vy, 0, 0)
        time.sleep(dt)

    mc.stop()


def trajectory(mc):
    v = 0.2         # straight-line speed (m/s)
    r_safe = 0.6   # safe turning radius (m)
    omega = 0.8     # angular speed (rad/s)
    angle = math.pi / 2  # 90 degrees (rad)

    T_straight = r_safe / v  # time to go r_safe meters at speed v

    # -------------------------
    # Segment I: straight +x
    # -------------------------
    fly_straight(mc, vx=v, vy=0, duration=T_straight)

    # -------------------------
    # Segment II: 90° arc turn
    # (pick clockwise based on your path diagram)
    # -------------------------
    fly_arc(mc, omega=omega, radius=r_safe, angle=angle, clockwise=False)

    # -------------------------
    # Segment III: straight +y
    # -------------------------
    fly_straight(mc, vx=0, vy=-v, duration=2*T_straight)
    time.sleep(2.0)
    # -------------------------
    # Segment IV: 90° arc turn back
    # -------------------------
    fly_arc(mc, omega=omega, radius=r_safe, angle=angle, clockwise=False)
    #fly_straight(mc, vx=v, vy=0, duration=T_straight)


if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    lg_stab = LogConfig(name='State Estimator', period_in_ms=60)
    # We use FP16 to fit more variables in a single log packet
    lg_stab.add_variable('stateEstimate.vx', 'FP16')
    lg_stab.add_variable('stateEstimate.vy', 'FP16')
    lg_stab.add_variable('stateEstimate.vz', 'FP16')
    lg_stab.add_variable('stateEstimate.x', 'FP16')
    lg_stab.add_variable('stateEstimate.y', 'FP16')
    lg_stab.add_variable('stateEstimate.z', 'FP16')
    # lg_stab.add_variable('stateEstimate.roll', 'FP16')
    # lg_stab.add_variable('stateEstimate.pitch', 'FP16')
    # lg_stab.add_variable('stateEstimate.yaw', 'FP16')
    lg_stab.add_variable('gyro.x', 'FP16')
    lg_stab.add_variable('gyro.y', 'FP16')
    lg_stab.add_variable('gyro.z', 'FP16')
    lg_stab.add_variable('acc.x', 'FP16')
    lg_stab.add_variable('acc.y', 'FP16')
    lg_stab.add_variable('acc.z', 'FP16')

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:

        cf = scf.cf
        cf.log.add_config(lg_stab)
        lg_stab.data_received_cb.add_callback(log_stab_callback)
        
        cf.platform.send_arming_request(True)
        time.sleep(1.0)



        print("Starting logging")
        
        lg_stab.start()
        
        json_data["start_unix_time"] =int( time.time() * 1000)
            
        with MotionCommander(scf, default_height=0.17) as mc:
            time.sleep(2.0)
            trajectory(mc)
            time.sleep(1.0)
            mc.stop()
            
            
        lg_stab.stop()
        
        file_path = 'data.json'
        json_data["packets"] = packets
        with open(file_path, 'w') as json_file:
            json.dump(json_data, json_file, indent=4)
