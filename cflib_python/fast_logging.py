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

p_time = time.time()
def log_stab_callback(timestamp, data, logconf):
    # return
    global first_packet, json_data, p_time
    if first_packet is False:
        first_packet = True
        json_data["start_drone_time"] = timestamp
    data["setpoint_vx"] = setpoint_vx
    data["setpoint_vy"] = setpoint_vy
    data["packet_timestamp"] = timestamp                    # raw timestamp received from drone when logged, drones timestamp
    data["command_timestamp"] = int(setpoint_timestamp * 1000)   # timestamp from when we sent the command to the drone
    data["log_processed_timestamp"] = int(time.time()  * 1000)   # timestamp from when we receive a packet and have processed it
    packets.append(data)
    print(f"time since last {int((time.time() - p_time) * 1000)} {timestamp}")
    p_time = time.time()
    print('%s\n\n' % (data))
    


if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    lg_stab = LogConfig(name='State Estimator', period_in_ms=50)
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
        
        cf.platform.send_arming_request(False)
        time.sleep(1.0)



        print("Starting logging")
        
        lg_stab.start()
        
        json_data["start_unix_time"] =int( time.time() * 1000)
            
        time.sleep(100.0)
            
        lg_stab.stop()
        
        file_path = 'data.json'
        json_data["packets"] = packets
        with open(file_path, 'w') as json_file:
            json.dump(json_data, json_file, indent=4)
