import logging
import struct
import sys
import time
from typing import cast

import cflib.crtp
import json
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crtp.crtpstack import CRTPPacket
from cflib.crtp.tcpdriver import CRTPPacket
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger
from cflib.crazyflie.log import LogConfig
from motorRaw import MotorRaw

URI = uri_helper.uri_from_env(default="radio://0/80/2M/E7E7E7E7E7")
logging.basicConfig(level=logging.ERROR)

if len(sys.argv) > 1:
    URI = sys.argv[1]

json_data = {"packets": [], "start_unix_time": 0.0, "start_drone_time": 0.0}

packets = []

first_packet = False


def log_stab_callback(timestamp, data, logconf):
    # return
    global first_packet, json_data
    if first_packet is False:
        first_packet = True
        json_data["start_drone_time"] = timestamp
    data["packet_timestamp"] = timestamp                    # raw timestamp received from drone when logged, drones timestamp
    data["log_processed_timestamp"] = int(time.time()  * 1000)   # timestamp from when we receive a packet and have processed it
    packets.append(data)
    print('%s\n\n' % (data))


def radioLinkStatistics(data):
    return
    print("radioLinkStatistics")
    print(data)


def linkError(error):
    print("linkError")
    print(error)


if __name__ == "__main__":
    cflib.crtp.init_drivers()
    
    lg_stab = LogConfig(name='State Estimator', period_in_ms=60)
    # We use FP16 to fit more variables in a single log packet
    lg_stab.add_variable('stateEstimate.vx', 'FP16')
    lg_stab.add_variable('stateEstimate.vy', 'FP16')
    lg_stab.add_variable('stateEstimate.vz', 'FP16')
    lg_stab.add_variable('stateEstimate.x', 'FP16')
    lg_stab.add_variable('stateEstimate.y', 'FP16')
    lg_stab.add_variable('stateEstimate.z', 'FP16')
    lg_stab.add_variable('gyro.x', 'FP16')
    lg_stab.add_variable('gyro.y', 'FP16')
    lg_stab.add_variable('gyro.z', 'FP16')
    lg_stab.add_variable('acc.x', 'FP16')
    lg_stab.add_variable('acc.y', 'FP16')
    lg_stab.add_variaable('acc.z', 'FP16')

    cf = Crazyflie(rw_cache="./cache")
    motorRaw = MotorRaw(crazyflie=cf)
    with SyncCrazyflie(URI, cf=cf) as scf:
        cf.log.add_config(lg_stab)
        lg_stab.data_received_cb.add_callback(log_stab_callback)
        scf.cf.param.set_value(complete_name="motorPowerSet.enable", value=1)
        min = 8000
        max = 10000
        current = 9000
        inc = 50
        start = time.time()
        duration = 1
        lg_stab.start()
        while time.time() < start + duration:
            # pass
            print(current)
            if current > max or current < min:
                inc *= -1
                
            current += inc
            motorRaw.send_motor_raw(current, current, current, current)
            time.sleep(.01)
        motorRaw.send_motor_raw(0, 0, 0, 0)
        lg_stab.stop()

        file_path = 'motorRawData.json'
        json_data["packets"] = packets
        with open(file_path, 'w') as json_file:
            json.dump(json_data, json_file, indent=4)