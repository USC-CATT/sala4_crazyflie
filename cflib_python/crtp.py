#!/usr/bin/env python3


import logging
import sys
from typing import cast
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crtp.tcpdriver import CRTPPacket
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger

URI = uri_helper.uri_from_env(default="radio://0/80/2M/E7E7E7E7E7")

if len(sys.argv) > 1:
    URI = sys.argv[1]

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

def radioLinkStatistics( data):
    return
    print("radioLinkStatistics")
    print(data)

def linkError( error):
    print("linkError")
    print(error)
    

if __name__ == "__main__":
    # Initialize the low-level drivers
    radioDriver = cflib.crtp.RadioDriver()
    print("Connecting...")
    radioDriver.connect(uri=URI, radio_link_statistics_callback=radioLinkStatistics, link_error_callback=linkError)
    print("Connected")
    
    pkt = CRTPPacket()
    pkt.port = 2
    pkt.channel = 0
    pkt.data = bytearray.fromhex('00')
    suc = radioDriver.send_packet(pkt)
    if suc is False:
        print("Failed.......................")
    pak = radioDriver.receive_packet(-1)
    print("Now getting all of the parameters")
    for  i in range(0, 32):
        pkt.data = bytearray.fromhex('01')
        suc = radioDriver.send_packet(pkt)
        if suc is False:
            print("Failed.......................")
        # print("Waiting for response")
        pak = cast(CRTPPacket, radioDriver.receive_packet(-1))
        if(pak.port == 2):
            print(pak.data)
        
        
    
    # time.sleep(0.1)
        
    radioDriver.close()