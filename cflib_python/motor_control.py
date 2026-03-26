import json
import logging
import math
import struct
import sys
import keyboard
import time
import time
from dataclasses import dataclass, field
from tkinter.constants import FALSE
from typing import cast

import cflib.crtp
import numpy as np
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crtp.crtpstack import CRTPPacket
from cflib.crtp.tcpdriver import CRTPPacket
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger
from controller.controller_pid import ControllerPID
from controller.controller_types import (
    AccData,
    Attitude,
    AttitudeRate,
    Axis3f,
    Control,
    GyroData,
    Position,
    Quaternion,
    SensorData,
    Setpoint,
    SetpointMode,
    StabMode,
    State,
    Velocity,
)
from motorRaw import MotorRaw

URI = uri_helper.uri_from_env(default="radio://0/80/2M/E7E7E7E7E7")
logging.basicConfig(level=logging.ERROR)

if len(sys.argv) > 1:
    URI = sys.argv[1]


@dataclass
class MotorThrust:
    motor_1: int = 0
    motor_2: int = 0
    motor_3: int = 0
    motor_4: int = 0


# Minimum and maximum thrust per motor
# Note: The maximum thrust is a trade-off between consistency of thrust over all battery levels
# and maximum performance with a full battery. Increase this value at your own risk. More info
# in this PR: https://github.com/bitcraze/crazyflie-firmware/pull/1526
# or this blog post: https://www.bitcraze.io/2025/10/keeping-thrust-consistent-as-the-battery-drains/
# Taken from platform_defaults_cf21bl.h
THRUST_MIN = 0.02136263065537499  # N (per motor)
THRUST_MAX = 0.2  # N (per motor)
# Thrust curve coefficients (per motor)
VMOTOR2THRUST0 = -0.014058926705279723
VMOTOR2THRUST1 = 0.04265273261724981
VMOTOR2THRUST2 = 0.0018327760144017432
VMOTOR2THRUST3 = 0.0020576974784587178
THRUST2TORQUE = 0.00569278844371417

IDLE_THRUST = 7000

UINT16_MAX = 65535


class PWMMotorController:
    def __init__(self):
        self.controller = ControllerPID()
        self.controller.init()

        self.stabilizer_step = 1

        self.cf_state = State()
        self.cf_state.position = Position(x=0.0, y=0.0, z=0.0)
        self.cf_state.velocity = Velocity(x=0.0, y=0.0, z=0.0)
        self.cf_state.acc = Axis3f(x=0.0, y=0.0, z=0.0)
        self.cf_state.attitude = Attitude(roll=0.0, pitch=0.0, yaw=0.0)
        self.cf_state.attitude_quaternion = Quaternion(x=0.0, y=0.0, z=0.0, w=0.0)

        self.cf_sensors = SensorData()
        self.cf_sensors.gyro = GyroData(x=0.0, y=0.0, z=0.0)
        self.cf_sensors.acc = AccData(x=0.0, y=0.0, z=0.0)

        self.cf_setpoint = Setpoint()
        self.cf_setpoint.position = Position(x=0.0, y=0.0, z=1.0) 
        self.cf_setpoint.velocity = Velocity(x=0.0, y=0.0, z=0.0)
        self.cf_setpoint.attitude = Attitude(roll=0.0, pitch=0.0, yaw=0.0)
        self.cf_setpoint.attitude_rate = AttitudeRate(roll=0.0, pitch=0.0, yaw=0.0)
        self.cf_setpoint.thrust = 0.0
        self.cf_setpoint.velocity_body = False

        self.cf_setpoint.mode = SetpointMode()
        self.cf_setpoint.mode.x = StabMode.MODE_ABS  # Position control
        self.cf_setpoint.mode.y = StabMode.MODE_ABS  # Position control
        self.cf_setpoint.mode.z = StabMode.MODE_ABS  # Position control
        self.cf_setpoint.mode.roll = StabMode.MODE_DISABLE
        self.cf_setpoint.mode.pitch = StabMode.MODE_DISABLE
        self.cf_setpoint.mode.yaw = StabMode.MODE_ABS

        self.cf_vbat = 0.0

        self.control = Control()

        self.lg_state = LogConfig(name="State Estimator", period_in_ms=60)
        self.lg_sensor = LogConfig(name="Sensor Values", period_in_ms=60)
        # We use FP16 to fit more variables in a single log packet
        self.lg_state.add_variable("stateEstimate.vx", "FP16")
        self.lg_state.add_variable("stateEstimate.vy", "FP16")
        self.lg_state.add_variable("stateEstimate.vz", "FP16")
        self.lg_state.add_variable("stateEstimate.ax", "FP16")
        self.lg_state.add_variable("stateEstimate.ay", "FP16")
        self.lg_state.add_variable("stateEstimate.az", "FP16")
        self.lg_state.add_variable("stateEstimate.x", "FP16")
        self.lg_state.add_variable("stateEstimate.y", "FP16")
        self.lg_state.add_variable("stateEstimate.z", "FP16")
        self.lg_state.add_variable("stateEstimate.qx", "FP16")
        self.lg_state.add_variable("stateEstimate.qy", "FP16")
        self.lg_state.add_variable("stateEstimate.qz", "FP16")
        self.lg_state.add_variable("stateEstimate.qw", "FP16")

        self.lg_sensor.add_variable("gyro.x", "float")
        self.lg_sensor.add_variable("gyro.y", "float")
        self.lg_sensor.add_variable("gyro.z", "float")
        self.lg_sensor.add_variable("acc.x", "float")
        self.lg_sensor.add_variable("acc.y", "float")
        self.lg_sensor.add_variable("acc.z", "float")
        self.lg_sensor.add_variable("pm.vbat", "FP16")

        self.cf = Crazyflie(rw_cache="./cache")
        self.motorRaw = MotorRaw(crazyflie=self.cf)

    def init(self):
        cflib.crtp.init_drivers()
        self.sync_crazyflie()

    def arm_motors(self):
        self.motorRaw.send_motor_raw(IDLE_THRUST, IDLE_THRUST, IDLE_THRUST, IDLE_THRUST)

    def stop_motors(self):
        cooldown_start = time.time()
        cooldown_duration = 1
        while time.time() < cooldown_start + cooldown_duration:
            self.motorRaw.send_motor_raw(0, 0, 0, 0)
            time.sleep(0.01)

    def sync_crazyflie(self):
        with SyncCrazyflie(URI, cf=self.cf) as scf:
            self.cf.log.add_config(self.lg_state)
            self.lg_state.data_received_cb.add_callback(self.log_state_callback)
            self.cf.log.add_config(self.lg_sensor)
            self.lg_sensor.data_received_cb.add_callback(self.log_sensor_callback)

            scf.cf.param.set_value(complete_name="motorPowerSet.enable", value=1)
            self.spinup()
            
            start = time.time()
            duration = 5
            self.lg_state.start()
            self.lg_sensor.start()
            print("Motor Control")
            emergency = False
            while time.time() < start + duration:
                # if keyboard.is_pressed('space'):
                #     print("stopping")
                #     emergency = True
                #     self.stop_motors()
                #     break
                self.motor_control()
                self.stabilizer_step += 1
                time.sleep(0.005)
            if emergency:
                self.lg_state.stop()
                self.lg_sensor.stop()
                return
            
            self.land()
            self.stop_motors()


            self.lg_state.stop()
            self.lg_sensor.stop()

    def spinup(self):
        print("Spinup")
        spinup_start = time.time()
        d = 3
        while time.time() < spinup_start + d:
            # if keyboard.is_pressed('space'):
            #     self.stop_motors()
            #     break
            self.arm_motors()
            time.sleep(0.03)
    
    def land(self):
        print("Landing")
        self.cf_setpoint.position.z = 0.02
        cooldown_start = time.time()
        cooldown_duration = 20
        while time.time() < cooldown_start + cooldown_duration:
            # if keyboard.is_pressed('space'):
            #     self.stop_motors()
            #     break
            self.motor_control()
            time.sleep(0.005)

    def motor_control(self):
        self.controller.controller_pid(
            self.control,
            self.cf_setpoint,
            self.cf_sensors,
            self.cf_state,
            self.stabilizer_step,
        )
        motor_thrust_uncapped = MotorThrust(motor_1=0, motor_2=0, motor_3=0, motor_4=0)
        self.power_distributor(control=self.control, motor_thrust=motor_thrust_uncapped)
        motor_thrust_battery_comp = MotorThrust(
            motor_1=0, motor_2=0, motor_3=0, motor_4=0
        )
        self.battery_compensator(
            motor_thrust_uncapped=motor_thrust_uncapped,
            motor_thrust_bat_comp=motor_thrust_battery_comp,
        )
        motor_thrust_pwm = MotorThrust(motor_1=0, motor_2=0, motor_3=0, motor_4=0)
        self.power_distribution_cap(
            motor_thrust_bat_comp=motor_thrust_battery_comp,
            motor_thrust_pwm=motor_thrust_pwm,
        )
        self.motorRaw.send_motor_raw(
            int(motor_thrust_pwm.motor_1),
            int(motor_thrust_pwm.motor_2),
            int(motor_thrust_pwm.motor_3),
            int(motor_thrust_pwm.motor_4),
        )

    def power_distributor(self, control, motor_thrust):
        r = control.roll / 2.0
        p = control.pitch / 2.0

        motor_thrust.motor_1 = control.thrust - r + p + control.yaw
        motor_thrust.motor_2 = control.thrust - r - p - control.yaw
        motor_thrust.motor_3 = control.thrust + r - p + control.yaw
        motor_thrust.motor_4 = control.thrust + r + p - control.yaw

    def battery_compensator(self, motor_thrust_uncapped, motor_thrust_bat_comp):
        b = 0.01
        supplyVoltage = 4.2
        supplyVoltage = supplyVoltage + b * (self.cf_vbat - supplyVoltage)
        motor_thrust_bat_comp.motor_1 = self.compensate_voltage(
            i_thrust=motor_thrust_uncapped.motor_1, supply_voltage=supplyVoltage
        )
        motor_thrust_bat_comp.motor_2 = self.compensate_voltage(
            i_thrust=motor_thrust_uncapped.motor_2, supply_voltage=supplyVoltage
        )
        motor_thrust_bat_comp.motor_3 = self.compensate_voltage(
            i_thrust=motor_thrust_uncapped.motor_3, supply_voltage=supplyVoltage
        )
        motor_thrust_bat_comp.motor_4 = self.compensate_voltage(
            i_thrust=motor_thrust_uncapped.motor_4, supply_voltage=supplyVoltage
        )

    def compensate_voltage(self, i_thrust, supply_voltage):
        if supply_voltage < 2.0:
            return 0.0

        thrust = (i_thrust / 65535.0) * THRUST_MAX  # rescaling integer thrust to N
        if thrust < THRUST_MIN:  # Make sure inversion is unique
            return 0.0

        else:
            # Motor voltage to thrust is a cubic fit
            # q, r, p to calculate the inverse of the third order polynomial
            # For more info see https://math.vanderbilt.edu/schectex/courses/cubic/
            # q and thus qrp need to be calculated each time while p and r are constant
            p = -VMOTOR2THRUST2 / (3 * VMOTOR2THRUST3)
            q = p * p * p + (
                VMOTOR2THRUST2 * VMOTOR2THRUST1
                - 3 * VMOTOR2THRUST3 * (VMOTOR2THRUST0 - thrust)
            ) / (6 * VMOTOR2THRUST3 * VMOTOR2THRUST3)
            r = VMOTOR2THRUST1 / (3 * VMOTOR2THRUST3)
            qrp = math.sqrt(q * q + (r - p * p) * (r - p * p) * (r - p * p))

            motorVoltage = np.cbrt(q + qrp) + np.cbrt(q - qrp) + p
            ratio = motorVoltage / supply_voltage
            return UINT16_MAX * ratio

        pass

    def power_distribution_cap(self, motor_thrust_bat_comp, motor_thrust_pwm):
        maxAllowedThrust = UINT16_MAX
        isCapped = False

        thrusts = np.array(
            [
                motor_thrust_bat_comp.motor_1,
                motor_thrust_bat_comp.motor_2,
                motor_thrust_bat_comp.motor_3,
                motor_thrust_bat_comp.motor_4,
            ]
        )
        maxThrust = np.amax(thrusts)

        reduction = 0
        if maxThrust > maxAllowedThrust:
            reduction = maxThrust - maxAllowedThrust
            isCapped = True

        motor_thrust_pwm.motor_1 = max(
            IDLE_THRUST, motor_thrust_bat_comp.motor_1 - reduction
        )
        motor_thrust_pwm.motor_2 = max(
            IDLE_THRUST, motor_thrust_bat_comp.motor_2 - reduction
        )
        motor_thrust_pwm.motor_3 = max(
            IDLE_THRUST, motor_thrust_bat_comp.motor_3 - reduction
        )
        motor_thrust_pwm.motor_4 = max(
            IDLE_THRUST, motor_thrust_bat_comp.motor_4 - reduction
        )

        return isCapped

    def log_state_callback(self, timestamp, data, logconf):
        self.cf_state.position.x = data["stateEstimate.x"]
        self.cf_state.position.y = data["stateEstimate.y"]
        self.cf_state.position.z = data["stateEstimate.z"]
        self.cf_state.velocity.x = data["stateEstimate.vx"]
        self.cf_state.velocity.y = data["stateEstimate.vy"]
        self.cf_state.velocity.z = data["stateEstimate.vz"]
        self.cf_state.acc.x = data["stateEstimate.ax"]
        self.cf_state.acc.y = data["stateEstimate.ay"]
        self.cf_state.acc.z = data["stateEstimate.az"]
        qx = data["stateEstimate.qx"]
        qy = data["stateEstimate.qy"]
        qz = data["stateEstimate.qz"]
        qw = data["stateEstimate.qw"]
        self.cf_state.attitude_quaternion.x = qx
        self.cf_state.attitude_quaternion.y = qy
        self.cf_state.attitude_quaternion.z = qz
        self.cf_state.attitude_quaternion.w = qw
        roll = np.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
        pitch = np.asin(2 * (qw * qy - qx * qz))
        yaw = np.atan2(2 * (qw * qz + qy * qx), 1 - 2 * (qy * qy + qz * qz))
        
        self.cf_state.attitude.roll = np.degrees(roll)
        self.cf_state.attitude.pitch = np.degrees(pitch)
        self.cf_state.attitude.yaw = np.degrees(yaw)

    def log_sensor_callback(self, timestamp, data, logconf):
        self.cf_sensors.gyro.x = data["gyro.x"]
        self.cf_sensors.gyro.y = data["gyro.y"]
        self.cf_sensors.gyro.z = data["gyro.z"]
        self.cf_sensors.acc.x = data["acc.x"]
        self.cf_sensors.acc.y = data["acc.y"]
        self.cf_sensors.acc.z = data["acc.z"]
        self.cf_vbat = data["pm.vbat"]

    def radioLinkStatistics(self, data):
        return
        print("radioLinkStatistics")
        print(data)

    def linkError(self, error):
        print("linkError")
        print(error)


if __name__ == "__main__":
    motorController = PWMMotorController()
    try:
        motorController.init()
    except KeyboardInterrupt:
        print("asdfasdfasdfasdfasdf")
        motorController.stop_motors()
        sys.exit(0)
