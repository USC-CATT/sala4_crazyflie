import time
import math
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
import cflib.crtp
URI = 'radio://0/80/2M/E7E7E7E7E7'

def fly_straight(mc, vx, vy, duration):
    dt = 1.0 / 50  # 20 ms
    steps = int(duration * 50)

    for _ in range(steps):
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

if __name__ == "__main__":
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        with MotionCommander(scf, default_height=0.17) as mc:
            time.sleep(2.0)
            trajectory(mc)
            time.sleep(1.0)
            mc.stop()