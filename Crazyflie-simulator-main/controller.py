import numpy as np
import model.params as params
from math import sin, cos

# Crazyflie-like cascaded PID controller.

CONTROL_DT = 1.0 / 500.0
OUTER_LOOP_DIV = 5  # 500 Hz inner loops, 100 Hz outer loops

# Yaw reference mode
USE_TRAJECTORY_YAW = True
HOLD_YAW = 0.0

# Outer loops
POS_XY_KP, POS_XY_KI, POS_XY_KD = 2.0, 10.0, 0.0
VEL_XY_KP, VEL_XY_KI, VEL_XY_KD = 25.0, 1.0, 0.0
POS_Z_KP, POS_Z_KI, POS_Z_KD = 2.0, 0.5, 0.0
VEL_Z_KP, VEL_Z_KI, VEL_Z_KD = 25.0, 15.0, 0.0

# Inner loops
ATT_RP_KP, ATT_RP_KI, ATT_RP_KD = 6.0, 3.0, 0.0
ATT_YAW_KP, ATT_YAW_KI, ATT_YAW_KD = 6.0, 1.0, 0.35
RATE_RP_KP, RATE_RP_KI, RATE_RP_KD = 200.0, 400.0, 2.5
RATE_YAW_KP, RATE_YAW_KI, RATE_YAW_KD = 120.0, 16.7, 0.0

# Integrator limits from firmware (deg or deg/s channels)
ATT_RP_INT_LIM = 20.0
ATT_YAW_INT_LIM = 360.0
RATE_RP_INT_LIM = 33.3
RATE_YAW_INT_LIM = 166.7

# Simulator-side practical limits (not from firmware)
MAX_TILT = np.deg2rad(50.0)
RATE_RP_SP_MAX = 360.0
RATE_YAW_SP_MAX = 200.0
POS_Z_INT_LIM = 1.5
POS_XY_INT_LIM = 2.0
VEL_XY_INT_LIM = 2.0
VEL_Z_INT_LIM = 2.0
MOTOR_MIX_CMD_MAX = 32767.0

_state = {}


def _clip_sym(x, lim):
    return float(np.clip(x, -lim, lim))


def _wrap_pi(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def _wrap_deg(deg):
    return (deg + 180.0) % 360.0 - 180.0


def set_dt(dt):
    global CONTROL_DT
    CONTROL_DT = dt


def reset():
    _state["tick"] = 0

    # outer-loop integrators
    _state["i_pos_x"] = 0.0
    _state["i_pos_y"] = 0.0
    _state["i_pos_z"] = 0.0
    _state["i_vel_x"] = 0.0
    _state["i_vel_y"] = 0.0
    _state["i_vel_z"] = 0.0

    # inner-loop integrators
    _state["i_att_roll"] = 0.0
    _state["i_att_pitch"] = 0.0
    _state["i_att_yaw"] = 0.0
    _state["i_rate_roll"] = 0.0
    _state["i_rate_pitch"] = 0.0
    _state["i_rate_yaw"] = 0.0

    # derivatives / memory
    _state["prev_vel_x"] = 0.0
    _state["prev_vel_y"] = 0.0
    _state["prev_vel_z"] = 0.0
    _state["prev_p_dps"] = 0.0
    _state["prev_q_dps"] = 0.0
    _state["prev_r_dps"] = 0.0

    # held setpoints from outer loop
    _state["des_phi"] = 0.0
    _state["des_theta"] = 0.0
    _state["thrust_cmd"] = params.mass * params.g


reset()


def run(quad, des_state):
    x, y, z = quad.position()
    x_dot, y_dot, z_dot = quad.velocity()
    phi, theta, psi = quad.attitude()
    p, q, r = quad.omega()

    des_x, des_y, des_z = des_state.pos
    des_x_dot, des_y_dot, des_z_dot = des_state.vel
    des_x_ddot, des_y_ddot, des_z_ddot = des_state.acc

    if USE_TRAJECTORY_YAW:
        des_psi = des_state.yaw
        des_psi_dot = des_state.yawdot
    else:
        des_psi = HOLD_YAW
        des_psi_dot = 0.0

    dt = CONTROL_DT
    _state["tick"] += 1
    update_outer = (_state["tick"] == 1) or (_state["tick"] % OUTER_LOOP_DIV == 0)

    # ----------------------
    # Outer loops (100 Hz)
    # ----------------------
    if update_outer:
        # position -> velocity setpoint
        ex, ey, ez = des_x - x, des_y - y, des_z - z
        _state["i_pos_x"] = _clip_sym(_state["i_pos_x"] + ex * dt, POS_XY_INT_LIM)
        _state["i_pos_y"] = _clip_sym(_state["i_pos_y"] + ey * dt, POS_XY_INT_LIM)
        _state["i_pos_z"] = _clip_sym(_state["i_pos_z"] + ez * dt, POS_Z_INT_LIM)
        vx_sp = des_x_dot + POS_XY_KP * ex + POS_XY_KI * _state["i_pos_x"] + POS_XY_KD * 0.0
        vy_sp = des_y_dot + POS_XY_KP * ey + POS_XY_KI * _state["i_pos_y"] + POS_XY_KD * 0.0
        vz_sp = des_z_dot + POS_Z_KP * ez + POS_Z_KI * _state["i_pos_z"] + POS_Z_KD * 0.0

        # velocity -> acceleration setpoint
        evx, evy, evz = vx_sp - x_dot, vy_sp - y_dot, vz_sp - z_dot
        _state["i_vel_x"] = _clip_sym(_state["i_vel_x"] + evx * dt, VEL_XY_INT_LIM)
        _state["i_vel_y"] = _clip_sym(_state["i_vel_y"] + evy * dt, VEL_XY_INT_LIM)
        _state["i_vel_z"] = _clip_sym(_state["i_vel_z"] + evz * dt, VEL_Z_INT_LIM)

        ax_cmd = des_x_ddot + VEL_XY_KP * evx + VEL_XY_KI * _state["i_vel_x"] + VEL_XY_KD * 0.0
        ay_cmd = des_y_ddot + VEL_XY_KP * evy + VEL_XY_KI * _state["i_vel_y"] + VEL_XY_KD * 0.0
        az_cmd = des_z_ddot + VEL_Z_KP * evz + VEL_Z_KI * _state["i_vel_z"] + VEL_Z_KD * 0.0

        # acceleration -> attitude + thrust
        des_phi = (1.0 / params.g) * (ax_cmd * sin(des_psi) - ay_cmd * cos(des_psi))
        des_theta = (1.0 / params.g) * (ax_cmd * cos(des_psi) + ay_cmd * sin(des_psi))
        _state["des_phi"] = float(np.clip(des_phi, -MAX_TILT, MAX_TILT))
        _state["des_theta"] = float(np.clip(des_theta, -MAX_TILT, MAX_TILT))

        max_upward_acc = params.maxF / params.mass - params.g
        az_cmd = float(np.clip(az_cmd, -0.95 * params.g, max_upward_acc))
        _state["thrust_cmd"] = float(np.clip(params.mass * (params.g + az_cmd), params.minF, params.maxF))

    F = _state["thrust_cmd"]
    des_phi = _state["des_phi"]
    des_theta = _state["des_theta"]

    # ----------------------
    # Attitude loop (500 Hz)
    # ----------------------
    phi_d = np.degrees(phi)
    theta_d = np.degrees(theta)
    psi_d = np.degrees(psi)
    des_phi_d = np.degrees(des_phi)
    des_theta_d = np.degrees(des_theta)
    des_psi_d = np.degrees(des_psi)
    des_psi_dot_dps = np.degrees(des_psi_dot)

    e_roll = _wrap_deg(des_phi_d - phi_d)
    e_pitch = _wrap_deg(des_theta_d - theta_d)
    e_yaw = _wrap_deg(des_psi_d - psi_d)

    _state["i_att_roll"] = _clip_sym(_state["i_att_roll"] + e_roll * dt, ATT_RP_INT_LIM)
    _state["i_att_pitch"] = _clip_sym(_state["i_att_pitch"] + e_pitch * dt, ATT_RP_INT_LIM)
    _state["i_att_yaw"] = _clip_sym(_state["i_att_yaw"] + e_yaw * dt, ATT_YAW_INT_LIM)

    # derivative on measurement (yaw D only in table)
    phi_dot_dps = np.degrees(p)
    theta_dot_dps = np.degrees(q)
    psi_dot_dps = np.degrees(r)

    p_sp = ATT_RP_KP * e_roll + ATT_RP_KI * _state["i_att_roll"] - ATT_RP_KD * phi_dot_dps
    q_sp = ATT_RP_KP * e_pitch + ATT_RP_KI * _state["i_att_pitch"] - ATT_RP_KD * theta_dot_dps
    r_sp = des_psi_dot_dps + ATT_YAW_KP * e_yaw + ATT_YAW_KI * _state["i_att_yaw"] - ATT_YAW_KD * psi_dot_dps
    p_sp = _clip_sym(p_sp, RATE_RP_SP_MAX)
    q_sp = _clip_sym(q_sp, RATE_RP_SP_MAX)
    r_sp = _clip_sym(r_sp, RATE_YAW_SP_MAX)

    # ----------------------
    # Rate loop (500 Hz)
    # ----------------------
    p_dps, q_dps, r_dps = np.degrees([p, q, r])
    ep, eq, er = p_sp - p_dps, q_sp - q_dps, r_sp - r_dps

    _state["i_rate_roll"] = _clip_sym(_state["i_rate_roll"] + ep * dt, RATE_RP_INT_LIM)
    _state["i_rate_pitch"] = _clip_sym(_state["i_rate_pitch"] + eq * dt, RATE_RP_INT_LIM)
    _state["i_rate_yaw"] = _clip_sym(_state["i_rate_yaw"] + er * dt, RATE_YAW_INT_LIM)

    p_dot_dps2 = (p_dps - _state["prev_p_dps"]) / dt
    q_dot_dps2 = (q_dps - _state["prev_q_dps"]) / dt
    r_dot_dps2 = (r_dps - _state["prev_r_dps"]) / dt
    _state["prev_p_dps"] = p_dps
    _state["prev_q_dps"] = q_dps
    _state["prev_r_dps"] = r_dps

    u_roll = RATE_RP_KP * ep + RATE_RP_KI * _state["i_rate_roll"] - RATE_RP_KD * p_dot_dps2
    u_pitch = RATE_RP_KP * eq + RATE_RP_KI * _state["i_rate_pitch"] - RATE_RP_KD * q_dot_dps2
    u_yaw = RATE_YAW_KP * er + RATE_YAW_KI * _state["i_rate_yaw"] - RATE_YAW_KD * r_dot_dps2

    # Map firmware-like rate outputs to realizable body moments.
    u_roll = _clip_sym(u_roll, MOTOR_MIX_CMD_MAX)
    u_pitch = _clip_sym(u_pitch, MOTOR_MIX_CMD_MAX)
    u_yaw = _clip_sym(u_yaw, MOTOR_MIX_CMD_MAX)

    max_mx = 2.0 * params.mixer_arm * params.maxF_per_motor
    max_my = max_mx
    max_mz = 2.0 * params.thrust_to_torque * params.maxF_per_motor
    M = np.array([
        max_mx * (u_roll / MOTOR_MIX_CMD_MAX),
        max_my * (u_pitch / MOTOR_MIX_CMD_MAX),
        max_mz * (u_yaw / MOTOR_MIX_CMD_MAX)
    ]).reshape(3, 1)

    return F, M
