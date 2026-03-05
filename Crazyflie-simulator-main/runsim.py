
from quadPlot import plot_quad_3d
import controller
import augmented_controller
import trajGen
import trajGen3D
import disturbance
import nominal_model
from model.quadcopter import Quadcopter
import numpy as np

animation_frequency = 50
control_frequency = 500 # Hz for attitude control loop 
control_iterations = control_frequency // animation_frequency
dt = 1.0 / control_frequency
time = [0.0]
trajectory_speed = 0.3 # m/s along the trajecotry 

# print nominal linearized model around hover before simulation.
PRINT_NOMINAL_LINEAR_MODEL = False
LINEAR_MODEL_STATE_EPS = 1e-6
LINEAR_MODEL_INPUT_EPS = 1e-6

# Controller mode:
# "baseline"  -> cascaded PID only (controller.py)
# "augmented" -> cascaded PID + LS virtual input (augmented_controller.py)
controller_mode = "baseline"

# Simulation mode:
# "trajectory" -> follow waypoints
# "hover_wind" -> hold a hover setpoint and inject sinusoidal lateral wind
sim_mode = "hover_wind"

# Initial state
INITIAL_POS = (0.0, 0.0, 0.0)
INITIAL_ATTITUDE = (0.0, 0.0, 0.0)

# Hover + wind test settings (used when sim_mode == "hover_wind")
HOVER_SETPOINT = np.array([0.0, 0.0, 1], dtype=float)  # [x, y, z] in meters
HOVER_YAW = 0.0  # rad
HOVER_WIND_SIM_DURATION = 50.0  # seconds
# Wind parameters are configured in disturbance.py

# Waypoint source:
# "helix" -> generated from HELIX_* settings
# "list"  -> uses USER_WAYPOINTS below
# "csv"   -> loads WAYPOINTS_CSV_PATH (each row: x,y,z[,yaw])
waypoint_source = "list"

HELIX_START_T = 0.0
HELIX_NUM_WAYPOINTS = 9

# Example custom waypoints (meters). Used when waypoint_source = "list".
# Waypoints requirements:
# - Must be an Nx3 or Nx4 array/list with columns [x, y, z] or [x, y, z, yaw].
# - Yaw unit is radians. If omitted, yaw defaults to 0.0.
# - Must have at least 2 waypoints.
# - Consecutive waypoints must not be identical or too close together.
# USER_WAYPOINTS = [
#     [0,0.2,0.3,0.5233],
#     [-0.2, 0.3, 0.5, 0.5233*2],
#     [-0.3, 0.5, 1.0, 0.5233*3],
#     [-0.5, 0.5, 1.5, 0.5233*2],
# ]

USER_WAYPOINTS = [
    [0,0,1],
]

# Used when waypoint_source = "csv".
WAYPOINTS_CSV_PATH = "waypoints.csv"


def build_waypoints():
    if waypoint_source == "helix":
        waypoints = trajGen3D.get_helix_waypoints(HELIX_START_T, HELIX_NUM_WAYPOINTS)
    elif waypoint_source == "list":
        try:
            waypoints = np.asarray(USER_WAYPOINTS, dtype=float)
        except ValueError as exc:
            row_lengths = []
            for row in USER_WAYPOINTS:
                try:
                    row_lengths.append(len(row))
                except TypeError:
                    row_lengths.append(-1)
            raise ValueError(
                "USER_WAYPOINTS rows must all have the same length: Nx3 [x,y,z] "
                "or Nx4 [x,y,z,yaw]. Found row lengths {}.".format(row_lengths)
            ) from exc
        if waypoints.ndim != 2 or waypoints.shape[1] not in (3, 4):
            raise ValueError("USER_WAYPOINTS must be an Nx3 or Nx4 array/list.")

        initial_pos = np.asarray(INITIAL_POS, dtype=float).reshape(3,)
        if waypoints.shape[1] == 3:
            initial_wp = initial_pos
            first_wp_pos = waypoints[0, :3]
        else:
            initial_wp = np.array([initial_pos[0], initial_pos[1], initial_pos[2], 0.0], dtype=float)
            first_wp_pos = waypoints[0, :3]

        if np.linalg.norm(first_wp_pos - initial_pos) > 1e-6:
            waypoints = np.vstack((initial_wp, waypoints))
    elif waypoint_source == "csv":
        waypoints = np.atleast_2d(np.loadtxt(WAYPOINTS_CSV_PATH, delimiter=",", dtype=float))
    else:
        raise ValueError('waypoint_source must be one of: "helix", "list", "csv"')

    waypoints = trajGen3D.format_waypoints(waypoints)

    segment_lengths = np.linalg.norm(np.diff(waypoints[:, :3], axis=0), axis=1)
    if np.any(segment_lengths < 1e-6):
        raise ValueError("Consecutive waypoints are identical or too close together.")

    return waypoints


def get_external_force_world(t):
    if sim_mode == "hover_wind":
        return disturbance.force_world(t)
    return np.zeros(3)


def wind_formula_text():
    if sim_mode != "hover_wind":
        return ""
    return disturbance.formula_text()


def get_desired_state(t, waypoints, coeff_x, coeff_y, coeff_z):
    if sim_mode == "hover_wind":
        return trajGen3D.DesiredState(
            pos=HOVER_SETPOINT.copy(),
            vel=np.zeros(3),
            acc=np.zeros(3),
            yaw=float(HOVER_YAW),
            yawdot=0.0
        )
    return trajGen3D.generate_trajectory(t, trajectory_speed, waypoints, coeff_x, coeff_y, coeff_z)

def attitudeControl(quad, time, waypoints, coeff_x, coeff_y, coeff_z, control_backend):
    desired_state = get_desired_state(time[0], waypoints, coeff_x, coeff_y, coeff_z)
    if control_backend is None:
        F, M = controller.run(quad, desired_state)
    else:
        F, M = control_backend.run(quad, desired_state, sim_time=time[0])
    quad.update(dt, F, M, external_force_world=get_external_force_world(time[0]))
    time[0] += dt

def main():
    # Initial conditions
    pos = INITIAL_POS
    attitude = INITIAL_ATTITUDE
    
    quadcopter = Quadcopter(pos, attitude)
    if controller_mode == "baseline":
        control_backend = None
        controller.set_dt(dt)
        controller.reset()
    elif controller_mode == "augmented":
        control_backend = augmented_controller.AugmentedController(dt=dt)
        control_backend.set_dt(dt)
        control_backend.reset()
    else:
        raise ValueError('controller_mode must be one of: "baseline", "augmented"')

    if PRINT_NOMINAL_LINEAR_MODEL:
        A, B, x_eq, u_eq = nominal_model.linearize_hover(
            pos=INITIAL_POS,
            attitude=INITIAL_ATTITUDE,
            state_eps=LINEAR_MODEL_STATE_EPS,
            input_eps=LINEAR_MODEL_INPUT_EPS,
        )
        np.set_printoptions(precision=5, suppress=True)
        print("Nominal linear model around hover")
        print("delta_x_dot = A*delta_x + B*delta_u")
        print("State order:", nominal_model.STATE_ORDER)
        print("Input order:", nominal_model.INPUT_ORDER)
        print("x_eq =", x_eq)
        print("u_eq =", u_eq)
        print("A =\n", A)
        print("B =\n", B)

    if sim_mode == "trajectory":
        waypoints = build_waypoints()
        (coeff_x, coeff_y, coeff_z) = trajGen3D.get_MST_coefficients(waypoints)
        segment_lengths = np.linalg.norm(np.diff(waypoints[:, :3], axis=0), axis=1)
        trajectory_duration = float(np.sum(segment_lengths) / trajectory_speed)
        sim_duration = trajectory_duration + 2.0
        plot_waypoints = waypoints[:, :3]
        wind_formula = ""
    elif sim_mode == "hover_wind":
        # Use a repeated point for plotting target marker.
        waypoints = np.vstack((HOVER_SETPOINT, HOVER_SETPOINT))
        coeff_x, coeff_y, coeff_z = None, None, None
        sim_duration = HOVER_WIND_SIM_DURATION
        plot_waypoints = waypoints
        wind_formula = wind_formula_text()
    else:
        raise ValueError('sim_mode must be one of: "trajectory", "hover_wind"')

    num_frames = int(np.ceil(sim_duration * animation_frequency))
    max_position_error = [float("nan")]

    def control_loop(i):
        for _ in range(control_iterations):
            attitudeControl(quadcopter, time, waypoints, coeff_x, coeff_y, coeff_z, control_backend)
        current_t = float(time[0])
        desired_now = get_desired_state(current_t, waypoints, coeff_x, coeff_y, coeff_z)
        pos_error_norm = float(np.linalg.norm(quadcopter.position() - desired_now.pos))
        pos_error_norm_for_max = pos_error_norm
        waiting_for_wind = bool(sim_mode == "hover_wind" and current_t < disturbance.START_TIME)
        should_track_max = (not waiting_for_wind)
        if should_track_max and np.isfinite(pos_error_norm):
            if not np.isfinite(max_position_error[0]):
                max_position_error[0] = pos_error_norm
            else:
                max_position_error[0] = max(max_position_error[0], pos_error_norm)
        return {
            "frame": quadcopter.world_frame(),
            "time": current_t,
            "wind_force_world": get_external_force_world(current_t),
            "wind_formula": wind_formula,
            "position_error_norm": pos_error_norm,
            "position_error_norm_for_max": pos_error_norm_for_max,
            "position_error_max": float(max_position_error[0]),
            "waiting_for_wind": waiting_for_wind,
            "controller_mode": controller_mode,
            "aug_mu": float(control_backend.mu) if control_backend is not None else float("nan"),
        }

    plot_quad_3d(plot_waypoints, control_loop, frames=num_frames)

if __name__ == "__main__":
    main()
