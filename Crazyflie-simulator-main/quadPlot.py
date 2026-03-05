from mpl_toolkits.mplot3d import Axes3D
from matplotlib.colors import cnames
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import sys
from utils.utils import RotToRPY

history = np.zeros((500,3))
time_history = np.zeros(500)
wind_force_history = np.zeros((500, 3))

count = 0
force_count = 0
status_text = None
controller_status_text = None
position_error_max = np.nan

ax_3d = None
ax_force = None
body_lines = []
waypoint_line = None
history_line = None
force_line_x = None
force_line_y = None
force_line_z = None

def _expand_3d_limits_to_include(point, margin=0.05):
    """Only expand current 3D limits so the provided point stays in view."""
    if ax_3d is None:
        return

    x, y, z = np.asarray(point, dtype=float).reshape(3,)

    x0, x1 = ax_3d.get_xlim3d()
    y0, y1 = ax_3d.get_ylim3d()
    z0, z1 = ax_3d.get_zlim3d()

    changed = False
    if x < x0:
        x0 = x - margin
        changed = True
    elif x > x1:
        x1 = x + margin
        changed = True

    if y < y0:
        y0 = y - margin
        changed = True
    elif y > y1:
        y1 = y + margin
        changed = True

    if z < z0:
        z0 = z - margin
        changed = True
    elif z > z1:
        z1 = z + margin
        changed = True

    if changed:
        ax_3d.set_xlim(x0, x1)
        ax_3d.set_ylim(y0, y1)
        ax_3d.set_zlim(z0, z1)

def _save_animation(anim):
    """Save animation from CLI: python runsim.py save [mp4|gif]."""
    save_format = "mp4"
    if len(sys.argv) > 2:
        save_format = str(sys.argv[2]).strip().lower()

    if save_format not in ("mp4", "gif"):
        print("Unknown save format '{}', defaulting to mp4.".format(save_format))
        save_format = "mp4"

    fps = 30
    dpi = 80

    if save_format == "mp4":
        if animation.writers.is_available("ffmpeg"):
            print("saving sim.mp4")
            writer = animation.FFMpegWriter(fps=fps, codec="libx264")
            anim.save("sim.mp4", dpi=dpi, writer=writer)
            print("saved sim.mp4")
            return
        print("MovieWriter ffmpeg unavailable; falling back to GIF.")
        save_format = "gif"

    if save_format == "gif":
        if animation.writers.is_available("imagemagick"):
            print("saving sim.gif")
            anim.save("sim.gif", dpi=dpi, writer="imagemagick", fps=fps)
            print("saved sim.gif")
            return
        if animation.writers.is_available("pillow"):
            print("saving sim.gif (pillow writer)")
            writer = animation.PillowWriter(fps=fps)
            anim.save("sim.gif", dpi=dpi, writer=writer)
            print("saved sim.gif")
            return
        raise RuntimeError("No GIF writer available. Install ffmpeg, imagemagick, or pillow.")

def plot_quad_3d(waypoints, get_world_frame, frames=400):
    """
    get_world_frame is a function which return the "next" world frame to be drawn
    """
    global history, time_history, wind_force_history
    global count, force_count, status_text, controller_status_text, position_error_max
    global ax_3d, ax_force, body_lines, waypoint_line, history_line
    global force_line_x, force_line_y, force_line_z

    n_hist = max(2, int(frames) + 1)
    history = np.zeros((n_hist, 3))
    time_history = np.zeros(n_hist)
    wind_force_history = np.zeros((n_hist, 3))
    count = 0
    force_count = 0
    status_text = None
    controller_status_text = None
    position_error_max = np.nan

    fig = plt.figure(figsize=(12, 6))
    grid = fig.add_gridspec(1, 2, width_ratios=[3.1, 1.4])
    ax_3d = fig.add_subplot(grid[0, 0], projection='3d')
    ax_force = fig.add_subplot(grid[0, 1])

    ax_3d.set_xlabel("X (m)")
    ax_3d.set_ylabel("Y (m)")
    ax_3d.set_zlabel("Z (m)")

    body_lines = [
        ax_3d.plot([], [], [], '-', c='cyan')[0],
        ax_3d.plot([], [], [], '-', c='red')[0],
        ax_3d.plot([], [], [], '-', c='blue', marker='o', markevery=2)[0],
    ]
    waypoint_line = ax_3d.plot([], [], [], '.', c='red', markersize=4)[0]
    history_line = ax_3d.plot([], [], [], '.', c='blue', markersize=2)[0]

    controller_status_text = ax_3d.text2D(
        0.02,
        0.99,
        "",
        transform=ax_3d.transAxes,
        va='top',
        fontweight='bold',
        fontsize=10,
        color='black',
        bbox=dict(facecolor='white', alpha=0.6, edgecolor='none')
    )
    status_text = ax_3d.text2D(
        0.02,
        0.94,
        "",
        transform=ax_3d.transAxes,
        va='top',
        fontsize=10,
        bbox=dict(facecolor='white', alpha=0.6, edgecolor='none')
    )

    force_line_x = ax_force.plot([], [], '-', lw=1.6, color='tab:orange', label='Fx')[0]
    force_line_y = ax_force.plot([], [], '-', lw=1.6, color='tab:green', label='Fy')[0]
    force_line_z = ax_force.plot([], [], '-', lw=1.6, color='tab:purple', label='Fz')[0]
    ax_force.set_title("Wind Force vs Time")
    ax_force.set_xlabel("Time (s)")
    ax_force.set_ylabel("Force (N)")
    ax_force.grid(True, alpha=0.3)
    ax_force.legend(loc='upper right', fontsize=8)
    ax_force.set_xlim(0.0, 10.0)
    ax_force.set_ylim(-0.05, 0.05)

    set_limit((-0.5,0.5), (-0.5,0.5), (0,3), ax=ax_3d)
    plot_waypoints(waypoints)
    an = animation.FuncAnimation(fig,
                                 anim_callback,
                                 fargs=(get_world_frame,),
                                 init_func=None,
                                 frames=frames, interval=10, blit=False)

    if len(sys.argv) > 1 and sys.argv[1] == 'save':
        _save_animation(an)
    else:
        plt.show()

def plot_waypoints(waypoints):
    global waypoint_line
    if waypoint_line is None:
        return
    waypoints = np.asarray(waypoints, dtype=float)
    if waypoints.ndim != 2 or waypoints.shape[0] == 0:
        return
    if waypoints.shape[1] > 3:
        waypoints = waypoints[:, :3]
    waypoint_line.set_data(waypoints[:,0], waypoints[:,1])
    waypoint_line.set_3d_properties(waypoints[:,2])

def set_limit(x, y, z, ax=None):
    if ax is None:
        ax = ax_3d
    if ax is None:
        return
    ax.set_xlim(x)
    ax.set_ylim(y)
    ax.set_zlim(z)

def anim_callback(i, get_world_frame):
    payload = get_world_frame(i)
    set_frame(payload)

def _parse_payload(payload):
    if isinstance(payload, dict):
        frame = np.asarray(payload.get("frame"))
        sim_time = float(payload.get("time", np.nan))
        wind_force = np.asarray(payload.get("wind_force_world", np.zeros(3)), dtype=float).reshape(3,)
        position_error_norm = float(payload.get("position_error_norm", np.nan))
        position_error_norm_for_max = float(payload.get("position_error_norm_for_max", position_error_norm))
        position_error_max_payload = float(payload.get("position_error_max", np.nan))
        waiting_for_wind = bool(payload.get("waiting_for_wind", False))
        controller_mode = str(payload.get("controller_mode", "n/a"))
        aug_mu = float(payload.get("aug_mu", np.nan))
        return frame, sim_time, wind_force, position_error_norm, position_error_norm_for_max, position_error_max_payload, waiting_for_wind, controller_mode, aug_mu
    frame = np.asarray(payload)
    return frame, np.nan, np.zeros(3), np.nan, np.nan, np.nan, False, "n/a", np.nan

def _update_force_plot(sim_time, wind_force):
    global force_count, time_history, wind_force_history
    if not np.isfinite(sim_time):
        return

    time_history[force_count] = sim_time
    wind_force_history[force_count] = wind_force
    if force_count < np.size(time_history, 0) - 1:
        force_count += 1

    if force_count < 2:
        return

    t = time_history[:force_count]
    f = wind_force_history[:force_count]

    force_line_x.set_data(t, f[:,0])
    force_line_y.set_data(t, f[:,1])
    force_line_z.set_data(t, f[:,2])

    t_end = t[-1]
    t_start = max(0.0, t_end - 10.0)
    ax_force.set_xlim(t_start, max(10.0, t_end))

    max_abs = np.max(np.abs(f))
    y_lim = max(0.03, 1.2 * max_abs)
    ax_force.set_ylim(-y_lim, y_lim)

def set_frame(payload):
    global status_text, controller_status_text, position_error_max
    frame, sim_time, wind_force, position_error_norm, position_error_norm_for_max, position_error_max_payload, waiting_for_wind, controller_mode, aug_mu = _parse_payload(payload)

    if frame.shape != (3, 6):
        raise ValueError("Frame must be a 3x6 matrix.")

    # convert 3x6 world_frame matrix into three line_data objects which is 3x2 (row:point index, column:x,y,z)
    lines_data = [frame[:,[0,2]], frame[:,[1,3]], frame[:,[4,5]]]
    for line, line_data in zip(body_lines, lines_data):
        x, y, z = line_data
        line.set_data(x, y)
        line.set_3d_properties(z)

    global history, count
    # plot history trajectory
    hist_len = np.size(history, 0)
    if count < hist_len:
        history[count] = frame[:,4]
        count += 1
    else:
        # Keep a rolling trail once the history buffer is full.
        history[:-1] = history[1:]
        history[-1] = frame[:,4]
    valid_count = min(count, hist_len)
    zline = history[:valid_count,-1]
    xline = history[:valid_count,0]
    yline = history[:valid_count,1]
    history_line.set_data(xline, yline)
    history_line.set_3d_properties(zline)
    # ax.plot3D(xline, yline, zline, 'blue')

    # Current position and attitude display.
    origin = frame[:,4]
    _expand_3d_limits_to_include(origin)
    x_axis = 0.5 * (frame[:,1] + frame[:,2]) - 0.5 * (frame[:,0] + frame[:,3])
    y_axis = 0.5 * (frame[:,2] + frame[:,3]) - 0.5 * (frame[:,0] + frame[:,1])
    x_norm = np.linalg.norm(x_axis)
    y_norm = np.linalg.norm(y_axis)
    if x_norm > 1e-9 and y_norm > 1e-9:
        x_hat = x_axis / x_norm
        y_hat = y_axis / y_norm
        z_hat = np.cross(x_hat, y_hat)
        z_norm = np.linalg.norm(z_hat)
        if z_norm > 1e-9:
            z_hat = z_hat / z_norm
            rot = np.column_stack((x_hat, y_hat, z_hat))
            roll, pitch, yaw = RotToRPY(rot)
            roll_deg, pitch_deg, yaw_deg = np.degrees([roll, pitch, yaw])
        else:
            roll_deg, pitch_deg, yaw_deg = 0.0, 0.0, 0.0
    else:
        roll_deg, pitch_deg, yaw_deg = 0.0, 0.0, 0.0

    _update_force_plot(sim_time, wind_force)

    wind_norm = np.linalg.norm(wind_force)
    time_str = "n/a" if not np.isfinite(sim_time) else "{:.2f}".format(sim_time)
    pos_error_text = ""
    if np.isfinite(position_error_max_payload):
        position_error_max = position_error_max_payload
    elif np.isfinite(position_error_norm_for_max):
        if not np.isfinite(position_error_max):
            position_error_max = position_error_norm_for_max
        else:
            position_error_max = max(position_error_max, position_error_norm_for_max)
    if np.isfinite(position_error_norm):
        if waiting_for_wind:
            pos_error_text = "\n|position error|: {:.3f} m    max: waiting for wind".format(
                position_error_norm
            )
        elif np.isfinite(position_error_max):
            pos_error_text = "\n|position error|: {:.3f} m    max: {:.3f} m".format(
                position_error_norm, position_error_max
            )
        else:
            pos_error_text = "\n|position error|: {:.3f} m    max: n/a".format(
                position_error_norm
            )
    if controller_status_text is not None:
        if controller_mode == "augmented":
            mu_text = "n/a" if (not np.isfinite(aug_mu) or abs(aug_mu) < 1e-12) else "{:.3f}".format(aug_mu)
            controller_status_text.set_text(
                "Controller: augmented | mu: {}".format(mu_text)
            )
        else:
            controller_status_text.set_text("Controller: {}".format(controller_mode))

    if status_text is not None:
        status_text.set_text(
            "t: {} s\nPos: x={:.2f}, y={:.2f}, z={:.2f}\nRoll: {:.1f} deg  Pitch: {:.1f} deg  Yaw: {:.1f} deg\nWind [N]: Fx={:.3f}, Fy={:.3f}, Fz={:.3f}, |F|={:.3f}{}".format(
                time_str, origin[0], origin[1], origin[2], roll_deg, pitch_deg, yaw_deg,
                wind_force[0], wind_force[1], wind_force[2], wind_norm, pos_error_text
            )
        )
