import argparse
import json
import math
import os
import sys

import matplotlib.pyplot as plt
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

SMOOTHING_WINDOW = 100
DEFAULT_LOOP_HZ = 200.0


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plotter for Crazyflie telemetry data."
    )
    parser.add_argument(
        "--file",
        default="waypoint_data.json",
        help="Data file to plot",
    )
    parser.add_argument(
        "--augmented_file",
        default=None,
        help="Augmented data file to plot",
    )
    parser.add_argument(
        "--pid_file",
        default=None,
        help="Pid data file to plot",
    )
    parser.add_argument(
        "--cutoff", default=math.inf, help="Cutoff time of data", type=float
    )
    parser.add_argument(
        "--start", default=0, help="Start cutoff time of data", type=float
    )
    parser.add_argument(
        "--offset", default=0, help="Time offset between two plots", type=float
    )
    return parser.parse_args()


# apply a moving average filter to the data for smoother plots
def _smooth(values, window):
    if window <= 1:
        return np.asarray(values, dtype=float)
    kernel = np.ones((window,), dtype=float) / float(window)

    return np.convolve(values, kernel, mode="same")


def _time_axis(data, sample_count):
    if "time_s" in data and len(data["time_s"]) == sample_count:
        return np.asarray(data["time_s"], dtype=float)

    loop_hz = float(data.get("loop_hz", DEFAULT_LOOP_HZ))
    if loop_hz <= 0.0:
        loop_hz = DEFAULT_LOOP_HZ
    return np.arange(sample_count, dtype=float) / loop_hz


def main():
    args = parse_args()
    dual_graph = False
    data = augmented_data = pid_data = {}
    if args.pid_file is not None and args.augmented_file is not None:
        AUGMENTED_DATA_PATH = os.path.join(SCRIPT_DIR, args.augmented_file)
        PID_DATA_PATH = os.path.join(SCRIPT_DIR, args.pid_file)
        dual_graph = True
        with open(AUGMENTED_DATA_PATH, "r", encoding="utf-8") as file:
            augmented_data = json.load(file)
        with open(PID_DATA_PATH, "r", encoding="utf-8") as file:
            pid_data = json.load(file)
    else:
        WAYPOINT_DATA_PATH = os.path.join(SCRIPT_DIR, args.file)
        with open(WAYPOINT_DATA_PATH, "r", encoding="utf-8") as file:
            data = json.load(file)
    t = []
    pid_cutoff_start = 0
    augmented_cutoff_start = 0
    if dual_graph:
        if args.offset < 0:
            t = _time_axis(pid_data, len(pid_data["position_x"]))
        else:
            t = _time_axis(augmented_data, len(augmented_data["position_x"]))
    else:
        t = _time_axis(data, len(data["position_x"]))
    if dual_graph:
        first_index = next((i for i, x in enumerate(t) if x > abs(args.offset)))
        print(first_index)
        t = t[first_index:]
        t = np.array(t)
        t = t - args.offset
        if args.offset < 0:
            pid_cutoff_start = first_index
        else:
            augmented_cutoff_start = first_index
    start_index = next((i for i, x in enumerate(t) if x > abs(args.start)))
    print(start_index)
    cutoff_time = t[t <= args.cutoff]
    cutoff_length = len(cutoff_time)
    print(cutoff_length)
    if dual_graph:
        cutoff_length = min(
            cutoff_length,
            len(augmented_data["position_x"]),
            len(pid_data["position_x"]),
        )
    cutoff_time = cutoff_time[
        (max(pid_cutoff_start, augmented_cutoff_start) + start_index) : cutoff_length
    ]
    print(len(cutoff_time), cutoff_length)
    x = y = z = x_sp = y_sp = z_sp = x2 = y2 = z2 = []
    if dual_graph:
        x = _smooth(pid_data["position_x"], SMOOTHING_WINDOW)[
            (pid_cutoff_start + start_index) : cutoff_length - augmented_cutoff_start
        ]
        y = _smooth(pid_data["position_y"], SMOOTHING_WINDOW)[
            (pid_cutoff_start + start_index) : cutoff_length - augmented_cutoff_start
        ]
        z = _smooth(pid_data["position_z"], SMOOTHING_WINDOW)[
            (pid_cutoff_start + start_index) : cutoff_length - augmented_cutoff_start
        ]
        x2 = _smooth(augmented_data["position_x"], SMOOTHING_WINDOW)[
            (augmented_cutoff_start + start_index) : cutoff_length - pid_cutoff_start
        ]
        y2 = _smooth(augmented_data["position_y"], SMOOTHING_WINDOW)[
            (augmented_cutoff_start + start_index) : cutoff_length - pid_cutoff_start
        ]
        z2 = _smooth(augmented_data["position_z"], SMOOTHING_WINDOW)[
            (augmented_cutoff_start + start_index) : cutoff_length - pid_cutoff_start
        ]
        x_sp = pid_data["setpoint_x"][
            (
                max(pid_cutoff_start, augmented_cutoff_start) + start_index
            ) : cutoff_length
        ]
        y_sp = pid_data["setpoint_y"][
            (
                max(pid_cutoff_start, augmented_cutoff_start) + start_index
            ) : cutoff_length
        ]
        z_sp = pid_data["setpoint_z"][
            (
                max(pid_cutoff_start, augmented_cutoff_start) + start_index
            ) : cutoff_length
        ]
        pass
    else:
        x = _smooth(data["position_x"], SMOOTHING_WINDOW)[start_index:cutoff_length]
        y = _smooth(data["position_y"], SMOOTHING_WINDOW)[start_index:cutoff_length]
        z = _smooth(data["position_z"], SMOOTHING_WINDOW)[start_index:cutoff_length]
        x_sp = data["setpoint_x"][start_index:cutoff_length]
        y_sp = data["setpoint_y"][start_index:cutoff_length]
        z_sp = data["setpoint_z"][start_index:cutoff_length]

    fig = plt.figure(figsize=(14, 10))

    try:
        ax_3d = fig.add_subplot(2, 2, 1, projection="3d")
        ax_3d.plot(x, y, z, label="pid" if dual_graph else "state", color="tab:blue")
        if dual_graph:
            ax_3d.plot(x2, y2, z2, label="augmented", color="tab:red")

        ax_3d.plot(x_sp, y_sp, z_sp, label="setpoint", color="tab:orange", marker="x")
        ax_3d.set_title("3D Trajectory")
        ax_3d.set_xlabel("X [m]")
        ax_3d.set_ylabel("Y [m]")
        ax_3d.set_zlabel("Z [m]")
        ax_3d.legend()
    except Exception:
        ax_xy = fig.add_subplot(2, 2, 1)
        ax_xy.plot(x, y, label="pid" if dual_graph else "state", color="tab:blue")
        if dual_graph:
            ax_xy.plot(x2, y2, label="augmented", color="tab:red")
        ax_xy.plot(x_sp, y_sp, label="setpoint", color="tab:orange")
        ax_xy.set_title("XY Trajectory")
        ax_xy.set_xlabel("X [m]")
        ax_xy.set_ylabel("Y [m]")
        ax_xy.axis("equal")
        ax_xy.grid(True)
        ax_xy.legend()

    ax_x = fig.add_subplot(2, 2, 2)
    ax_x.plot(cutoff_time, x, label="x pid" if dual_graph else "x", color="tab:blue")
    if dual_graph:
        ax_x.plot(cutoff_time, x2, label="x augmented", color="tab:red")
    ax_x.plot(cutoff_time, x_sp, label="x setpoint", color="tab:orange", linestyle="--")
    ax_x.set_title("X vs Time")
    ax_x.set_xlabel("Time [s]")
    ax_x.set_ylabel("X [m]")
    ax_x.grid(True)
    ax_x.legend()

    ax_y = fig.add_subplot(2, 2, 3)
    ax_y.plot(cutoff_time, y, label="y pid" if dual_graph else "y", color="tab:blue")
    if dual_graph:
        ax_y.plot(cutoff_time, y2, label="y augmented", color="tab:red")
    ax_y.plot(cutoff_time, y_sp, label="y setpoint", color="tab:orange", linestyle="--")
    ax_y.set_title("Y vs Time")
    ax_y.set_xlabel("Time [s]")
    ax_y.set_ylabel("Y [m]")
    ax_y.grid(True)
    ax_y.legend()

    ax_z = fig.add_subplot(2, 2, 4)
    ax_z.plot(cutoff_time, z, label="z pid" if dual_graph else "z", color="tab:blue")
    if dual_graph:
        ax_z.plot(cutoff_time, z2, label="z augmented", color="tab:red")
    ax_z.plot(cutoff_time, z_sp, label="z setpoint", color="tab:orange", linestyle="--")
    ax_z.set_title("Z vs Time")
    ax_z.set_xlabel("Time [s]")
    ax_z.set_ylabel("Z [m]")
    ax_z.grid(True)
    ax_z.legend()

    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()

# `1`, `2`, `3` and `4` all
