#!/usr/bin/env python3
import os
from pathlib import Path
import json
import numpy as np

MPL_CACHE_DIR = Path("results/.mplconfig")
MPL_CACHE_DIR.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(MPL_CACHE_DIR.resolve()))

import matplotlib

# Force non-interactive backend for headless report generation.
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import controller
import augmented_controller
import disturbance
import runsim
import trajGen3D
from model.quadcopter import Quadcopter
import model.params as params


OUT_DIR = Path("results/augmented_report")
FIG_XY = OUT_DIR / "fig_xy_trajectory.png"
FIG_ERR = OUT_DIR / "fig_error_norm.png"
FIG_AUG = OUT_DIR / "fig_aug_signals.png"
METRICS_JSON = OUT_DIR / "metrics.json"
REPORT_TEX = OUT_DIR / "augmented_controller_report.tex"


def desired_hover_state():
    return trajGen3D.DesiredState(
        pos=np.array(runsim.HOVER_SETPOINT, dtype=float).copy(),
        vel=np.zeros(3),
        acc=np.zeros(3),
        yaw=float(runsim.HOVER_YAW),
        yawdot=0.0,
    )


def simulate(mode, duration_s):
    dt = float(runsim.dt)
    n = int(np.ceil(float(duration_s) / dt))

    quad = Quadcopter(runsim.INITIAL_POS, runsim.INITIAL_ATTITUDE)

    if mode == "baseline":
        backend = None
        controller.set_dt(dt)
        controller.reset()
    elif mode == "augmented":
        backend = augmented_controller.AugmentedController(dt=dt)
        backend.set_dt(dt)
        backend.reset()
    else:
        raise ValueError("Unknown mode: {}".format(mode))

    t = np.zeros(n, dtype=float)
    pos = np.zeros((n, 3), dtype=float)
    err = np.zeros(n, dtype=float)
    wind = np.zeros((n, 3), dtype=float)
    v_aug = np.zeros((n, 2), dtype=float)

    sp = np.array(runsim.HOVER_SETPOINT, dtype=float).reshape(3,)

    for k in range(n):
        tk = k * dt
        des = desired_hover_state()

        if backend is None:
            F, M = controller.run(quad, des)
            v_xy = np.zeros(2, dtype=float)
        else:
            F, M = backend.run(quad, des, sim_time=tk)
            v_xy = np.asarray(backend.diagnostics()["v_aug_xy"], dtype=float).reshape(2,)

        wind_k = disturbance.force_world(tk)
        quad.update(dt, F, M, external_force_world=wind_k)

        t[k] = tk + dt
        pos[k] = np.asarray(quad.position(), dtype=float).reshape(3,)
        err[k] = float(np.linalg.norm(pos[k] - sp))
        wind[k] = np.asarray(wind_k, dtype=float).reshape(3,)
        v_aug[k] = v_xy

    return {
        "mode": mode,
        "t": t,
        "pos": pos,
        "err": err,
        "wind": wind,
        "v_aug": v_aug,
    }


def summarize(run):
    t = run["t"]
    e = run["err"]
    mask_post = t >= float(disturbance.START_TIME)
    if not np.any(mask_post):
        mask_post = np.ones_like(t, dtype=bool)

    e_post = e[mask_post]
    return {
        "max_err_all_m": float(np.max(e)),
        "max_err_post_wind_m": float(np.max(e_post)),
        "rms_err_post_wind_m": float(np.sqrt(np.mean(e_post ** 2))),
        "final_err_m": float(e[-1]),
    }


def make_plots(baseline, augmented):
    sp = np.array(runsim.HOVER_SETPOINT, dtype=float).reshape(3,)
    t = baseline["t"]

    # XY trajectory comparison.
    fig, ax = plt.subplots(figsize=(6.5, 5.5))
    ax.plot(baseline["pos"][:, 0], baseline["pos"][:, 1], label="Baseline PID", lw=1.8)
    ax.plot(augmented["pos"][:, 0], augmented["pos"][:, 1], label="PID + Augmented", lw=1.8)
    ax.plot(sp[0], sp[1], marker="x", color="black", markersize=9, mew=2.0, linestyle="None", label="Hover setpoint")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Hover-Wind XY Trajectory")
    ax.grid(True, alpha=0.3)
    ax.axis("equal")
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(FIG_XY, dpi=180)
    plt.close(fig)

    # Position-error norm vs time.
    fig, ax = plt.subplots(figsize=(7.2, 4.4))
    ax.plot(t, baseline["err"], label="Baseline PID", lw=1.8)
    ax.plot(t, augmented["err"], label="PID + Augmented", lw=1.8)
    ax.axvline(float(disturbance.START_TIME), color="k", ls="--", lw=1.2, label="Wind start")
    ax.set_xlabel("time (s)")
    ax.set_ylabel(r"$\|p - p_d\|$ (m)")
    ax.set_title("Position Error Norm")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(FIG_ERR, dpi=180)
    plt.close(fig)

    # Wind and augmented virtual input.
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7.2, 5.8), sharex=True)
    ax1.plot(t, augmented["wind"][:, 0], label="Fx", lw=1.5)
    ax1.plot(t, augmented["wind"][:, 1], label="Fy", lw=1.5)
    ax1.axvline(float(disturbance.START_TIME), color="k", ls="--", lw=1.0)
    ax1.set_ylabel("wind force (N)")
    ax1.set_title("Disturbance and Augmented Virtual Input")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="best")

    ax2.plot(t, augmented["v_aug"][:, 0], label=r"$v_x$", lw=1.5)
    ax2.plot(t, augmented["v_aug"][:, 1], label=r"$v_y$", lw=1.5)
    ax2.axvline(float(disturbance.START_TIME), color="k", ls="--", lw=1.0)
    ax2.set_xlabel("time (s)")
    ax2.set_ylabel(r"virtual input $v$ (m/s$^2$)")
    # Auto-zoom v signals so small corrections are visible.
    v_peak = float(np.max(np.abs(augmented["v_aug"])))
    v_ylim = 0.1 if v_peak < 1e-6 else 1.15 * v_peak
    ax2.set_ylim(-v_ylim, v_ylim)
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="best")

    fig.tight_layout()
    fig.savefig(FIG_AUG, dpi=180)
    plt.close(fig)


def write_report(metrics):
    base = metrics["baseline"]
    aug = metrics["augmented"]
    max_improve_pct = 100.0 * (base["max_err_post_wind_m"] - aug["max_err_post_wind_m"]) / max(base["max_err_post_wind_m"], 1e-12)
    rms_improve_pct = 100.0 * (base["rms_err_post_wind_m"] - aug["rms_err_post_wind_m"]) / max(base["rms_err_post_wind_m"], 1e-12)
    wind_formula_lines = disturbance.formula_text()

    tex = r"""\documentclass[11pt]{article}
\usepackage[margin=1in]{geometry}
\usepackage{amsmath,amssymb}
\usepackage{booktabs}
\usepackage{graphicx}
\usepackage{subcaption}
\usepackage{siunitx}

\title{Augmented Controller Explanation and Simulation Plots}
\date{}

\begin{document}
\maketitle

\section{Overview}
This report explains the augmented controller implemented in \texttt{augmented\_controller.py} and compares it against the baseline cascaded PID controller in a hover-with-wind test.

\section{Controller Structure}
The baseline controller outputs thrust and body moments through cascaded loops. The baseline outer position loop includes integral action on $x$/$y$:
\begin{equation}
v_{x,sp}=K_{p,xy}^{pos}e_x + K_{i,xy}^{pos}\int e_x dt,\quad
v_{y,sp}=K_{p,xy}^{pos}e_y + K_{i,xy}^{pos}\int e_y dt.
\end{equation}
The augmentation then modifies only the commanded lateral acceleration before it enters the baseline mapping:
\begin{equation}
a_{\mathrm{cmd,aug}} =
\begin{bmatrix}
a_{x,\mathrm{nom}} + v_x\\
a_{y,\mathrm{nom}} + v_y\\
a_{z,\mathrm{nom}}
\end{bmatrix}.
\end{equation}
The modified desired acceleration is then passed to the existing baseline controller unchanged.

\section{Implemented Augmentation Law}
For each lateral axis $i\in\{x,y\}$, define
\begin{equation}
x_i = \begin{bmatrix} p_i \\ \dot p_i \end{bmatrix}, \quad
A=\begin{bmatrix}0&1\\0&0\end{bmatrix}, \quad
B=\begin{bmatrix}0\\1\end{bmatrix}.
\end{equation}
The digital twin used in code is
\begin{equation}
\dot{\hat{x}}_i = A x_i + B u_{\mathrm{nom},i}
= \begin{bmatrix}\dot p_i\\u_{\mathrm{nom},i}\end{bmatrix},
\end{equation}
with model error
\begin{equation}
e_i = x_i - \hat{x}_i.
\end{equation}
Using a filtered derivative estimate:
\begin{equation}
\dot e_{f,i} = \frac{\dot e_i^{\mathrm{est}} - e_{f,i}}{\mu},
\end{equation}
the least-squares virtual input with $B=[0,1]^\top$ becomes
\begin{equation}
v_i = - (B^\top B)^{-1} B^\top e_{f,i} = -e_{f,i,2},
\end{equation}
followed by saturation
\begin{equation}
v_i \leftarrow \mathrm{clip}(v_i,\,-v_{\max},\,v_{\max}),
\end{equation}
and a start-time gate $v_i=0$ for $t<t_0$.

\section{Simulation Setup}
\begin{itemize}
\item Vehicle mass: \SI{%.3f}{kg}
\item Hover setpoint: $(%.2f, %.2f, %.2f)$ m
\item Wind start time: \SI{%.2f}{s}
\item Augmentation filter constant: $\mu=%.4f$
\item Augmentation limit: $v_{\max}=%.2f$ m/s$^2$
\item Baseline XY position integral gain: $K_{i,xy}^{pos}=%.3f$
\item Baseline XY position integral limit: $\pm %.3f$
\end{itemize}

\section{Wind Disturbance Model}
In \texttt{hover\_wind} mode, the active disturbance configuration is:
\begin{verbatim}
%s
\end{verbatim}

\section{Results}
\begin{figure}[h!]
\centering
\includegraphics[width=0.72\linewidth]{fig_xy_trajectory.png}
\caption{XY hover trajectories under the same wind disturbance.}
\end{figure}

\begin{figure}[h!]
\centering
\includegraphics[width=0.78\linewidth]{fig_error_norm.png}
\caption{Position error norm over time. The dashed line marks wind start.}
\end{figure}

\begin{figure}[h!]
\centering
\includegraphics[width=0.78\linewidth]{fig_aug_signals.png}
\caption{Applied wind forces and augmented virtual inputs.}
\end{figure}

\begin{table}[h!]
\centering
\caption{Error Metrics (post-wind interval)}
\begin{tabular}{lccc}
\toprule
Metric & Baseline PID & PID + Augmented & Improvement (\%%) \\
\midrule
Max position error [m] & %.4f & %.4f & %.2f \\
RMS error [m] & %.4f & %.4f & %.2f \\
\bottomrule
\end{tabular}
\end{table}

\section{Interpretation}
The augmentation does not replace the cascaded PID loops. It adds a bounded lateral correction term based on filtered model mismatch, then the baseline controller still performs acceleration-to-attitude and rate/moment control.

\end{document}
""" % (
        params.mass,
        float(runsim.HOVER_SETPOINT[0]),
        float(runsim.HOVER_SETPOINT[1]),
        float(runsim.HOVER_SETPOINT[2]),
        float(disturbance.START_TIME),
        float(augmented_controller.MU),
        float(augmented_controller.V_LIMIT_XY),
        float(controller.POS_XY_KI),
        float(controller.POS_XY_INT_LIM),
        wind_formula_lines,
        base["max_err_post_wind_m"],
        aug["max_err_post_wind_m"],
        max_improve_pct,
        base["rms_err_post_wind_m"],
        aug["rms_err_post_wind_m"],
        rms_improve_pct,
    )

    REPORT_TEX.write_text(tex)


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    duration_s = float(runsim.HOVER_WIND_SIM_DURATION)

    baseline = simulate("baseline", duration_s)
    augmented = simulate("augmented", duration_s)

    metrics = {
        "duration_s": duration_s,
        "dt_s": float(runsim.dt),
        "baseline": summarize(baseline),
        "augmented": summarize(augmented),
    }

    make_plots(baseline, augmented)
    METRICS_JSON.write_text(json.dumps(metrics, indent=2))
    write_report(metrics)

    print("Wrote:")
    print(" - {}".format(FIG_XY))
    print(" - {}".format(FIG_ERR))
    print(" - {}".format(FIG_AUG))
    print(" - {}".format(METRICS_JSON))
    print(" - {}".format(REPORT_TEX))
    print("")
    print("To build PDF:")
    print("  cd {}".format(OUT_DIR))
    print("  pdflatex augmented_controller_report.tex")


if __name__ == "__main__":
    main()
