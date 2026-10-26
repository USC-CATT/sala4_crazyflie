#!/usr/bin/env python3
"""Lightweight live telemetry display for host-side Crazyflie controllers.

Attaches directly to a cflib LogConfig callback or accepts individual
telemetry snapshots. Shows a 3D flight box, a numeric telemetry panel,
and a battery gauge.

Usage from a controller:

    from live_display import LiveDisplay

    display = LiveDisplay()
    self.log_state.data_received_cb.add_callback(display.log_callback)
    ...
    display.close()

Every datapoint is optional; only push what you have.
"""

import threading
import time
from collections import deque
from typing import Optional, Tuple

try:
    import matplotlib

    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
except Exception as exc:
    matplotlib = None
    plt = None
    _MATPLOTLIB_IMPORT_ERROR = exc
else:
    _MATPLOTLIB_IMPORT_ERROR = None

BOX_SIZE_M = 2.0
UPDATE_HZ = 15.0
HISTORY_SECONDS = 90.0
STALE_TIMEOUT_S = 0.75

BG_COLOR = "#0f172a"
PANEL_BG = "#1e293b"
FG_COLOR = "#e2e8f0"
MUTED_COLOR = "#94a3b8"
GRID_COLOR = "#334155"
TRAIL_COLOR = "#38bdf8"
DRONE_COLOR = "#f43f5e"
SETPOINT_COLOR = "#4ade80"
ACCENT_COLOR = "#facc15"


class _NullDisplay:
    """No-op stand-in used when matplotlib is unavailable."""

    def __init__(self, reason: str):
        print(f"[LiveDisplay] Disabled: {reason}")

    def log_callback(self, _timestamp, data, _logconf):
        return

    def update_state(self, **_kwargs):
        return

    def update_position(self, *_args, **_kwargs):
        return

    def update_velocity(self, *_args, **_kwargs):
        return

    def update_attitude(self, *_args, **_kwargs):
        return

    def update_gyro(self, *_args, **_kwargs):
        return

    def update_battery(self, *_args, **_kwargs):
        return

    def update_setpoint(self, *_args, **_kwargs):
        return

    def close(self):
        return


def LiveDisplay(*args, **kwargs):
    """Create a live display, falling back to a no-op if matplotlib is missing."""
    if _MATPLOTLIB_IMPORT_ERROR is not None:
        return _NullDisplay(str(_MATPLOTLIB_IMPORT_ERROR))
    return _TelemetryDisplay(*args, **kwargs)


class _TelemetryDisplay:
    """Matplotlib-backed telemetry dashboard updated from log callbacks."""

    def __init__(
        self,
        box_size_m: float = BOX_SIZE_M,
        update_hz: float = UPDATE_HZ,
        history_s: float = HISTORY_SECONDS,
    ):
        self._half = box_size_m / 2.0
        self._period = 1.0 / max(update_hz, 1.0)
        max_points = max(100, int(history_s * 50))

        self._lock = threading.Lock()
        self._stop = threading.Event()

        self._position: Optional[Tuple[float, float, float]] = None
        self._velocity: Optional[Tuple[float, float, float]] = None
        self._attitude: Optional[Tuple[float, float, float]] = None
        self._gyro: Optional[Tuple[float, float, float]] = None
        self._setpoint: Optional[Tuple[float, float, float]] = None
        self._vbat: Optional[float] = None
        self._last_update: Optional[float] = None
        self._trail_x: deque = deque(maxlen=max_points)
        self._trail_y: deque = deque(maxlen=max_points)
        self._trail_z: deque = deque(maxlen=max_points)

        self._fig = None

        self._thread = threading.Thread(target=self._gui_main, daemon=True)
        self._thread.start()

    def log_callback(self, _timestamp, data, _logconf):
        """cflib LogConfig callback; accepts the raw log data dict."""
        try:
            position = self._triple(data, "stateEstimate.x", "stateEstimate.y", "stateEstimate.z")
            velocity = self._triple(data, "stateEstimate.vx", "stateEstimate.vy", "stateEstimate.vz")
            attitude = self._triple(data, "stateEstimate.roll", "stateEstimate.pitch", "stateEstimate.yaw")
            gyro = self._triple(data, "gyro.x", "gyro.y", "gyro.z")
        except KeyError:
            return
        self.update_state(
            position=position,
            velocity=velocity,
            attitude=attitude,
            gyro=gyro,
            battery=data.get("pm.vbat"),
        )

    def update_state(
        self,
        position: Optional[Tuple[float, float, float]] = None,
        velocity: Optional[Tuple[float, float, float]] = None,
        attitude: Optional[Tuple[float, float, float]] = None,
        gyro: Optional[Tuple[float, float, float]] = None,
        battery: Optional[float] = None,
        setpoint: Optional[Tuple[float, float, float]] = None,
    ) -> None:
        with self._lock:
            if position is not None:
                self._position = position
                self._trail_x.append(position[0])
                self._trail_y.append(position[1])
                self._trail_z.append(position[2])
            if velocity is not None:
                self._velocity = velocity
            if attitude is not None:
                self._attitude = attitude
            if gyro is not None:
                self._gyro = gyro
            if battery is not None:
                self._vbat = battery
            if setpoint is not None:
                self._setpoint = setpoint
            self._last_update = time.monotonic()

    def update_position(self, x: float, y: float, z: float) -> None:
        self.update_state(position=(x, y, z))

    def update_velocity(self, vx: float, vy: float, vz: float) -> None:
        self.update_state(velocity=(vx, vy, vz))

    def update_attitude(self, roll: float, pitch: float, yaw: float) -> None:
        self.update_state(attitude=(roll, pitch, yaw))

    def update_gyro(self, gx: float, gy: float, gz: float) -> None:
        self.update_state(gyro=(gx, gy, gz))

    def update_battery(self, vbat: float) -> None:
        self.update_state(battery=vbat)

    def update_setpoint(self, x: float, y: float, z: float) -> None:
        self.update_state(setpoint=(x, y, z))

    def close(self) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)

    @staticmethod
    def _triple(data, key_x: str, key_y: str, key_z: str):
        return (data[key_x], data[key_y], data[key_z])

    def _build_figure(self) -> None:
        try:
            plt.ion()
            fig = plt.figure(figsize=(12.5, 6.5), facecolor=BG_COLOR)
            fig.canvas.manager.set_window_title("Crazyflie Telemetry")
            ax_3d = fig.add_axes([0.03, 0.08, 0.55, 0.84], projection="3d", facecolor=PANEL_BG)
            ax_text = fig.add_axes([0.62, 0.34, 0.35, 0.58], facecolor=PANEL_BG)
            ax_bat = fig.add_axes([0.62, 0.12, 0.35, 0.10], facecolor=PANEL_BG)
        except Exception as exc:
            print(f"[LiveDisplay] Could not open a window ({exc}); running headless.")
            self._fig = None
            return

        self._fig = fig
        self._ax_3d = ax_3d
        self._ax_text = ax_text
        self._ax_bat = ax_bat

        h = self._half
        top = 2.0 * h
        edges = [
            ((-h, -h, 0.0), (h, -h, 0.0)), ((-h, h, 0.0), (h, h, 0.0)),
            ((-h, -h, top), (h, -h, top)), ((-h, h, top), (h, h, top)),
            ((-h, -h, 0.0), (-h, h, 0.0)), ((h, -h, 0.0), (h, h, 0.0)),
            ((-h, -h, top), (-h, h, top)), ((h, -h, top), (h, h, top)),
            ((-h, -h, 0.0), (-h, -h, top)), ((h, -h, 0.0), (h, -h, top)),
            ((-h, h, 0.0), (-h, h, top)), ((h, h, 0.0), (h, h, top)),
        ]
        for start, end in edges:
            ax_3d.plot(
                [start[0], end[0]], [start[1], end[1]], [start[2], end[2]],
                color=GRID_COLOR, linewidth=1.0,
            )
        ax_3d.plot([0, 0], [0, 0], [0.0, top], color=GRID_COLOR, linewidth=0.6, alpha=0.5)
        ax_3d.plot([0, 0], [-h, h], [0.0, 0.0], color=GRID_COLOR, linewidth=0.6, alpha=0.5)
        ax_3d.plot([-h, h], [0, 0], [0.0, 0.0], color=GRID_COLOR, linewidth=0.6, alpha=0.5)

        (self._trail_line,) = ax_3d.plot([], [], [], color=TRAIL_COLOR, linewidth=1.8, alpha=0.9)
        (self._drone_point,) = ax_3d.plot([], [], [], marker="o", color=DRONE_COLOR, markersize=9)
        (self._setpoint_point,) = ax_3d.plot([], [], [], marker="X", color=SETPOINT_COLOR, markersize=9)

        ax_3d.set_xlim(-h, h)
        ax_3d.set_ylim(-h, h)
        ax_3d.set_zlim(0.0, top)
        ax_3d.set_box_aspect((1, 1, 1))
        ax_3d.set_xlabel("X (m)", color=MUTED_COLOR, labelpad=0)
        ax_3d.set_ylabel("Y (m)", color=MUTED_COLOR, labelpad=0)
        ax_3d.set_zlabel("Z (m)", color=MUTED_COLOR, labelpad=0)
        ax_3d.tick_params(colors=MUTED_COLOR, labelsize=8)
        ax_3d.set_title(f"Flight Box  {2 * self._half:.0f} x {2 * self._half:.0f} x {2 * self._half:.0f} m",
                        color=FG_COLOR, fontsize=12, pad=12)
        try:
            ax_3d.xaxis.set_pane_color(BG_COLOR)
            ax_3d.yaxis.set_pane_color(BG_COLOR)
            ax_3d.zaxis.set_pane_color(BG_COLOR)
        except Exception:
            pass

        ax_text.set_xticks([])
        ax_text.set_yticks([])
        for spine in ax_text.spines.values():
            spine.set_color(GRID_COLOR)
        self._text_artist = ax_text.text(
            0.04, 0.97, "", transform=ax_text.transAxes, family="monospace",
            fontsize=11, color=FG_COLOR, verticalalignment="top",
        )

        ax_bat.set_xlim(3.0, 4.3)
        ax_bat.set_ylim(0.0, 1.0)
        ax_bat.set_yticks([])
        ax_bat.tick_params(colors=MUTED_COLOR, labelsize=8)
        ax_bat.set_xlabel("Battery (V)", color=MUTED_COLOR, fontsize=9)
        for spine in ("top", "right", "left"):
            ax_bat.spines[spine].set_visible(False)
        self._bat_artist = ax_bat.barh([0.5], [0.0], height=0.55, color=ACCENT_COLOR)[0]
        self._bat_label = ax_bat.text(
            0.98, 0.5, "", transform=ax_bat.transAxes, family="monospace",
            fontsize=11, color=FG_COLOR, verticalalignment="center", horizontalalignment="right",
        )

    def _gui_main(self) -> None:
        self._build_figure()
        if self._fig is None:
            self._stop.wait()
            return
        try:
            plt.show(block=False)
            while not self._stop.wait(self._period):
                try:
                    self._render()
                except Exception:
                    pass
        finally:
            try:
                plt.close(self._fig)
            except Exception:
                pass

    def _render(self) -> None:
        with self._lock:
            position = self._position
            velocity = self._velocity
            attitude = self._attitude
            gyro = self._gyro
            setpoint = self._setpoint
            vbat = self._vbat
            last_update = self._last_update
            trail = (list(self._trail_x), list(self._trail_y), list(self._trail_z))

        stale = last_update is None or (time.monotonic() - last_update) > STALE_TIMEOUT_S

        self._trail_line.set_data(trail[0], trail[1])
        self._trail_line.set_3d_properties(trail[2])
        if position is not None and not stale:
            self._drone_point.set_data([position[0]], [position[1]])
            self._drone_point.set_3d_properties([position[2]])
        else:
            self._drone_point.set_data([], [])
            self._drone_point.set_3d_properties([])
        if setpoint is not None:
            self._setpoint_point.set_data([setpoint[0]], [setpoint[1]])
            self._setpoint_point.set_3d_properties([setpoint[2]])
        else:
            self._setpoint_point.set_data([], [])
            self._setpoint_point.set_3d_properties([])

        self._text_artist.set_text(self._format_panel(position, velocity, attitude, gyro, setpoint, stale))

        if vbat is not None:
            self._bat_artist.set_width(max(0.0, min(vbat, 4.3) - 3.0))
            self._bat_artist.set_color(self._battery_color(vbat))
            self._bat_label.set_text(f"{vbat:.2f} V")
        else:
            self._bat_label.set_text("---")

        self._fig.canvas.draw()
        tk_canvas = getattr(self._fig.canvas, "_tkcanvas", None)
        if tk_canvas is not None:
            tk_canvas.update()
        else:
            self._fig.canvas.flush_events()

    def _format_panel(self, position, velocity, attitude, gyro, setpoint, stale: bool) -> str:
        rows = [
            "  POSITION (m)        VELOCITY (m/s)",
            f"  x  {self._fmt(position, 0)}   vx  {self._fmt(velocity, 0)}",
            f"  y  {self._fmt(position, 1)}   vy  {self._fmt(velocity, 1)}",
            f"  z  {self._fmt(position, 2)}   vz  {self._fmt(velocity, 2)}",
            "",
            "  ATTITUDE (deg)      GYRO (deg/s)",
            f"  roll  {self._fmt(attitude, 0)}   gx  {self._fmt(gyro, 0)}",
            f"  pitch {self._fmt(attitude, 1)}   gy  {self._fmt(gyro, 1)}",
            f"  yaw   {self._fmt(attitude, 2)}   gz  {self._fmt(gyro, 2)}",
            "",
            "  SETPOINT (m)",
            f"  x  {self._fmt(setpoint, 0)}    y  {self._fmt(setpoint, 1)}    z  {self._fmt(setpoint, 2)}",
        ]
        if stale:
            rows.append("")
            rows.append("  NO DATA")
        return "\n".join(rows)

    @staticmethod
    def _fmt(values: Optional[Tuple[float, float, float]], index: int) -> str:
        if values is None:
            return "  ---  "
        return f"{values[index]:+7.2f}"

    @staticmethod
    def _battery_color(vbat: float) -> str:
        if vbat >= 3.9:
            return "#4ade80"
        if vbat >= 3.6:
            return "#facc15"
        return "#f87171"


def _demo() -> None:
    display = LiveDisplay()
    try:
        t0 = time.monotonic()
        while True:
            t = time.monotonic() - t0
            display.update_state(
                position=(0.8 * math.cos(0.5 * t), 0.8 * math.sin(0.5 * t), 0.8 + 0.1 * math.sin(t)),
                velocity=(-0.4 * math.sin(0.5 * t), 0.4 * math.cos(0.5 * t), 0.1 * math.cos(t)),
                attitude=(5.0 * math.sin(0.5 * t), -5.0 * math.cos(0.5 * t), t * 5.0 % 360.0),
                gyro=(10.0 * math.cos(0.5 * t), 10.0 * math.sin(0.5 * t), 2.0),
                battery=4.2 - 0.0001 * t,
                setpoint=(0.8, 0.0, 0.85),
            )
            time.sleep(0.04)
    except KeyboardInterrupt:
        pass
    finally:
        display.close()


if __name__ == "__main__":
    import math

    _demo()
