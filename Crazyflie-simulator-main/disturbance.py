import numpy as np

# Flexible wind/disturbance model per axis (world frame).
# You can choose a different waveform for X, Y, Z independently.
#
# Supported per-axis modes:
# - "off"      : always zero
# - "sin"      : offset + A*sin(2*pi*f*(t-t0)+phase)
# - "constant" : fixed constant force
# - "square"   : high/low square wave with duty cycle
#
# Quick examples:
# 1) Constant force only in +X:
#    FX_MODE = "constant"; FX_CONSTANT = 0.15
#    FY_MODE = "off"; FZ_MODE = "off"
#
# 2) Square wave in +Y/-Y and no other axes:
#    FX_MODE = "off"
#    FY_MODE = "square"; FY_SQUARE_HIGH = 0.20; FY_SQUARE_LOW = -0.20; FY_FREQUENCY_HZ = 0.5
#    FZ_MODE = "off"

START_TIME = 2.0  # seconds
END_TIME = None   # seconds, set to a float to stop disturbance after this time

# Disturbance mode constants.
MODE_OFF = "off"
MODE_SIN = "sin"
MODE_CONSTANT = "constant"
MODE_SQUARE = "square"

# ---------------- X axis ----------------
FX_MODE = MODE_SIN 
# Sinusoid: Fx(t) = Fx_offset + A_x * sin(2*pi*f_x*(t-t0) + phi_x)
FX_AMPLITUDE = 0.4  # Newton
FX_FREQUENCY_HZ = 0.1
FX_PHASE = 0.0  # rad
FX_OFFSET = 0.0  # Newton
# Constant mode:
FX_CONSTANT = 0.2  # Newton
# Square mode:
FX_SQUARE_HIGH = 0.4  # Newton
FX_SQUARE_LOW = -0.4  # Newton
FX_SQUARE_DUTY = 0.5  # fraction in [0,1]

# ---------------- Y axis ----------------
FY_MODE = MODE_SIN
# Sinusoid: Fy(t) = Fy_offset + A_y * sin(2*pi*f_y*(t-t0) + phi_y)
FY_AMPLITUDE = 0.4  # Newton
FY_FREQUENCY_HZ = 0.05
FY_PHASE = 0.0  # rad
FY_OFFSET = 0.0  # Newton
# Constant mode:
FY_CONSTANT = 0.0  # Newton
# Square mode:
FY_SQUARE_HIGH = 0.4  # Newton
FY_SQUARE_LOW = -0.4  # Newton
FY_SQUARE_DUTY = 0.5  # fraction in [0,1]

# ---------------- Z axis ----------------
# Keep Z disturbance off by default.
FZ_MODE = MODE_OFF
# Sinusoid: Fz(t) = Fz_offset + A_z * sin(2*pi*f_z*(t-t0) + phi_z)
FZ_AMPLITUDE = 0.0  # Newton
FZ_FREQUENCY_HZ = 0.5
FZ_PHASE = 0.0  # rad
FZ_OFFSET = 0.0  # Newton
# Constant mode:
FZ_CONSTANT = 0.0  # Newton
# Square mode:
FZ_SQUARE_HIGH = 0.0  # Newton
FZ_SQUARE_LOW = 0.0  # Newton
FZ_SQUARE_DUTY = 0.5  # fraction in [0,1]


def _sinusoid(amplitude, frequency_hz, phase, t_rel):
    return float(amplitude) * np.sin(2.0 * np.pi * float(frequency_hz) * t_rel + float(phase))


def _square_wave(high, low, frequency_hz, phase, duty, t_rel):
    duty = float(np.clip(duty, 0.0, 1.0))
    frequency_hz = float(frequency_hz)
    if frequency_hz <= 0.0:
        return float(high)

    phase_cycles = float(phase) / (2.0 * np.pi)
    phase_in_period = (t_rel * frequency_hz + phase_cycles) % 1.0
    return float(high) if phase_in_period < duty else float(low)


def _axis_force(mode, t_rel, amplitude, frequency_hz, phase, offset, constant, square_high, square_low, square_duty):
    mode = str(mode).strip().lower()
    if mode == MODE_OFF:
        return 0.0
    if mode == MODE_SIN:
        return float(offset) + _sinusoid(amplitude, frequency_hz, phase, t_rel)
    if mode == MODE_CONSTANT:
        return float(constant)
    if mode == MODE_SQUARE:
        return _square_wave(square_high, square_low, frequency_hz, phase, square_duty, t_rel)
    raise ValueError("Unknown disturbance mode '{}' (expected off/sin/constant/square).".format(mode))


def force_world(t):
    """Flexible disturbance force in world frame at time t."""
    if t < START_TIME:
        return np.zeros(3)
    if END_TIME is not None and t > END_TIME:
        return np.zeros(3)

    t_rel = float(t - START_TIME)
    fx = _axis_force(
        FX_MODE, t_rel,
        FX_AMPLITUDE, FX_FREQUENCY_HZ, FX_PHASE, FX_OFFSET,
        FX_CONSTANT, FX_SQUARE_HIGH, FX_SQUARE_LOW, FX_SQUARE_DUTY
    )
    fy = _axis_force(
        FY_MODE, t_rel,
        FY_AMPLITUDE, FY_FREQUENCY_HZ, FY_PHASE, FY_OFFSET,
        FY_CONSTANT, FY_SQUARE_HIGH, FY_SQUARE_LOW, FY_SQUARE_DUTY
    )
    fz = _axis_force(
        FZ_MODE, t_rel,
        FZ_AMPLITUDE, FZ_FREQUENCY_HZ, FZ_PHASE, FZ_OFFSET,
        FZ_CONSTANT, FZ_SQUARE_HIGH, FZ_SQUARE_LOW, FZ_SQUARE_DUTY
    )
    return np.array([fx, fy, fz], dtype=float)


def _axis_formula_text(axis_name, mode, amplitude, frequency_hz, phase, offset, constant, square_high, square_low, square_duty):
    mode = str(mode).strip().lower()
    if mode == MODE_OFF:
        return "{}(t) = 0".format(axis_name)
    if mode == MODE_SIN:
        return "{}(t) = {:.3f} + {:.3f} sin(2pi*{:.3f}(t-t0)+{:.3f})".format(
            axis_name, float(offset), float(amplitude), float(frequency_hz), float(phase)
        )
    if mode == MODE_CONSTANT:
        return "{}(t) = {:.3f} N".format(axis_name, float(constant))
    if mode == MODE_SQUARE:
        return "{}(t) = square(high={:.3f}, low={:.3f}, f={:.3f} Hz, duty={:.2f}, phase={:.3f})".format(
            axis_name, float(square_high), float(square_low), float(frequency_hz), float(square_duty), float(phase)
        )
    return "{}(t) = <invalid mode '{}'>".format(axis_name, mode)


def formula_text():
    return (
        "Wind model (flexible):\n"
        "active for t >= t0{} \n"
        "t0={:.2f} s\n"
        "{}\n"
        "{}\n"
        "{}"
    ).format(
        "" if END_TIME is None else " and t <= t1, t1={:.2f} s".format(float(END_TIME)),
        START_TIME,
        _axis_formula_text(
            "Fx", FX_MODE, FX_AMPLITUDE, FX_FREQUENCY_HZ, FX_PHASE, FX_OFFSET,
            FX_CONSTANT, FX_SQUARE_HIGH, FX_SQUARE_LOW, FX_SQUARE_DUTY
        ),
        _axis_formula_text(
            "Fy", FY_MODE, FY_AMPLITUDE, FY_FREQUENCY_HZ, FY_PHASE, FY_OFFSET,
            FY_CONSTANT, FY_SQUARE_HIGH, FY_SQUARE_LOW, FY_SQUARE_DUTY
        ),
        _axis_formula_text(
            "Fz", FZ_MODE, FZ_AMPLITUDE, FZ_FREQUENCY_HZ, FZ_PHASE, FZ_OFFSET,
            FZ_CONSTANT, FZ_SQUARE_HIGH, FZ_SQUARE_LOW, FZ_SQUARE_DUTY
        )
    )
