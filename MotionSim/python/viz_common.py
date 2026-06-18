"""Shared helpers for the motion-log visualisers.

Loads motion_log.csv (produced by the C simulator) and derives the quantities
the firmware does not log directly (wheel/centre speed). Robot geometry mirrors
the constants in dsp/main.c so distances agree with the C side.
"""
import numpy as np
import pandas as pd

# Mirror dsp/main.c
PI         = 3.141592
WHEEL_R    = 0.0257
WHEEL_BASE = 0.111
STEP_DIST  = WHEEL_R * (1.8 / 180.0 * PI)   # metres per 1.8 deg step


def load(path):
    """Read a motion log and append derived columns (t_s, speeds, theta_rad)."""
    df = pd.read_csv(path)

    # Drop duplicated trailing rows (sim writes a final state snapshot).
    df = df.drop_duplicates(subset="tick", keep="last").reset_index(drop=True)

    t = df["time_us"].to_numpy(dtype=float) * 1e-6
    df["t_s"] = t
    df["theta_rad"] = np.deg2rad(df["theta_deg"].to_numpy(dtype=float))

    df["v_left"]  = _speed(df["leftSteps"].to_numpy(dtype=float), t)
    df["v_right"] = _speed(df["rightSteps"].to_numpy(dtype=float), t)
    df["v_center"] = 0.5 * (df["v_left"] + df["v_right"])

    # Light smoothing for display (non-uniform/event-triggered sampling is spiky).
    win = max(1, len(df) // 200)
    df["v_center_s"] = (
        df["v_center"].rolling(win, center=True, min_periods=1).mean()
    )
    return df


def _speed(steps, t):
    """Signed wheel speed [m/s] from cumulative step count vs time."""
    dist = steps * STEP_DIST
    dt = np.gradient(t)
    dt[dt == 0] = np.nan
    v = np.gradient(dist) / dt
    return np.nan_to_num(v, nan=0.0, posinf=0.0, neginf=0.0)


def subsample(n, frames):
    """Indices into an n-row log for at most `frames` animation steps."""
    if frames <= 0 or frames >= n:
        return np.arange(n)
    return np.unique(np.linspace(0, n - 1, frames).astype(int))
