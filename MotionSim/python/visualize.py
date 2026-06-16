#!/usr/bin/env python3
"""Static overview of a motion log.

Left:  the robot's 2D trajectory (colour = time), with start/end markers.
Right: centre speed, velocity-profile index (current vs target Adr), mode/flags,
       and the curve-mode remain/brake diagnostics.

Usage:
    python visualize.py [motion_log.csv] [--save out.png]
"""
import argparse
import sys

import matplotlib
import numpy as np

import viz_common as vc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="?", default="../sim/motion_log.csv")
    ap.add_argument("--save", help="write PNG instead of showing a window")
    args = ap.parse_args()

    if args.save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    df = vc.load(args.csv)
    t = df["t_s"].to_numpy()

    fig = plt.figure(figsize=(14, 8))
    gs = fig.add_gridspec(4, 2, width_ratios=[1.2, 1.0])

    # ---- Left: trajectory ----
    axt = fig.add_subplot(gs[:, 0])
    x, y = df["x"].to_numpy(), df["y"].to_numpy()
    sc = axt.scatter(x, y, c=t, cmap="viridis", s=6)
    axt.plot(x[0], y[0], "go", ms=10, label="start")
    axt.plot(x[-1], y[-1], "rs", ms=10, label="end")
    # heading arrow at the end
    th = df["theta_rad"].to_numpy()
    span = max(np.ptp(x), np.ptp(y), 1e-3)
    L = 0.06 * span
    axt.arrow(x[-1], y[-1], L * np.cos(th[-1]), L * np.sin(th[-1]),
              head_width=0.4 * L, color="r", length_includes_head=True)
    axt.set_aspect("equal", adjustable="datalim")
    axt.set_xlabel("x [m]"); axt.set_ylabel("y [m]")
    axt.set_title("Trajectory")
    axt.legend(loc="best"); axt.grid(True, alpha=0.3)
    fig.colorbar(sc, ax=axt, label="time [s]", fraction=0.046, pad=0.04)

    # ---- Right: time series ----
    ax1 = fig.add_subplot(gs[0, 1])
    ax1.plot(t, df["v_center_s"], "b")
    ax1.set_ylabel("speed [m/s]"); ax1.set_title("Centre speed")
    ax1.grid(True, alpha=0.3)

    ax2 = fig.add_subplot(gs[1, 1], sharex=ax1)
    ax2.plot(t, df["currentAdrL"], "b", label="curAdrL")
    ax2.plot(t, df["currentAdrR"], "r", label="curAdrR")
    ax2.plot(t, df["changeAdrL"], "b--", lw=0.8, label="tgtAdrL")
    ax2.plot(t, df["changeAdrR"], "r--", lw=0.8, label="tgtAdrR")
    ax2.set_ylabel("VP Adr"); ax2.legend(fontsize=7, ncol=2)
    ax2.grid(True, alpha=0.3)

    ax3 = fig.add_subplot(gs[2, 1], sharex=ax1)
    ax3.step(t, df["curveMode"], "m", where="post", label="curveMode")
    ax3.step(t, df["VP_ON"], "g", where="post", label="VP_ON")
    ax3.step(t, df["motionDone"], "k", where="post", label="motionDone")
    ax3.set_ylabel("flags"); ax3.set_ylim(-0.2, 1.2)
    ax3.legend(fontsize=7, ncol=3); ax3.grid(True, alpha=0.3)

    ax4 = fig.add_subplot(gs[3, 1], sharex=ax1)
    ax4.plot(t, df["remainDiff"], "c", label="remainDiff")
    ax4.plot(t, df["brakeDiff"], "orange", label="brakeDiff")
    ax4.set_ylabel("curve steps"); ax4.set_xlabel("time [s]")
    ax4.legend(fontsize=7); ax4.grid(True, alpha=0.3)

    fig.suptitle(f"Motion log: {args.csv}", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.97))

    if args.save:
        fig.savefig(args.save, dpi=110)
        print(f"saved {args.save}")
    else:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
