#!/usr/bin/env python3
"""Real-time animation of a motion log.

Left:  the robot driving its trajectory, with a heading arrow and a fading trail.
Right: live speed, velocity-profile index, mode/flags and curve remain/brake,
       each with a moving time cursor.

Usage:
    python animate.py [motion_log.csv]              # live window
    python animate.py motion_log.csv --save out.gif # render to GIF
    python animate.py --frames 600 --fps 30
"""
import argparse
import sys

import matplotlib
import numpy as np

import viz_common as vc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="?", default="../sim/motion_log.csv")
    ap.add_argument("--frames", type=int, default=500,
                    help="max animation frames (log is subsampled)")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--trail", type=int, default=120,
                    help="trail length in frames (0 = full)")
    ap.add_argument("--save", help="render to GIF/MP4 instead of showing")
    ap.add_argument("--dpi", type=int, default=80,
                    help="render DPI when saving (lower = faster/smaller)")
    ap.add_argument("--maxpts", type=int, default=2500,
                    help="cap working rows (keeps redraw cheap for big logs)")
    args = ap.parse_args()

    if args.save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    df = vc.load(args.csv)
    if len(df) > args.maxpts:        # decimate so every redraw stays cheap
        keep = np.unique(np.linspace(0, len(df) - 1, args.maxpts).astype(int))
        df = df.iloc[keep].reset_index(drop=True)
    n = len(df)
    idx = vc.subsample(n, args.frames)

    t  = df["t_s"].to_numpy()
    x  = df["x"].to_numpy()
    y  = df["y"].to_numpy()
    th = df["theta_rad"].to_numpy()

    fig = plt.figure(figsize=(14, 8))
    gs = fig.add_gridspec(4, 2, width_ratios=[1.2, 1.0])

    # ---- Left: robot view ----
    axr = fig.add_subplot(gs[:, 0])
    span = max(np.ptp(x), np.ptp(y), 1e-3)
    pad = 0.1 * span
    axr.set_xlim(x.min() - pad, x.max() + pad)
    axr.set_ylim(y.min() - pad, y.max() + pad)
    axr.set_aspect("equal")
    axr.set_xlabel("x [m]"); axr.set_ylabel("y [m]")
    axr.set_title("Robot")
    axr.grid(True, alpha=0.3)
    axr.plot(x[0], y[0], "go", ms=9, label="start")
    axr.plot(x[-1], y[-1], "rx", ms=9, label="target")
    axr.legend(loc="upper right", fontsize=8)
    (trail_ln,) = axr.plot([], [], "b-", lw=1.2, alpha=0.7)
    (robot_pt,) = axr.plot([], [], "ko", ms=7)
    (head_ln,) = axr.plot([], [], "r-", lw=2)
    arrow_len = 0.08 * span

    # ---- Right: live time series ----
    ax1 = fig.add_subplot(gs[0, 1])
    ax1.plot(t, df["v_center_s"], "b", lw=1)
    ax1.set_ylabel("speed\n[m/s]"); ax1.grid(True, alpha=0.3)

    ax2 = fig.add_subplot(gs[1, 1], sharex=ax1)
    ax2.plot(t, df["currentAdrL"], "b", lw=1, label="curL")
    ax2.plot(t, df["currentAdrR"], "r", lw=1, label="curR")
    ax2.plot(t, df["changeAdrL"], "b--", lw=0.7, label="tgtL")
    ax2.plot(t, df["changeAdrR"], "r--", lw=0.7, label="tgtR")
    ax2.set_ylabel("VP Adr"); ax2.legend(fontsize=6, ncol=2); ax2.grid(True, alpha=0.3)

    ax3 = fig.add_subplot(gs[2, 1], sharex=ax1)
    ax3.step(t, df["curveMode"], "m", where="post", label="curveMode")
    ax3.step(t, df["VP_ON"], "g", where="post", label="VP_ON")
    ax3.step(t, df["motionDone"], "k", where="post", label="motionDone")
    ax3.set_ylabel("flags"); ax3.set_ylim(-0.2, 1.2)
    ax3.legend(fontsize=6, ncol=3); ax3.grid(True, alpha=0.3)

    ax4 = fig.add_subplot(gs[3, 1], sharex=ax1)
    ax4.plot(t, df["remainDiff"], "c", lw=1, label="remain")
    ax4.plot(t, df["brakeDiff"], "orange", lw=1, label="brake")
    ax4.set_ylabel("curve\nsteps"); ax4.set_xlabel("time [s]")
    ax4.legend(fontsize=6); ax4.grid(True, alpha=0.3)

    cursors = [ax.axvline(t[0], color="gray", lw=1, ls=":")
               for ax in (ax1, ax2, ax3, ax4)]

    txt = axr.text(0.02, 0.98, "", transform=axr.transAxes, va="top",
                   fontsize=9, family="monospace",
                   bbox=dict(boxstyle="round", fc="w", alpha=0.7))

    def update(k):
        i = idx[k]
        lo = 0 if args.trail <= 0 else max(0, i - args.trail * (n // max(len(idx), 1) + 1))
        trail_ln.set_data(x[lo:i + 1], y[lo:i + 1])
        robot_pt.set_data([x[i]], [y[i]])
        head_ln.set_data([x[i], x[i] + arrow_len * np.cos(th[i])],
                         [y[i], y[i] + arrow_len * np.sin(th[i])])
        for c in cursors:
            c.set_xdata([t[i], t[i]])
        txt.set_text(
            f"t={t[i]:7.3f}s\n"
            f"v={df['v_center_s'].iloc[i]:+.3f} m/s\n"
            f"theta={df['theta_deg'].iloc[i]:8.1f} deg\n"
            f"curve={int(df['curveMode'].iloc[i])} "
            f"done={int(df['motionDone'].iloc[i])}"
        )
        return trail_ln, robot_pt, head_ln, txt, *cursors

    fig.suptitle(f"Motion replay: {args.csv}", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.97))

    ani = FuncAnimation(fig, update, frames=len(idx),
                        interval=1000 / args.fps, blit=True, repeat=False,
                        cache_frame_data=False)

    if args.save:
        if args.save.endswith(".gif"):
            ani.save(args.save, writer=PillowWriter(fps=args.fps), dpi=args.dpi)
        else:
            ani.save(args.save, fps=args.fps, dpi=args.dpi)
        print(f"saved {args.save}")
    else:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
