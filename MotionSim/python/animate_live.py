#!/usr/bin/env python3
"""Live animation driven directly by the C simulator (no intermediate file).

Launches ./motion_sim with --stream, reads its CSV from stdout in a background
thread, and animates the trajectory + live time-series as rows arrive.

    python animate_live.py curve
    python animate_live.py movevp --decim 100 --fps 30
    python animate_live.py curve --save live.gif      # collect, then render

Because the C simulator runs far faster than real time, the data fills the
buffer almost instantly; the animation then plays it back at --fps. The point
is that it is fed straight from the running C process, so editing the C code
and re-running immediately shows the new behaviour.
"""
import argparse
import subprocess
import sys
import threading

import matplotlib
import numpy as np

import viz_common as vc


def reader_thread(proc, rows, lock, done):
    """Append each streamed CSV data row (as floats) to the shared buffer."""
    for line in proc.stdout:
        line = line.strip()
        if not line:
            continue
        try:
            vals = [float(v) for v in line.split(",")]
        except ValueError:
            continue  # header or stray text
        with lock:
            rows.append(vals)
    done.set()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scenario", nargs="?", default="movevp",
                    choices=["movevp", "curve"])
    ap.add_argument("--bin", default="../sim/motion_sim")
    ap.add_argument("--decim", type=int, default=100)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--realtime", action="store_true",
                    help="pace the C sim to wall-clock time (true real-time)")
    ap.add_argument("--speed", type=float, default=1.0,
                    help="real-time multiplier (e.g. 2 = twice as fast)")
    ap.add_argument("--maxpts", type=int, default=2500)
    ap.add_argument("--frames", type=int, default=300,
                    help="number of frames when --save is used")
    ap.add_argument("--save", help="collect the full run then render to GIF/MP4")
    ap.add_argument("--dpi", type=int, default=80)
    args = ap.parse_args()

    if args.save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    # ---- launch the C simulator in stream mode ----
    cmd = [args.bin, args.scenario, "--stream", "--decim", str(args.decim)]
    # Real-time pacing would make a file render wait the full run; skip on --save.
    if args.realtime and not args.save:
        cmd += ["--realtime", "--speed", str(args.speed)]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, text=True, bufsize=1)
    header = proc.stdout.readline().strip().split(",")
    col = {name: i for i, name in enumerate(header)}

    rows, lock, done = [], threading.Lock(), threading.Event()
    threading.Thread(target=reader_thread, args=(proc, rows, lock, done),
                     daemon=True).start()

    # For --save we need a finite frame count: drain everything first.
    if args.save:
        done.wait()

    # ---- figure ----
    fig = plt.figure(figsize=(14, 8))
    gs = fig.add_gridspec(4, 2, width_ratios=[1.2, 1.0])

    axr = fig.add_subplot(gs[:, 0])
    axr.set_aspect("equal"); axr.grid(True, alpha=0.3)
    axr.set_xlabel("x [m]"); axr.set_ylabel("y [m]"); axr.set_title("Robot (live)")
    (trail_ln,) = axr.plot([], [], "b-", lw=1.2, alpha=0.7)
    (robot_pt,) = axr.plot([], [], "ko", ms=7)
    (head_ln,) = axr.plot([], [], "r-", lw=2)
    txt = axr.text(0.02, 0.98, "", transform=axr.transAxes, va="top",
                   fontsize=9, family="monospace",
                   bbox=dict(boxstyle="round", fc="w", alpha=0.7))

    ax1 = fig.add_subplot(gs[0, 1]); ax1.set_ylabel("speed\n[m/s]")
    (v_ln,) = ax1.plot([], [], "b", lw=1)
    ax2 = fig.add_subplot(gs[1, 1]); ax2.set_ylabel("VP Adr")
    (al_ln,) = ax2.plot([], [], "b", lw=1, label="curL")
    (ar_ln,) = ax2.plot([], [], "r", lw=1, label="curR")
    ax2.legend(fontsize=6, ncol=2)
    ax3 = fig.add_subplot(gs[2, 1]); ax3.set_ylabel("flags"); ax3.set_ylim(-0.2, 1.2)
    (cm_ln,) = ax3.plot([], [], "m", lw=1, label="curveMode")
    (md_ln,) = ax3.plot([], [], "k", lw=1, label="motionDone")
    ax3.legend(fontsize=6, ncol=2)
    ax4 = fig.add_subplot(gs[3, 1]); ax4.set_ylabel("curve\nsteps"); ax4.set_xlabel("time [s]")
    (rem_ln,) = ax4.plot([], [], "c", lw=1, label="remain")
    (brk_ln,) = ax4.plot([], [], "orange", lw=1, label="brake")
    ax4.legend(fontsize=6)
    for ax in (ax1, ax2, ax3, ax4):
        ax.grid(True, alpha=0.3)

    def update(k):
        with lock:
            if not rows:
                return ()
            data = np.array(rows)
        # Live: show everything received so far (the buffer grows).
        # Save: the buffer is already full, so grow with the frame index.
        if args.save and nframes:
            show_n = max(2, int(round((k + 1) / nframes * len(data))))
            data = data[:show_n]
        m = len(data)
        if m > args.maxpts:
            sel = np.unique(np.linspace(0, m - 1, args.maxpts).astype(int))
            data = data[sel]

        t  = data[:, col["time_us"]] * 1e-6
        x  = data[:, col["x"]]
        y  = data[:, col["y"]]
        th = np.deg2rad(data[:, col["theta_deg"]])
        v  = vc._speed(0.5 * (data[:, col["leftSteps"]] + data[:, col["rightSteps"]]), t)

        trail_ln.set_data(x, y)
        robot_pt.set_data([x[-1]], [y[-1]])
        span = max(np.ptp(x), np.ptp(y), 1e-3)
        L = 0.08 * span
        head_ln.set_data([x[-1], x[-1] + L * np.cos(th[-1])],
                         [y[-1], y[-1] + L * np.sin(th[-1])])

        v_ln.set_data(t, v)
        al_ln.set_data(t, data[:, col["currentAdrL"]])
        ar_ln.set_data(t, data[:, col["currentAdrR"]])
        cm_ln.set_data(t, data[:, col["curveMode"]])
        md_ln.set_data(t, data[:, col["motionDone"]])
        rem_ln.set_data(t, data[:, col["remainDiff"]])
        brk_ln.set_data(t, data[:, col["brakeDiff"]])

        for ax in (axr, ax1, ax2, ax4):
            ax.relim(); ax.autoscale_view()

        txt.set_text(f"t={t[-1]:7.3f}s\nv={v[-1]:+.3f} m/s\n"
                     f"theta={data[-1, col['theta_deg']]:8.1f} deg\n"
                     f"rows={m}")

        if done.is_set() and not args.save:
            ani.event_source.stop()
        return ()

    fig.suptitle(f"Live: {' '.join(cmd)}", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.97))

    # When saving we know the row count; otherwise stream indefinitely.
    nframes = None
    if args.save:
        nframes = max(1, args.frames)

    ani = FuncAnimation(fig, update, frames=nframes, interval=1000 / args.fps,
                        blit=False, repeat=False, cache_frame_data=False)

    if args.save:
        if args.save.endswith(".gif"):
            ani.save(args.save, writer=PillowWriter(fps=args.fps), dpi=args.dpi)
        else:
            ani.save(args.save, fps=args.fps, dpi=args.dpi)
        print(f"saved {args.save}")
    else:
        plt.show()

    proc.wait()
    return 0


if __name__ == "__main__":
    sys.exit(main())
