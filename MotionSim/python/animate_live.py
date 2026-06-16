#!/usr/bin/env python3
"""Live animation driven directly by the C simulator (no intermediate file).

Launches ./motion_sim with --stream, reads its CSV from stdout in a background
thread, and animates the trajectory + time-series.

    python animate_live.py curve                 # smooth frame-paced playback
    python animate_live.py movevp --fps 30
    python animate_live.py curve --realtime       # follow the C sim in real time
    python animate_live.py curve --save live.gif  # render to file

Two playback modes:
  * default / --save : the run is buffered, then played back frame-by-frame at
    --fps (smooth, uses blitting). The C sim fills the buffer almost instantly.
  * --realtime       : the C sim paces its output to wall-clock time and the
    plot follows the data as it arrives.
"""
import argparse
import subprocess
import sys
import threading

import matplotlib
import numpy as np

import viz_common as vc


def reader_thread(proc, rows, lock, done):
    for line in proc.stdout:
        line = line.strip()
        if not line:
            continue
        try:
            vals = [float(v) for v in line.split(",")]
        except ValueError:
            continue
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
    ap.add_argument("--frames", type=int, default=400,
                    help="playback frames (buffered modes)")
    ap.add_argument("--maxpts", type=int, default=2500)
    ap.add_argument("--realtime", action="store_true",
                    help="follow the C sim paced to wall-clock time")
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--save", help="render to GIF/MP4 instead of showing")
    ap.add_argument("--dpi", type=int, default=80)
    args = ap.parse_args()

    if args.save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    follow = args.realtime and not args.save   # live-follow vs buffered playback

    cmd = [args.bin, args.scenario, "--stream", "--decim", str(args.decim)]
    if follow:
        cmd += ["--realtime", "--speed", str(args.speed)]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, text=True, bufsize=1)
    header = proc.stdout.readline().strip().split(",")
    col = {name: i for i, name in enumerate(header)}

    rows, lock, done = [], threading.Lock(), threading.Event()
    threading.Thread(target=reader_thread, args=(proc, rows, lock, done),
                     daemon=True).start()

    if not follow:
        done.wait()   # buffer the whole run (instant unless paced)

    # ---- figure ----
    fig = plt.figure(figsize=(13, 7.5))
    gs = fig.add_gridspec(4, 2, width_ratios=[1.2, 1.0])

    axr = fig.add_subplot(gs[:, 0])
    axr.set_aspect("equal"); axr.grid(True, alpha=0.3)
    axr.set_xlabel("x [m]"); axr.set_ylabel("y [m]"); axr.set_title("Robot")
    (trail_ln,) = axr.plot([], [], "b-", lw=1.4, alpha=0.8)
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
    (cm_ln,) = ax3.plot([], [], "m", lw=1.4, label="curveMode")
    (md_ln,) = ax3.plot([], [], "k", lw=1.4, label="motionDone")
    ax3.legend(fontsize=6, ncol=2)
    ax4 = fig.add_subplot(gs[3, 1]); ax4.set_ylabel("curve\nsteps"); ax4.set_xlabel("time [s]")
    (rem_ln,) = ax4.plot([], [], "c", lw=1, label="remain")
    (brk_ln,) = ax4.plot([], [], "orange", lw=1, label="brake")
    ax4.legend(fontsize=6)
    for ax in (ax1, ax2, ax3, ax4):
        ax.grid(True, alpha=0.3)

    lines = (v_ln, al_ln, ar_ln, cm_ln, md_ln, rem_ln, brk_ln)
    dyn = (trail_ln, robot_pt, head_ln, txt, *lines)

    def snapshot(n=None):
        with lock:
            data = np.array(rows)
        if n is not None:
            data = data[:max(2, n)]
        if len(data) > args.maxpts:
            sel = np.unique(np.linspace(0, len(data) - 1, args.maxpts).astype(int))
            data = data[sel]
        return data

    def draw(data):
        t  = data[:, col["time_us"]] * 1e-6
        x  = data[:, col["x"]]; y = data[:, col["y"]]
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
        txt.set_text(f"t={t[-1]:7.3f}s\nv={v[-1]:+.3f} m/s\n"
                     f"theta={data[-1, col['theta_deg']]:8.1f} deg\n"
                     f"rows={len(rows)}")

    fig.suptitle(f"{'Live ' if follow else ''}replay: {' '.join(cmd)}", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.97))

    if follow:
        # Live-follow: axes grow with the data, no blit.
        def update(_):
            with lock:
                if not rows:
                    return dyn
            draw(snapshot())
            for ax in (axr, ax1, ax2, ax4):
                ax.relim(); ax.autoscale_view()
            if done.is_set():
                ani.event_source.stop()
            return dyn
        ani = FuncAnimation(fig, update, interval=1000 / args.fps,
                            blit=False, repeat=False, cache_frame_data=False)
        plt.show()
    else:
        # Buffered: fix axes from the full run, then reveal frame-by-frame (blit).
        full = snapshot()
        N = len(np.array(rows))
        t = full[:, col["time_us"]] * 1e-6
        xa, ya = full[:, col["x"]], full[:, col["y"]]
        px = 0.1 * max(np.ptp(xa), 1e-3); py = 0.1 * max(np.ptp(ya), 1e-3)
        axr.set_xlim(xa.min() - px, xa.max() + px)
        axr.set_ylim(ya.min() - py, ya.max() + py)
        va = vc._speed(0.5 * (full[:, col["leftSteps"]] + full[:, col["rightSteps"]]), t)
        for ax, lo, hi in [
            (ax1, va.min(), va.max()),
            (ax2, 0, max(full[:, col["currentAdrL"]].max(),
                         full[:, col["currentAdrR"]].max(), 1)),
            (ax4, min(full[:, col["remainDiff"]].min(), full[:, col["brakeDiff"]].min()),
                  max(full[:, col["remainDiff"]].max(), full[:, col["brakeDiff"]].max(), 1)),
        ]:
            m = 0.1 * (hi - lo + 1e-9)
            ax.set_xlim(t.min(), t.max()); ax.set_ylim(lo - m, hi + m)
        ax3.set_xlim(t.min(), t.max())

        frames = max(2, min(args.frames, N))

        def update(k):
            draw(snapshot(int(round((k + 1) / frames * N))))
            return dyn

        ani = FuncAnimation(fig, update, frames=frames, interval=1000 / args.fps,
                            blit=True, repeat=False, cache_frame_data=False)
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
