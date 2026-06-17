# C6701 DSP Motion Simulator (Digital Twin)

PC simulation of the TI TMS320C6701 stepper-motion control firmware. The goal
is a *digital twin*: run the **unmodified** DSP ISR / motion logic on a PC,
mock the hardware, and reproduce the board's behaviour closely enough to find
ISR/main synchronisation bugs, `curveMode` issues, deceleration/overshoot
timing, `motionDone` problems, etc.

```
DSP C code ──► C simulator (repeats the ISR) ──► motion_log.csv ──► Python viz
```

## Layout

```
MotionSim/
├─ dsp/      Uploaded board source compiled by BOTH targets
│            main.c, interrupt.c, bss.h, constant.h, function.h, interrupt.h
├─ board/    Board-only originals (real-HW build): register map (c6701.h),
│            vectors/linker/hex command files, batch scripts, prebuilt objects
├─ sim/      The PC digital twin
│  ├─ include/   Mock headers: c6x.h, c6701.h, typedef.h, USBMon.h
│  ├─ sim_hw.c/.h   virtual hardware + encoder + CSV logger + watch debugger
│  ├─ sim_main.c    cooperative driver + selectable scenarios
│  ├─ serial_stub.c serial/USB stubs (not provided / not needed for motion)
│  └─ Makefile
└─ python/   (visualisation — next phase)
```

## How it works

**Cooperative single-thread model.** On the board, `main()` issues commands
(`MoveVP`, `MoveCurveRatio`) and busy-waits on flags raised by the 100 kHz
timer ISR. On PC there is no interrupt, so each blocking primitive is
re-expressed as "advance the simulation by N ISR ticks":

| Board primitive            | Simulator behaviour                     |
|----------------------------|-----------------------------------------|
| `WaitTFlag()`              | run `ISRtimer0()` once (1 tick = 10 µs) |
| `WaitTFlagCnt(n)`          | run `n` ticks                           |
| `delay_us` / `delay_ms`    | run the equivalent number of ticks      |
| `WaitMotionDone(...)`      | run ticks until ISR sets `motionDone`   |

These live in `sim_main.c` under `-DSIM_BUILD`; the originals in `main.c` are
compiled out by `#ifndef SIM_BUILD` guards (the only edits to the DSP source).
`interrupt.c` is compiled completely unchanged.

**Virtual encoder.** Each tick, the change in `phaseAdrL` / `phaseAdrR`
(`±1 mod 4`) is read as one stepper step per wheel and integrated with a
differential-drive model (`SIM_STEP_DIST`, `SIM_WHEEL_BASE`, mirroring the
constants in `main.c`) into `(x, y, theta)`. This decodes *physical* motor
motion, independent of the firmware's internal step counters.

**Hardware mocks.** `c6x.h` (IER/CSR/IFR, the `interrupt` keyword) and
`c6701.h` (FPGA/EMIF register map → backing arrays) are replaced by harmless
RAM. `MACRO_PRINT` is routed to stdout so existing `DBGV2`/`DBGF2` debug traces
are visible (silence with `--quiet`).

## Build & run

```sh
cd sim
make
./motion_sim movevp                 # the active "Test 03 : MoveVP" sequence
./motion_sim curve                  # single 30 deg ratio turn (Bresenham)
./motion_sim parking                # perpendicular parking: fwd+30 / rev+30 / fwd+30
./motion_sim movevp --quiet --decim 100 --out run.csv
```

Options: `--out FILE`, `--decim N` (log every Nth tick; state changes always
logged), `--quiet`, `--max-ms M` (safety cap so a never-completing motion is
reported as a TIMEOUT instead of hanging).

### CSV columns (`motion_log.csv`)

`tick, time_us, leftSteps, rightSteps, currentAdrL, currentAdrR, changeAdrL,
changeAdrR, DelayCntL, DelayCntR, curveMode, VP_ON, motionDone,
currentStepDiff, changeStepDiff, remainDiff, brakeDiff, x, y, theta_deg`

### Change-watch debugger

Registered variables print only when they change, e.g.:

```
[watch] curveMode        0 -> 1   (tick 1, t=10us)
[watch] currentAdrR      71 -> 70 (tick ...)
```

## Findings so far

- **`movevp`** reproduces real physical motion (wheel steps accumulate, pose
  evolves) — VP mode sets the wheel directions correctly.
- **`curve`** now turns ~30 deg at the requested wheel ratio and **stops**
  (`leftSteps:rightSteps = 36:72 = 1:2`, pose advances, then halts). Earlier it
  never stopped / never moved; fixed by a set of `curveMode` bugs:
  - `currentDirO` was read but never assigned (`phaseAdrO` never advanced) —
    now `currentDirO = changeDirO`.
  - No stop guard: `currentAdrO == 0` is `VParray[0]` (slowest speed), not a
    halt, and `curveMode` was never cleared, so the outer wheel crept forever.
    Added an outer pulse-freeze guard and ISR self-termination
    (`curveMode = 0` at the terminal condition, mirroring the `VP_ON` pattern).
  - Removed a premature `motionDone` that fired at vmax cruise.
- **`parking`** chains three turns — `fwd +30 / rev +30 / fwd +30` — into a
  ~90 deg perpendicular park. `isOuterRight` is derived from
  `sign(angle) == sign(DirO)`, so the same `+angle` with a reversed `vmax`
  flips the outer wheel and keeps the heading rotating the same way.

## Python visualisation (`python/`)

```sh
cd python
pip install -r requirements.txt      # numpy pandas matplotlib pillow

# static overview (trajectory + speed / Adr / flags / remain-brake)
python visualize.py ../sim/motion_log.csv
python visualize.py ../sim/motion_log.csv --save overview.png

# real-time replay (left: robot + heading + trail; right: live time series)
python animate.py ../sim/motion_log.csv               # live window
python animate.py ../sim/curve_log.csv --save run.gif # render GIF
#   --frames N  animation steps   --fps N   playback rate
#   --maxpts N  cap working rows   --dpi N   render DPI when saving
```

Geometry constants in `viz_common.py` mirror `dsp/main.c` so plotted distances
match the C side. Centre/wheel speed is derived from the logged step counts.

## One-shot workflow and live mode

The whole point: **edit the C, re-run, and the visualisation reflects it** —
Python only draws the CSV the C simulator produces.

```
dsp/*.c  ──build──►  motion_sim  ──CSV──►  Python  ──►  plot / animation
```

**`run.sh`** does build → run → visualise in a single command:

```sh
cd MotionSim
./run.sh curve static            # build, run, static plot
./run.sh movevp anim             # build, run, recorded animation
./run.sh curve live              # build, then pipe C straight into Python
./run.sh movevp anim --save out.gif   # extra args go to the Python script
```

Equivalent Makefile targets (in `sim/`): `make viz`, `make anim`, `make live`
(`SCN=curve make live` to pick the scenario).

### Windows

`run.sh` is a bash script and will **not** run in PowerShell. Two options:

- **WSL (recommended):** `wsl --install`, then inside Ubuntu
  `sudo apt install -y build-essential python3 python3-venv python3-pip`,
  `cd /mnt/c/Users/<you>/.../MotionSim`, install the Python deps, and use
  `./run.sh ...` exactly as above (GUI windows work via WSLg on Windows 11).
- **Native PowerShell:** install MSYS2/MinGW (`gcc`, `make`) + Python on PATH,
  then use the PowerShell port:

  ```powershell
  .\run.ps1 curve live
  .\run.ps1 movevp anim --save run.gif
  ```

  `--realtime` works under MinGW (winpthreads provides `clock_gettime`/
  `nanosleep`). If scripts are blocked:
  `powershell -ExecutionPolicy Bypass -File .\run.ps1 curve live`.

**Live mode** (`animate_live.py`) launches `motion_sim --stream`, reads its CSV
from stdout in a background thread, and animates it. By default the run is
buffered and played back frame-by-frame at `--fps` (smooth, blitted) — the C sim
fills the buffer almost instantly. Add `--realtime` to instead *follow* the C
sim paced to wall-clock time. No intermediate file is used:

```sh
./sim/motion_sim curve --stream | python python/animate_live.py   # (or your own tool)
```

After changing `dsp/interrupt.c` / `dsp/main.c`, just re-run any of the above;
no other steps are needed.

### True real-time playback

By default the C sim runs as fast as possible. `--realtime` paces the *emitted
rows* to wall-clock time using the simulated timestamp (`time_us`), so 1 s of
simulated motion takes 1 s of real time; `--speed X` scales that (x2 = twice as
fast). Pacing happens at the C producer, so any consumer of the stream sees
real-time data.

```sh
./sim/motion_sim curve --stream --realtime            # x1 real time
./sim/motion_sim curve --stream --realtime --speed 5  # 5x

# live window at real-time speed:
./run.sh curve live --realtime
./run.sh movevp live --realtime --speed 4
```

Smoothness tips:
- Increase `--frames` (default 400) and/or `--fps` for a more continuous replay;
  the buffered modes use blitting so this stays cheap.
- Without `--realtime`, live mode buffers then replays frame-by-frame — it will
  not "jump" through the data. Use `--realtime` only when you want true
  wall-clock following (its smoothness is then bounded by the row cadence).

Notes:
- Pacing is per logged row, so keep `--decim` small enough that rows are frequent
  (the default is fine); too-large a decimation makes the playback step coarsely.
- Real-time is ignored when rendering to a file (`--save`), which would otherwise
  block for the full run duration.
- Uses POSIX `clock_gettime`/`nanosleep` (Linux/macOS/WSL). On native Windows,
  run under WSL for `--realtime`.

The **curve** / **parking** plots show the Bresenham ratio profile, `remainDiff`
counting down and `brakeDiff` triggering deceleration, with the trajectory now
tracing the actual arc(s) and the pose halting at the target heading (see
*Findings* for the `curveMode` fixes).
