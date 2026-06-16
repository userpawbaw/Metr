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
./motion_sim curve                  # the previously commented-out curve test
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
- **`curve`** exposes a firmware bug: in the `curveMode` branch of
  `interrupt.c`, `currentDirO` is read (lines 257/293/299) but **never
  assigned** from `changeDirO`, so `phaseAdrO` never advances. The internal
  `currentStepDiff` still reaches its target and `motionDone` fires, but no
  physical step is emitted (`leftSteps = rightSteps = 0`, pose stays at the
  origin). Faithful reproduction — the DSP logic is intentionally not patched.

## Next phase

Python visualisation (`python/`): static plots from the CSV, then a real-time
matplotlib animation (2D trajectory + heading on the left; remain / speed /
curveMode / brake graphs on the right).
