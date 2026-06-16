/* sim_main.c - Digital-twin driver (cooperative single-thread model).
 *
 * On the board, main() issues motion commands and busy-waits on flags that the
 * 100 kHz timer ISR raises in the background. On PC there is no real interrupt,
 * so every blocking primitive below is re-expressed as "advance the simulation
 * by N ISR ticks" via sim_tick(). The DSP motion logic in main.c / interrupt.c
 * is otherwise compiled and executed unchanged.
 *
 * Scenarios are selectable on the command line:
 *     ./motion_sim movevp   [options]   (the active MoveVP test sequence)
 *     ./motion_sim curve    [options]   (the previously commented-out curve test)
 *
 * Options:
 *     --out FILE      CSV output path        (default motion_log.csv)
 *     --decim N       log every Nth tick     (default 200; events always logged)
 *     --quiet         suppress MACRO_PRINT debug traces
 *     --max-ms M      safety cap per wait     (default 60000 ms of sim time)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "constant.h"
#include "typedef.h"
#include "bss.h"
#include "c6701.h"
#include "function.h"
#include "USBMon.h"
#include "sim_hw.h"

/* ---- Motion API implemented in main.c (no prototypes in function.h) ---- */
extern void MakeVP(float accel);
extern void MoveVP(float changeVel_L, float changeVel_R);
extern void MoveL(float spdL);
extern void MoveR(float spdR);
extern void MoveCurveRatio(float angle, int num, int den, float vmax);

/* Safety cap so a never-completing motion (a real bug worth catching) cannot
 * hang the simulator. Configurable via --max-ms. */
static unsigned long g_max_ticks = (60000UL * 1000UL) / SIM_TICK_US;

/* DBGV2 is a main.c-local macro; mirror it here for the scenario traces. */
#define DBGV2(a,b) \
    MACRO_PRINT((tmp_string, #a "=%d " #b "=%d\r\n", (a), (b)))

/* ======================================================================== */
/* Cooperative replacements for the board's blocking primitives.            */
/* These satisfy the extern declarations in function.h (the originals in     */
/* main.c are compiled out under -DSIM_BUILD).                               */
/* ======================================================================== */

void delay_us(unsigned int time_us)
{
    unsigned int n = time_us / SIM_TICK_US;
    if (time_us % SIM_TICK_US) n++;
    while (n--) sim_tick();
}

void delay_ms(unsigned int time_ms)
{
    delay_us(time_ms * 1000u);
}

void WaitTFlag(void)
{
    sim_tick();        /* one ISR period; the ISR sets TFlag = 1 */
    TFlag = 0;
}

void WaitTFlagCnt(unsigned int cnt)
{
    unsigned int i;
    TFlag = 0;
    for (i = 0; i < cnt; i++) WaitTFlag();
}

/* Same signature (and deliberate pass-by-value parameter shadowing) as the
 * board's debugging helper, so the snapshot behaviour is faithfully reproduced.
 * Pumps ticks until the ISR reports motionDone, with a safety cap. */
void WaitMotionDone(int currentAdrR, int changeAdrR)
{
    unsigned long guard = 0;
    (void)currentAdrR; (void)changeAdrR;   /* shadows the globals, as on board */

    motionDone = 0;
    while (!motionDone) {
        sim_tick();
        if (debugAdrL || debugAdrR) {      /* original drains the debug flags  */
            debugAdrL = 0;
            debugAdrR = 0;
        }
        if (++guard >= g_max_ticks) {
            fprintf(stderr,
                "[sim] TIMEOUT: motionDone not reached after %lu ticks "
                "(%.1f ms). currentAdrL=%d changeAdrL=%d currentAdrR=%d "
                "changeAdrR=%d curveMode=%d\n",
                guard, guard * SIM_TICK_US / 1000.0,
                currentAdrL, changeAdrL, currentAdrR, changeAdrR, curveMode);
            break;
        }
    }
    motionDone = 0;
}

/* ======================================================================== */
/* Scenarios                                                                 */
/* ======================================================================== */

/* The "Test 03 : MoveVP" sequence active in the uploaded main(). */
static void scenario_movevp(void)
{
    fprintf(stderr, "[sim] scenario: movevp\n");

    MoveVP(1080, 360);
    DBGV2(VP_ON, curveMode);
    WaitMotionDone(currentAdrR, changeAdrR);
    WaitTFlagCnt((unsigned int)(1e5 * 2));

    MoveVP(180, 360);
    WaitTFlagCnt((unsigned int)(1e5 * 2));

    MoveVP(180, -1080);
    WaitMotionDone(currentAdrR, changeAdrR);
    WaitTFlagCnt((unsigned int)(1e5 * 2));

    MoveVP(0, 0);
    WaitTFlagCnt((unsigned int)(1e5 * 2));

    MoveVP(180, -1080);
    WaitMotionDone(currentAdrR, changeAdrR);
    MoveVP(0, 0);
    WaitMotionDone(currentAdrR, changeAdrR);
}

/* The curve / Bresenham test that was commented out in main(). */
static void scenario_curve(void)
{
    unsigned long guard = 0;

    fprintf(stderr, "[sim] scenario: curve\n");

    MoveCurveRatio(30, 2, 1, 300);
    DBGV2(changeStepDiff, isOuterRight);
    DBGV2(curveMode, changeAdrO);

    motionDone = 0;
    while (!motionDone) {
        sim_tick();
        if (debugAdrL || debugAdrR) {
            DBGV2(currentStepDiff, remainDiff);
            DBGV2(stepCountL, stepCountR);
            debugAdrL = 0;
            debugAdrR = 0;
        }
        if (++guard >= g_max_ticks) {
            fprintf(stderr,
                "[sim] TIMEOUT: curve motionDone not reached after %lu ticks "
                "(%.1f ms). currentAdrO=%d changeAdrO=%d remainDiff=%d "
                "brakeDiff=%d\n",
                guard, guard * SIM_TICK_US / 1000.0,
                currentAdrO, changeAdrO, remainDiff, brakeDiff);
            break;
        }
    }
    motionDone = 0;
    curveMode = 0;   /* 회전 완료 후 ISR을 curve 분기에서 빼냄(정지 확정) */

    /* 종료가 "루프가 멈춰서"가 아니라 "펌웨어가 모터를 세워서"임을 확인:
     * curveMode=0 + outer 정지 가드가 동작하면 추가 tick에도 step이 안 늘어야 함. */
    {
        long ls0 = sim_leftSteps, rs0 = sim_rightSteps;
        int  k;
        for (k = 0; k < 20000; k++) sim_tick();
        fprintf(stderr,
            "[sim] post-stop check: dLeft=%ld dRight=%ld (0,0 이면 정지 성공)\n",
            sim_leftSteps - ls0, sim_rightSteps - rs0);
    }
}

/* ======================================================================== */
/* main                                                                      */
/* ======================================================================== */

static void register_watches(void)
{
    sim_watch_int("curveMode",   &curveMode);
    sim_watch_int("VP_ON",       &VP_ON);
    sim_watch_int("motionDone",  &motionDone);
    sim_watch_int("currentAdrR", &currentAdrR);
    sim_watch_int("changeAdrR",  &changeAdrR);
    sim_watch_int("currentAdrL", &currentAdrL);
    sim_watch_int("changeAdrL",  &changeAdrL);
}

int main(int argc, char **argv)
{
    const char *scenario = "movevp";
    const char *outpath  = "motion_log.csv";
    int   decim   = 200;
    int   i;

    for (i = 1; i < argc; i++) {
        if      (argv[i][0] != '-')              scenario = argv[i];
        else if (!strcmp(argv[i], "--out")    && i + 1 < argc) outpath = argv[++i];
        else if (!strcmp(argv[i], "--decim")  && i + 1 < argc) decim   = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--quiet"))    sim_verbose = 0;
        else if (!strcmp(argv[i], "--stream"))   sim_stream  = 1;
        else if (!strcmp(argv[i], "--realtime")) sim_realtime = 1;
        else if (!strcmp(argv[i], "--speed")  && i + 1 < argc)
            sim_speed = atof(argv[++i]);
        else if (!strcmp(argv[i], "--max-ms") && i + 1 < argc)
            g_max_ticks = (unsigned long)atol(argv[++i]) * 1000UL / SIM_TICK_US;
        else {
            fprintf(stderr, "[sim] unknown option '%s'\n", argv[i]);
            return 2;
        }
    }

    /* In stream mode stdout carries pure CSV; status text goes to stderr. */
    FILE *info = sim_stream ? stderr : stdout;
    if (sim_stream) sim_verbose = 0;

    fprintf(info, "[sim] C6701 motion digital-twin  (tick=%d us, decim=%d)\n",
            SIM_TICK_US, decim);

    /* Board-like initialisation that is meaningful on PC. */
    InitUART();              /* stub */
    MakeVP(500);             /* build the velocity-profile table */
    *STEP_PHASE_L = phase[0];
    *STEP_PHASE_R = phase[0];

    register_watches();
    if (sim_stream) sim_open_log_stream(decim);
    else            sim_open_log(outpath, decim);

    if      (!strcmp(scenario, "movevp")) scenario_movevp();
    else if (!strcmp(scenario, "curve"))  scenario_curve();
    else {
        fprintf(stderr, "[sim] unknown scenario '%s' (use movevp|curve)\n",
                scenario);
        sim_close_log();
        return 2;
    }

    sim_close_log();

    fprintf(info, "[sim] done: %lu ticks (%.1f ms sim time)\n",
            sim_tick_count, sim_time_us / 1000.0);
    fprintf(info, "[sim] final: x=%.4f m  y=%.4f m  theta=%.2f deg  "
            "leftSteps=%ld rightSteps=%ld\n",
            sim_x, sim_y, sim_theta * 180.0 / SIM_PI,
            sim_leftSteps, sim_rightSteps);
    if (!sim_stream) fprintf(info, "[sim] log written to %s\n", outpath);
    return 0;
}
