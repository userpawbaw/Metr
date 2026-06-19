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
extern float curveAccumErr;   /* 각도->스텝 변환 누적 소수부 오차(main.c) */

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

/* 사용자가 보고한 대칭 케이스: MoveVP(360,360); wait; MoveVP(0,0) -> 가속 후 감속을
 * 기대. 순항 도달 시점의 동작을 관찰하기 위한 디버그 시나리오. */
static void scenario_vptest(void)
{
    unsigned long g = 0;
    fprintf(stderr, "[sim] scenario: vptest  MoveVP(360,360) -> hold -> MoveVP(0,0)\n");

    MoveVP(360, 360);
    /* 순항 도달(=motionDone) 까지 대기 */
    while (!motionDone) { sim_tick(); if (++g >= g_max_ticks) break; }
    fprintf(stderr, "[sim] reached cruise: currentAdrR=%d changeAdrR=%d VP_ON=%d "
                    "(t=%.0fms, rightSteps=%ld)\n",
            currentAdrR, changeAdrR, VP_ON, sim_time_us/1000.0, sim_rightSteps);

    /* 순항을 유지해야 하는 구간: 추가로 1초(100k tick) 돌리며 스텝이 계속 나오는지 확인 */
    {
        long rs0 = sim_rightSteps; int k;
        for (k = 0; k < 100000; k++) sim_tick();
        fprintf(stderr, "[sim] after +1s hold: dRight=%ld  (0이면 순항이 멈춰버린 것=버그)"
                        "  VP_ON=%d motorRun_R=%d\n",
                sim_rightSteps - rs0, VP_ON, motorRun_R);
    }

    MoveVP(0, 0);
    g = 0;
    while (!motionDone) { sim_tick(); if (++g >= g_max_ticks) break; }
    fprintf(stderr, "[sim] after MoveVP(0,0): currentAdrR=%d rightSteps=%ld\n",
            currentAdrR, sim_rightSteps);
}

/* 측정 헬퍼: win tick 동안 양 바퀴 스텝 증가량을 step/s 환산해 출력. */
static void measure(const char *tag, int win)
{
    long l0 = sim_leftSteps, r0 = sim_rightSteps; int k;
    for (k = 0; k < win; k++) sim_tick();
    fprintf(stderr, "[sim]   %-22s L=%4ld R=%4ld steps/%dtick  (L~%.0f R~%.0f step/s)\n",
            tag, sim_leftSteps - l0, sim_rightSteps - r0, win,
            (sim_leftSteps - l0) * 1e6 / (win * (double)SIM_TICK_US),
            (sim_rightSteps - r0) * 1e6 / (win * (double)SIM_TICK_US));
}

/* 팀 요청 구조 유지 버전 검증: MoveVP -> MoveR 단독 -> MoveVP -> 정지.
 * 기대: 순항 안 끊김 / MoveR 시 L은 360 유지·R만 720 / 이후 둘 다 240으로 감속 / 정지. */
static void scenario_handoff(void)
{
    unsigned long g = 0;
    fprintf(stderr, "[sim] scenario: handoff  VP(360,360)->R(720)->VP(240,240)->VP(0,0)\n");

    MoveVP(360, 360);
    while (!motionDone) { sim_tick(); if (++g >= g_max_ticks) break; }
    fprintf(stderr, "[sim] reached cruise (currentAdrL=%d R=%d)\n", currentAdrL, currentAdrR);
    measure("cruise hold (both 360)", 100000);   /* 끊기면 R=0 */

    MoveR(720);
    measure("after MoveR(720)",       100000);   /* L 360 유지, R 720 기대 */

    MoveVP(240, 240);
    g = 0; while (!motionDone) { sim_tick(); if (++g >= g_max_ticks) break; }
    measure("after VP(240,240) done", 100000);   /* 둘 다 240 기대 */

    MoveVP(0, 0);
    g = 0; while (!motionDone) { sim_tick(); if (++g >= g_max_ticks) break; }
    measure("after VP(0,0) (stop)",   100000);   /* 둘 다 0 기대 */
}

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

/* 한 회전 세그먼트 실행: 새 API(MoveCurveRatio(); while(!motionDone);)를 그대로 사용.
 * curveMode 종료/정지는 ISR이 자체 처리하므로 호출측은 done만 기다리면 됨. */
static void run_curve_segment(float angle, int num, int den, float vmax)
{
    unsigned long guard = 0;
    long pl0 = sim_leftSteps, pr0 = sim_rightSteps;   /* 이 세그먼트의 물리 step 기준점 */

    MoveCurveRatio(angle, num, den, vmax);
    fprintf(stderr, "[sim] segment angle=%.0f vmax=%.0f  isOuterRight=%d "
                    "changeStepDiff=%d theta0=%.2f\n",
            angle, vmax, isOuterRight, changeStepDiff,
            sim_theta * 180.0 / SIM_PI);

    while (!motionDone) {                /* ISR이 curveMode=0 + motionDone=1로 종료 */
        sim_tick();
        if (debugAdrL || debugAdrR) {
            debugAdrL = 0;
            debugAdrR = 0;
        }
        if (++guard >= g_max_ticks) {
            fprintf(stderr,
                "[sim] TIMEOUT: segment not done after %lu ticks (%.1f ms). "
                "currentAdrO=%d changeAdrO=%d remainDiff=%d brakeDiff=%d\n",
                guard, guard * SIM_TICK_US / 1000.0,
                currentAdrO, changeAdrO, remainDiff, brakeDiff);
            return;
        }
    }

    fprintf(stderr, "[sim]   -> done: theta=%.2f deg  curveMode=%d  "
                    "fw stepDiff=%d/target%d  physical stepDiff=%ld  curveAccumErr=%.3f\n",
            sim_theta * 180.0 / SIM_PI, curveMode,
            currentStepDiff, changeStepDiff,
            (sim_rightSteps - pr0) - (sim_leftSteps - pl0),
            curveAccumErr);
}

/* The curve / Bresenham test that was commented out in main(). */
static void scenario_curve(void)
{
    fprintf(stderr, "[sim] scenario: curve\n");

    run_curve_segment(30, 2, 1, 300);

    /* 종료가 "루프가 멈춰서"가 아니라 "펌웨어가 모터를 세워서"임을 확인:
     * ISR이 curveMode를 내리고 outer 정지 가드가 동작하면 추가 tick에도 step이 안 늘어야 함. */
    {
        long ls0 = sim_leftSteps, rs0 = sim_rightSteps;
        int  k;
        for (k = 0; k < 20000; k++) sim_tick();
        fprintf(stderr,
            "[sim] post-stop check: dLeft=%ld dRight=%ld (0,0 이면 정지 성공)\n",
            sim_leftSteps - ls0, sim_rightSteps - rs0);
    }
}

/* 디버그용: 후진 먼저 -> 전진. "후진 이후 전진이 안된다" 재현/확인용. */
static void scenario_revfwd(void)
{
    fprintf(stderr, "[sim] scenario: revfwd (rev+30 then fwd+30)\n");
    run_curve_segment( 30, 2, 1, -300);   /* 후진 +30 */
    run_curve_segment( 30, 2, 1,  300);   /* 전진 +30 */
    fprintf(stderr, "[sim] revfwd final theta=%.2f deg\n",
            sim_theta * 180.0 / SIM_PI);
}

/* 직각주차: 전진 +30 / 후진 +30 / 전진 +30 = 누적 +90deg 헤딩 회전.
 * 같은 +angle을 vmax 부호(전진/후진)만 바꿔 호출 -> ISR이 outer 바퀴를 자동으로 뒤집어
 * 세 세그먼트 모두 같은 방향(+) 회전이 누적되어 직각으로 들어감. */
static void scenario_parking(void)
{
    fprintf(stderr, "[sim] scenario: parking (perpendicular, 3-move 30+30+30)\n");

    run_curve_segment( 30, 2, 1,  300);   /* 전진 +30 */
    run_curve_segment( 30, 2, 1, -300);   /* 후진 +30 */
    run_curve_segment( 30, 2, 1,  300);   /* 전진 +30 */

    fprintf(stderr, "[sim] parking final theta=%.2f deg (target ~90)\n",
            sim_theta * 180.0 / SIM_PI);
}

/* 사용자 실측 시퀀스 재현: 각 세그먼트 사이 0.3*1e5 tick(300ms) 정지 포함. */
static void scenario_usertraj(void)
{
    fprintf(stderr, "[sim] scenario: usertraj (user-supplied curve sequence)\n");

    run_curve_segment(90, 2, 1,  720); WaitTFlagCnt((unsigned int)(1e5 * 0.3));
    run_curve_segment(60, 3, 2, -720); WaitTFlagCnt((unsigned int)(1e5 * 0.3));
    run_curve_segment(60, 3, 2,  720); WaitTFlagCnt((unsigned int)(1e5 * 0.3));
    run_curve_segment(60, 3, 2, -720); WaitTFlagCnt((unsigned int)(1e5 * 0.3));
    run_curve_segment(45, 7, 5,  720); WaitTFlagCnt((unsigned int)(1e5 * 0.3));
    run_curve_segment(45, 7, 5,  720); WaitTFlagCnt((unsigned int)(1e5 * 0.3));

    fprintf(stderr, "[sim] usertraj final: theta=%.2f deg  x=%.4f y=%.4f\n",
            sim_theta * 180.0 / SIM_PI, sim_x, sim_y);
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

    if      (!strcmp(scenario, "movevp"))  scenario_movevp();
    else if (!strcmp(scenario, "curve"))   scenario_curve();
    else if (!strcmp(scenario, "parking")) scenario_parking();
    else if (!strcmp(scenario, "revfwd"))  scenario_revfwd();
    else if (!strcmp(scenario, "vptest"))  scenario_vptest();
    else if (!strcmp(scenario, "handoff")) scenario_handoff();
    else if (!strcmp(scenario, "usertraj")) scenario_usertraj();
    else {
        fprintf(stderr, "[sim] unknown scenario '%s' (use movevp|curve|parking)\n",
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
