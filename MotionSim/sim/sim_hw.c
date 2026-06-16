/* sim_hw.c - Digital-twin runtime implementation.
 *
 * Hosts the mock hardware storage, the CPU control-register globals, the
 * virtual differential-drive encoder, the CSV logger and the change-watch
 * debugger. sim_tick() is the heartbeat: it runs the unmodified DSP ISR once
 * and translates its effect on the stepper phases into robot motion.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "constant.h"
#include "typedef.h"
#include "bss.h"
#include "sim_hw.h"

/* The real ISR lives in interrupt.c; `interrupt` is a no-op macro on PC.
 * Declared here directly to avoid pulling in interrupt.h (which would
 * duplicate the empty INT04/05/08/09 handler definitions). */
extern void ISRtimer0(void);

/* ---- Mock hardware backing storage (referenced by c6701.h / c6x.h) ---- */
volatile unsigned int sim_emif[0x40];
volatile unsigned int sim_fpga[0x100];
volatile unsigned int IER;
volatile unsigned int IFR;
volatile unsigned int CSR;

/* ---- Simulation state ---- */
unsigned long sim_tick_count = 0;
double        sim_time_us     = 0.0;
long          sim_leftSteps   = 0;
long          sim_rightSteps  = 0;
double        sim_x = 0.0, sim_y = 0.0;
double        sim_theta = 0.0;
int           sim_verbose = 1;

/* ---- Virtual encoder bookkeeping ---- */
static unsigned int prevPhaseL = 0;
static unsigned int prevPhaseR = 0;
static int          enc_primed = 0;

/* Signed step from a phase-index change (sequence advances by +/-1 mod 4). */
static int phase_delta(unsigned int now, unsigned int prev)
{
    int d = (int)((now - prev) & 3u);   /* 0,1,2,3 */
    if (d == 1) return  1;
    if (d == 3) return -1;
    if (d == 2)                          /* two phases in one tick: unexpected */
        fprintf(stderr,
            "[sim] WARN tick %lu: phase jumped by 2 (prev=%u now=%u)\n",
            sim_tick_count, prev, now);
    return 0;
}

/* ---- CSV logger ---- */
static FILE *logfp      = NULL;
static int   logdecim   = 1;
/* previous values for event-triggered logging */
static int   pv_curveMode, pv_motionDone, pv_VP_ON, pv_cAdrL, pv_cAdrR;
static int   pv_valid = 0;

static void csv_header(void)
{
    fprintf(logfp,
        "tick,time_us,leftSteps,rightSteps,"
        "currentAdrL,currentAdrR,changeAdrL,changeAdrR,DelayCntL,DelayCntR,"
        "curveMode,VP_ON,motionDone,"
        "currentStepDiff,changeStepDiff,remainDiff,brakeDiff,"
        "x,y,theta_deg\n");
}

static void csv_row(void)
{
    fprintf(logfp,
        "%lu,%.1f,%ld,%ld,"
        "%d,%d,%d,%d,%u,%u,"
        "%d,%d,%d,"
        "%d,%d,%d,%d,"
        "%.6f,%.6f,%.3f\n",
        sim_tick_count, sim_time_us, sim_leftSteps, sim_rightSteps,
        currentAdrL, currentAdrR, changeAdrL, changeAdrR, DelayCntL, DelayCntR,
        curveMode, VP_ON, motionDone,
        currentStepDiff, changeStepDiff, remainDiff, brakeDiff,
        sim_x, sim_y, sim_theta * 180.0 / SIM_PI);
}

static int event_changed(void)
{
    int changed;
    if (!pv_valid) return 1;
    changed = (pv_curveMode  != curveMode)  ||
              (pv_motionDone != motionDone) ||
              (pv_VP_ON      != VP_ON)      ||
              (pv_cAdrL      != currentAdrL)||
              (pv_cAdrR      != currentAdrR);
    return changed;
}

static void event_snapshot(void)
{
    pv_curveMode  = curveMode;
    pv_motionDone = motionDone;
    pv_VP_ON      = VP_ON;
    pv_cAdrL      = currentAdrL;
    pv_cAdrR      = currentAdrR;
    pv_valid      = 1;
}

void sim_open_log(const char *path, int decimation)
{
    logfp = fopen(path, "w");
    if (!logfp) {
        fprintf(stderr, "[sim] ERROR: cannot open log '%s'\n", path);
        exit(1);
    }
    logdecim = (decimation > 0) ? decimation : 1;
    csv_header();
}

void sim_close_log(void)
{
    if (logfp) {
        csv_row();          /* always capture the final state */
        fclose(logfp);
        logfp = NULL;
    }
}

/* ---- Change-watch debugger ---- */
#define SIM_MAX_WATCH 32
typedef struct {
    const char    *name;
    int            is_uint;
    volatile int  *ip;
    volatile unsigned int *up;
    long           last;
} watch_t;
static watch_t watches[SIM_MAX_WATCH];
static int     nwatch = 0;

void sim_watch_int(const char *name, volatile int *var)
{
    if (nwatch >= SIM_MAX_WATCH) return;
    watches[nwatch].name = name;
    watches[nwatch].is_uint = 0;
    watches[nwatch].ip = var;
    watches[nwatch].last = *var;
    nwatch++;
}

void sim_watch_uint(const char *name, volatile unsigned int *var)
{
    if (nwatch >= SIM_MAX_WATCH) return;
    watches[nwatch].name = name;
    watches[nwatch].is_uint = 1;
    watches[nwatch].up = var;
    watches[nwatch].last = (long)*var;
    nwatch++;
}

static void watch_check(void)
{
    int i;
    for (i = 0; i < nwatch; i++) {
        long cur = watches[i].is_uint
                 ? (long)*watches[i].up
                 : (long)*watches[i].ip;
        if (cur != watches[i].last) {
            printf("[watch] %-16s %ld -> %ld   (tick %lu, t=%.0fus)\n",
                   watches[i].name, watches[i].last, cur,
                   sim_tick_count, sim_time_us);
            watches[i].last = cur;
        }
    }
}

/* ---- The heartbeat ---- */
void sim_tick(void)
{
    int dL, dR;
    double ds_l, ds_r, dCenter, dTheta;

    if (!enc_primed) {           /* align encoder with the initial phase */
        prevPhaseL = phaseAdrL;
        prevPhaseR = phaseAdrR;
        enc_primed = 1;
    }

    ISRtimer0();                 /* run the unmodified DSP ISR once */

    /* Virtual encoder: translate phase-index motion into wheel steps. */
    dL = phase_delta(phaseAdrL, prevPhaseL);
    dR = phase_delta(phaseAdrR, prevPhaseR);
    prevPhaseL = phaseAdrL;
    prevPhaseR = phaseAdrR;
    sim_leftSteps  += dL;
    sim_rightSteps += dR;

    /* Differential-drive integration (matches the spec in the brief). */
    ds_l    = dL * SIM_STEP_DIST;
    ds_r    = dR * SIM_STEP_DIST;
    dCenter = (ds_l + ds_r) * 0.5;
    dTheta  = (ds_r - ds_l) / SIM_WHEEL_BASE;
    sim_x     += dCenter * cos(sim_theta);
    sim_y     += dCenter * sin(sim_theta);
    sim_theta += dTheta;

    sim_tick_count++;
    sim_time_us += SIM_TICK_US;

    if (logfp && ((sim_tick_count % (unsigned long)logdecim == 0) ||
                  event_changed())) {
        csv_row();
        event_snapshot();
    }

    watch_check();
}
