/* sim_hw.h - Digital-twin runtime: virtual hardware, encoder, logging.
 *
 * Provides the cooperative single-thread driver used by the PC build:
 *   sim_tick()  - run ISRtimer0() once, update the virtual encoder/pose,
 *                 advance simulated time and emit a log row.
 * The DSP busy-wait primitives (WaitTFlag, WaitMotionDone, ...) are layered
 * on top of sim_tick() in sim_main.c.
 */
#ifndef SIM_HW_H
#define SIM_HW_H

/* ---- Robot / timing parameters (mirror main.c so step maths agree) ---- */
#ifndef SIM_PI
#define SIM_PI        3.141592
#endif
#define SIM_WHEEL_R    0.05                                   /* wheel radius [m]      */
#define SIM_WHEEL_BASE 0.11                                   /* track width [m]       */
#define SIM_STEP_DIST  (SIM_WHEEL_R * (1.8 / 180.0 * SIM_PI)) /* metres per 1.8deg step */

/* ISR period. TIMER_FRQ=100kHz on the board => 10 us per tick. */
#define SIM_TICK_US    10

/* ---- Simulation state (read by loggers / scenarios) ---- */
extern unsigned long sim_tick_count;   /* number of ISR ticks executed */
extern double        sim_time_us;      /* simulated time [us]          */
extern long          sim_leftSteps;    /* net signed left-wheel steps  */
extern long          sim_rightSteps;   /* net signed right-wheel steps */
extern double        sim_x, sim_y;     /* robot position [m]           */
extern double        sim_theta;        /* robot heading [rad]          */
extern int           sim_verbose;      /* echo MACRO_PRINT traces      */

/* ---- Lifecycle ---- */
void sim_open_log(const char *path, int decimation);
void sim_close_log(void);
void sim_tick(void);                   /* advance one ISR period       */

/* ---- Change-watch debugger: print "name: old -> new" on change ---- */
void sim_watch_int(const char *name, volatile int *var);
void sim_watch_uint(const char *name, volatile unsigned int *var);

#endif /* SIM_HW_H */
