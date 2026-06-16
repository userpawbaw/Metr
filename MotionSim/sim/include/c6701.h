/* c6701.h - PC simulator mock of the C6701 board register map.
 *
 * The real header maps peripherals to absolute bus addresses. Here every
 * register is redirected into two backing arrays (sim_emif / sim_fpga) defined
 * in sim_hw.c, so `*REG = x` / `x = *REG` behave as harmless RAM accesses.
 *
 * The motion of the virtual robot is NOT derived from these registers; the
 * virtual encoder watches phaseAdrL/phaseAdrR (see sim_hw.c). These mocks only
 * keep the original code compiling and running.
 */
#ifndef C6701_SIM_H
#define C6701_SIM_H

extern volatile unsigned int sim_emif[0x40];   /* EMIF + timer space  */
extern volatile unsigned int sim_fpga[0x100];  /* FPGA (CE2) space     */

/* ---- EMIF (External Memory Interface) ---- */
#define GBLCTL              (&sim_emif[0x00])
#define CECTL0              (&sim_emif[0x02])
#define CECTL1              (&sim_emif[0x01])
#define CECTL2              (&sim_emif[0x04])
#define CECTL3              (&sim_emif[0x05])
#define SDCTL               (&sim_emif[0x06])
#define SDTIM               (&sim_emif[0x07])

/* ---- TIMER0 / TIMER1 ---- */
#define T0CTL               (&sim_emif[0x10])
#define T0PRD               (&sim_emif[0x11])
#define T0CNT               (&sim_emif[0x12])
#define T1CTL               (&sim_emif[0x18])
#define T1PRD               (&sim_emif[0x19])
#define T1CNT               (&sim_emif[0x1A])

/* ---- DSP => FPGA (CE2) ---- */
#define FPGABASE            0x02000000
#define ADDR_SHIFT(A)       ((volatile unsigned int*)(&sim_fpga[(A) & 0xFF]))

/* UARTA */
#define txrx_divl           ADDR_SHIFT(0x000)
#define inten_divh          ADDR_SHIFT(0x001)
#define intid_fifo          ADDR_SHIFT(0x002)
#define linecon             ADDR_SHIFT(0x003)
#define modemcon            ADDR_SHIFT(0x004)
#define linestatus          ADDR_SHIFT(0x005)
#define modemstatus         ADDR_SHIFT(0x006)
#define scratch             ADDR_SHIFT(0x007)

#define DOUT0               ADDR_SHIFT(0x010)
#define DOUT1               ADDR_SHIFT(0x011)
#define DOUT2               ADDR_SHIFT(0x012)

#define FPGALED             ADDR_SHIFT(0x013)

#define DC_DRV_EN           ADDR_SHIFT(0x020)
#define DC_PWM_R            ADDR_SHIFT(0x021)
#define DC_PWM_L            ADDR_SHIFT(0x022)
#define DC_ENCCNT_CLR       ADDR_SHIFT(0x023)
#define DC_ENCCNT_MODE      ADDR_SHIFT(0x024)

#define DC_ENCST_R          ADDR_SHIFT(0x028)
#define DC_ENCST_L          ADDR_SHIFT(0x029)
#define DC_ENC_POSCNT_R     ADDR_SHIFT(0x02A)
#define DC_ENC_POSCNT_L     ADDR_SHIFT(0x02B)
#define DC_ENC_VELCNT_R     ADDR_SHIFT(0x02C)
#define DC_ENC_VELCNT_L     ADDR_SHIFT(0x02D)

#define STEP_PHASE_R        ADDR_SHIFT(0x030)
#define STEP_PHASE_L        ADDR_SHIFT(0x031)

#define BLDC_DRV_MODE       ADDR_SHIFT(0x040)
#define BLDC_DRV_EN         ADDR_SHIFT(0x041)
#define BLDC_PWM0           ADDR_SHIFT(0x042)
#define BLDC_PWM1           ADDR_SHIFT(0x043)
#define BLDC_PWM2           ADDR_SHIFT(0x044)
#define BLDC_HALLCNT_CLR    ADDR_SHIFT(0x045)
#define BLDC_DRIVE          ADDR_SHIFT(0x046)

#define BLDC_HALLST         ADDR_SHIFT(0x048)
#define BLDC_POSCNT         ADDR_SHIFT(0x049)
#define BLDC_VELCNT         ADDR_SHIFT(0x04A)

#define MUX_SEL             ADDR_SHIFT(0x050)
#define ADCIN               ADDR_SHIFT(0x051)

#define DAC0                ADDR_SHIFT(0x058)
#define DAC1                ADDR_SHIFT(0x059)
#define DAC2                ADDR_SHIFT(0x05A)
#define DAC3                ADDR_SHIFT(0x05B)
#define DAC4                ADDR_SHIFT(0x05C)
#define DAC5                ADDR_SHIFT(0x05D)
#define DAC6                ADDR_SHIFT(0x05E)
#define DAC7                ADDR_SHIFT(0x05F)

#define FPGAVER             ADDR_SHIFT(0x0FF)

#endif /* C6701_SIM_H */
