/* c6x.h - PC simulator mock of TI C6x compiler intrinsics / control regs.
 *
 * On the real TMS320C6701 this header (shipped with the TI compiler) exposes
 * the CPU control registers (IER, CSR, IFR, ...) and the `interrupt` keyword.
 * For the digital-twin we only need them to exist as plain writable storage so
 * the original DSP source compiles and runs unchanged under gcc.
 */
#ifndef C6X_SIM_H
#define C6X_SIM_H

/* The `interrupt` function qualifier has no meaning on PC. */
#define interrupt

/* CPU control registers modelled as ordinary globals (see sim_hw.c).
 * Only the registers actually touched by the DSP source are provided. */
extern volatile unsigned int IER;   /* Interrupt Enable Register   */
extern volatile unsigned int IFR;   /* Interrupt Flag Register     */
extern volatile unsigned int CSR;   /* Control Status Register     */

#endif /* C6X_SIM_H */
