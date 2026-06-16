/* USBMon.h - PC simulator mock of the USB monitor / debug-print layer.
 *
 * On the board MACRO_PRINT formats into tmp_string and ships it over UART.
 * Here it formats into tmp_string and echoes to stdout, so the existing
 * DBGV2/DBGF2 debug traces become visible in the simulator console.
 * Set the global sim_verbose = 0 to silence them.
 */
#ifndef USBMON_SIM_H
#define USBMON_SIM_H

#include <stdio.h>

extern int sim_verbose;
extern char tmp_string[];

/* MACRO_PRINT((tmp_string, "fmt", args...)) -> sprintf then echo. */
#define MACRO_PRINT(x)                       \
    do {                                     \
        sprintf x;                           \
        if (sim_verbose) fputs(tmp_string, stdout); \
    } while (0)

extern void InitUSBMon(void);

#endif /* USBMON_SIM_H */
