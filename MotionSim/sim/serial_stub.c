/* serial_stub.c - PC simulator stubs for the serial / USB-monitor layer.
 *
 * serial.c and USBMon.c were not part of the digital-twin scope (no UART on
 * PC). These stubs satisfy the linker and own the shared buffers referenced by
 * bss.h. The motion ISR path never calls into the real serial routines.
 */
#include <stdio.h>
#include "constant.h"
#include "typedef.h"

/* Shared buffers declared extern in bss.h */
COMPACKET compacket;
RESPACKET respacket;
char      tmp_string[SIZE_OF_RESPACKET];

/* Serial API (declared in function.h) - no-ops on PC */
void InitUART(void)            { }
char ReceiveByte(void)         { return 0; }
void SendByte(char data)       { (void)data; }
int  CheckSum(void)            { return 0; }
void SendData(void)            { }
void Report(void)              { }

/* USB monitor init (declared in USBMon.h) */
void InitUSBMon(void)          { }
