/* typedef.h - PC simulator reconstruction of the packet structures.
 *
 * The original typedef.h was not provided. These definitions are reconstructed
 * from how COMPACKET/RESPACKET are accessed in serial.c / interrupt.c
 * (ISRextint6) so the code compiles. The serial path is stubbed and never
 * exercised by the motion simulator, so exact layout is not critical.
 */
#ifndef TYPEDEF_SIM_H
#define TYPEDEF_SIM_H

#include "constant.h"

typedef struct {
    char *wr_ptr;                       /* write cursor             */
    char *rd_ptr;                       /* read cursor              */
    char  packet[SIZE_OF_COMPACKET];    /* command ring buffer      */
} COMPACKET;

typedef struct {
    int   flag;                         /* response in-progress     */
    int   char_num;                     /* chars left to send       */
    int   char_to_send;                 /* index of next char       */
    char  string[SIZE_OF_RESPACKET];    /* response buffer          */
} RESPACKET;

#endif /* TYPEDEF_SIM_H */
