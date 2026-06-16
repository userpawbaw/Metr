/* Bss.h */

// Serial.c
extern COMPACKET compacket;
extern RESPACKET respacket;
extern char tmp_string[SIZE_OF_RESPACKET];

// Interrupt.c
extern unsigned int TFlag;
extern int cntL;
extern int cntR;
extern unsigned int phaseAdrL, phaseAdrR, phaseAdrO;
extern int motionDone;
extern int debugAdrL, debugAdrR;
// 디버그용
extern int remainDiff, brakeDiff;
extern int stepCountL, stepCountR;


// extern volatile int pulse_R;
// extern volatile int pulse_L;

// Main.c
extern int DirL;
extern int DirR;
extern unsigned int DelayCntL;
extern unsigned int DelayCntR;
extern unsigned int phase[4];
extern int currentAdrL, changeAdrL;
extern int currentAdrR, changeAdrR;
extern unsigned int VParray[3000];
// curveMode��
extern int DirO;
extern int currentAdrO, changeAdrO;
extern int curveMode, isOuterRight;
extern int ratio_num, ratio_den;
extern int curveAccum;
extern int changeStepDiff, currentStepDiff;
extern int stepCntRst;
