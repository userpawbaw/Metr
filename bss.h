/* Bss.h */

// Serial.c
extern COMPACKET compacket;
extern RESPACKET respacket;
extern char tmp_string[SIZE_OF_RESPACKET];

// Interrupt.c
extern unsigned int TFlag;
extern int cntL;
extern int cntR;
extern int motionDone;
extern int debugAdrL, debugAdrR;

// Main.c

	//Move()
	extern unsigned int phase[4];
	extern int DirL, DirR;
	extern unsigned int DelayCntL, DelayCntR;
	extern int motorRun_L, motorRun_R;

	//MakeVP()
	extern unsigned int VParray[3000];

	//MoveVP()
	extern unsigned int currentAdrL, changeAdrL;
	extern unsigned int currentAdrR, changeAdrR;

	// curveMode
	extern int DirO;
	extern int currentAdrO, changeAdrO;
	extern int curveMode, isOuterRight;
	extern int ratio_num, ratio_den;
	extern int curveAccum;
	extern int changeStepDiff, currentStepDiff;
	extern int stepCntRst;
