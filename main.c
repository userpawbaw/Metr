/* Main.c */

#include <c6x.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "c6701.h"
#include "constant.h"
#include "typedef.h"
#include "bss.h"
#include "function.h"
#include "USBMon.h"

#define PI 3.141592
#define VPNUM 3000
#define EPSILON 0.000001
#define WHEEL_R 0.05 		// 바퀴의 반지름
#define WHEEL_BASE 0.11 		// 바퀴 사이의 거리
#define STEP_DIST (WHEEL_R * (1.8 / 180.0 * PI))  // 스텝당 이동거리 -> r * theta = r * 1.8/180*PI (rad 환산)  

#define DBGV2(a,b) \
    MACRO_PRINT((tmp_string, \
        #a "=%d " #b "=%d\r\n", \
        (a), (b)))

#define DBGF2(a,b) \
    MACRO_PRINT((tmp_string, \
        #a "=%f " #b "=%f\r\n", \
        (a), (b)))


void InitEXINTF()
{
	// Disable SSCEN(bit#5), CLK1EN(bit#4), CLK2EN(bit#3)
	*GBLCTL &= 0xFFFFFFC7;

	// IOCS0: Async 32bit Mode, Setup 0 and Strobe 3, Hold 1  
	*CECTL0 = 	(0x00 << 28)	|	// (4-bit) WriteSetup[31:28]
				(0x03 << 22)	|	// (6-bit) WriteStrobe[27:22]
				(0x01 << 20)	|	// (2-bit) WriteHold[21:20]
				(0x00 << 16)	|	// (4-bit) ReadSetup[19:16]
				(0x00 << 14)	|	// (2-bit) Rsvd[15:14]
				(0x03 <<  8)	|	// (6-bit) ReadStrobe[13:8]
				(0x00 <<  7)	|	// (1-bit) Rsvd[7]
				(0x02 <<  4)	|	// (3-bit) MType[6:4]
				(0x00 <<  2)	|	// (2-bit) Rsvd[3:2]
				(0x01 <<  0)	;	// (2-bit) ReadHold[1:0]

	// IOCS2: Async 32bit Mode, Setup 2 and Strobe 5, Hold 1
	*CECTL2 =	(0x02 << 28)	|	// (4-bit) WriteSetup[31:28]
				(0x08 << 22)	|	// (6-bit) WriteStrobe[27:22]
				(0x01 << 20)	|	// (2-bit) WriteHold[21:20]
				(0x02 << 16)	|	// (4-bit) ReadSetup[19:16]
				(0x00 << 14)	|	// (2-bit) Rsvd[15:14]
				(0x08 <<  8)	|	// (6-bit) ReadStrobe[13:8]
				(0x00 <<  7)	|	// (1-bit) Rsvd[7]
				(0x02 <<  4)	|	// (3-bit) MType[6:4]
				(0x00 <<  2)	|	// (2-bit) Rsvd[3:2]
				(0x01 <<  0)	;	// (2-bit) ReadHold[1:0]
}

void InitTimer()
{
	// Hold 0 and Go 0, Internal Clock Source (160Mhz/4), Clock Mode   
	*T0CTL |= 0x00000300;

	// Timer Period
	*T0PRD = (CPU_FRQ/4.0f)/(2.0f*TIMER_FRQ);	// f = 40Mhz/2*Period 

	// Hold 1 and Go 1   
	*T0CTL |= 0x000000C0;
}

void InitINT()
{
	// Enable CPU Interrupt INT06(EXTINT6), INT14(TINT0) and NMI
	IER |= 0x00004042;
}

void GIE()
{
	// Global Interrupt Enable
	CSR |= 0x00000001;
}

// Caution: The delayed time is not exact.
void delay_us(unsigned int time_us)
{
	register unsigned int i;

	for (i = 0; i < (time_us * 14); i++) ;
}

// Caution: The delayed time is not exact.
void delay_ms(unsigned int time_ms)
{
	register unsigned int i;

	for (i = 0; i < time_ms; i++) {
		delay_us(1000);
	}
}

// Wait until timer interrupt
void WaitTFlag()
{
	while (!TFlag) ;
	TFlag = 0;
}

// Waiting time = (Timer Period) * cnt
void WaitTFlagCnt(unsigned int cnt)
{
	unsigned int i;

	TFlag = 0;

	for (i=0; i<cnt; i++) {
		WaitTFlag();
	}
}


//좌우 스텝모터 구동 구현

	//전역변수 구현
	unsigned int phase[4] = {0x2, 0x8, 0x1, 0x4};
	int DirL;
	int DirR;
	unsigned int DelayCntL;
	unsigned int DelayCntR;

void MoveL(float spdL){

	//변수 설정
	DirL = 1;	//방향지정변수, 1이 전진, -1이 후진
	// interrupt에서 PhaseAdr++, --를 PhaseAdr += Dir로 대체하기 위해 +-1로 설정
	DelayCntL = 0;	//대기할 스텝수
	
	
	//interrupt.c로 값을 보냄
	DirL = (spdL >= 0) ? 1 : -1;  //속도입력에서 방향 계산 						
	DelayCntL = (int)(1.8f/abs(spdL) * 100000.0f);	//속도입력에서 필요 대기스텝 계산
}


void MoveR(float spdR){
	
	//변수 설정
	DirR = 0;	//방향지정변수, boolean 사용하는게 나을듯
	DelayCntR = 0;	//대기할 스텝수
	
	
	//interrupt.c로 값을 보냄
	DirR = (spdR >= 0) ? 1 : -1;
	DelayCntR = (int)(1.8f/abs(spdR) * 100000.0f);	//속도입력에서 필요 대기스텝 계산
}




//VParray 생성

unsigned int VParray[VPNUM];

void MakeVP(float accel){
	int i;
	 // 0 일때 정지. 가속 시작시 VParray는 1부터 시작해야 함. 방향전환시 1에서 전환해야 함.
	for(i = 0; i < VPNUM; i++){
		VParray[i]= sqrt(1.8f/(2*accel*(i+1)))*100000.0;
	}
} 


//VParray 사용
float currentVel_R, currentVel_L;

int currentAdrL = 0;
int changeAdrL = 0;
int currentAdrR = 0;
int changeAdrR = 0;
int currentAdrO = 0;
int changeAdrO = 0;
unsigned int changeDelayR, changeDelayL, changeDelayO;


void MoveVP(float changeVel_L, float changeVel_R){
	
	/* 	//지역변수 설정(main에서 pulse로 컨트롤하는 버젼)
	int Vel_R, Vel_L;
	int sign_R, sign_L;
	*/
	int i;
	DirL = (changeVel_L >= 0) ? 1 : -1;
	DirR = (changeVel_R >= 0) ? 1 : -1;

	changeDelayR = (int)(1.8f / abs(changeVel_R) * 100000.0f); // EPSILON: 0 나누기 문제 방지
	changeDelayL = (int)(1.8f / abs(changeVel_L) * 100000.0f); 
	MACRO_PRINT((tmp_string, "changeDelayR: %d	changeDelayL: %d\r\n", changeDelayR, changeDelayL));
	for(i = 0; i < VPNUM; i++){
		if (VParray[i] <= changeDelayL){
			changeAdrL = i; //목표 Adr 구함
			break;
		}
	}	
	for(i = 0; i < VPNUM; i++){
		if (VParray[i] <= changeDelayR){
			changeAdrR = i; //목표 Adr 구함
			break;
		}
	}
	
	MACRO_PRINT((tmp_string, "currentAdrR: %d	changeAdrR: %d\r\n ", currentAdrR, changeAdrR));
	MACRO_PRINT((tmp_string, "currentAdrL: %d	changeAdrL: %d\r\n", currentAdrL, changeAdrL));
	MACRO_PRINT((tmp_string, "DirL: %d DirR: %d	\r\n", DirL, DirR));
	
	
	/*
	
		while(1) {
			if(currentAdrR == changeAdrR & currentAdrL == changeAdrL){
				// MACRO_PRINT((tmp_string, " Done! "));
				break;
			}
			else{
				if(currentAdrR != changeAdrR){
					//MACRO_PRINT((tmp_string, "IN?"));
					if(pulseR){
						//MACRO_PRINT((tmp_string, "IN?"));
						//sign_R = ((changeAdrR - currentAdrR) > 0) ? 1 : -1;						
						pulseR = 0;
						// Vel_R = 1.8 / VParray[currentAdrR] * 100000.0;
						// MoveR(Vel_R);   	// 이전 moveR함수 그대로 이용하고자 했으나, 현재 구조에선 사용 x. 매번 1.8f/abs(spd)*100000 같은 float 나머지연산을 수행해야 하므로 비쌈.
											// 어차피 Adr 구했으니까 그냥 MoveVP()에서 DelayCntR 자체를 VParray[Adr]로 바꿔주는게 유리.
						DelayCntR = VParray[currentAdrR];
						if((changeAdrR - currentAdrR) > 0){
							currentAdrR++; 
						} else{
							currentAdrR--;
						}
						// changeAdrR - currentAdrR의 부호
						//MACRO_PRINT((tmp_string, "current_R: %d	Vel_R: %d\r\n", currentAdrR, Vel_R));
	
					}
				}
				if(currentAdrL != changeAdrL){
					if(pulseL){
						//sign_L = ((changeAdrL - currentAdrL) > 0 ) ? 1 : -1;
						pulseL = 0;
						// Vel_L = 1.8 / VParray[currentAdrL] * 100000.0;
						// MoveL(Vel_L);
						DelayCntR = VParray[currentAdrR];
						if((changeAdrL - currentAdrL) > 0){
							currentAdrL++; 
						} else{
							currentAdrL--;
						}
						// changeAdrR - currentAdrR의 부호
						//MACRO_PRINT((tmp_string, "current_L: %d	Vel_L: %d\r\n", currentAdrL, Vel_L));
					}
				}
			}
		}
		//속도 고정
		//MoveL(changeVel);
		//MoveR(changeVel);
	*/
}

void Rotate(float angle, float vmax){
	
}

// curveMode Adr 처리용
int DirO;

void CurveVP(float changeVel_O){ // MoveVP와 로직 동일. changeAdr만 찾아줌 

	int i;
	DirO = (changeVel_O >= 0) ? 1 : -1;

	changeDelayO = (int)(1.8f / abs(changeVel_O) * 100000.0f); // EPSILON: 0 나누기 문제 방지
	MACRO_PRINT((tmp_string, "changeDelayO: %d\r\n", changeDelayO));
	for(i = 0; i < VPNUM; i++){
		if (VParray[i] <= changeDelayO){
			changeAdrO = i; //목표 Adr 구함
			break;
		}
	}
}	

// curveMode 처리용 전역변수(interrupt에서 사용 예정)
int curveMode, isOuterRight;
int ratio_num, ratio_den;	// 바퀴의 속도비
int curveAccum;				// bresenham 누적 알고리즘용 
int changeStepDiff;			// interrupt에 보내줄 목표step차
int currentStepDiff; 		// interrupt에서 매번 갱신할 현재Step차
int stepCntRst;				// 양 바퀴 step차 계산용 stepCnt 초기화신호

void ResetStepCount(){
	stepCntRst = 1;
}


int AngleToDiffStep(float angle){ 	
// 헤딩 회전 각도를 받아서 그 각도로 향하기 위한 바퀴간의 스텝 차이로 환산해주는 함수
// 용도: AngleToDiffStep(90.0) => 110 -> stepCountR-stepCountL = 110이어야 90도.
	float theta;
	float diffDist;
	int targetDiffStep;
	
	
	theta = PI * angle / 180.0f; // rad 단위
	
	// 헤딩이 +90도 회전 -> 오른쪽 바퀴가 더 움직임 
	// 오른쪽이 더 이동할 거리: 
	// 왼쪽 바퀴 정지시 -> 왼쪽은 이동거리 0, 오른쪽은 r*theta
	// => 이동거리 = 양 바퀴 거리 * 회전할 각도 = WHEEL_BASE * PI/2.
	
	// diffDist = 오른쪽 이동거리 - 왼쪽 이동거리. 양수면 +, 음수면 -방향 회전.
	
	// 왼쪽 바퀴가 움직이는 경우엔?
	// 같은 스텝만큼 움직이면 회전 x. 즉 동일 스텝차 -> 동일 각도 회전. 
	// => 느린바퀴 이동과 무관하게 회전각도는 위의 식 그대로 사용하면 됨.
	
	// 최종적으로 리턴할 스텝차이 targetDiffStep = 이동거리 차/step당 거리 =  diffDist / STEP_DIST.
	
	diffDist = WHEEL_BASE * theta;
	DBGF2(diffDist, STEP_DIST);
	DBGF2(diffDist / STEP_DIST, theta / PI);
	targetDiffStep = (int)(diffDist / STEP_DIST + 0.5f);  // +0.5 -> 반올림 처리 가능
	
	return targetDiffStep;
}

void MoveCurveRatio(float angle, int num, int den, float vmax){
	// 위의 함수들을 이용해 실제 자동차처럼 회전하기 위한 정보를 넘겨주는 함수.

	// MoveVP()는 구현상 양 바퀴마다 독립적으로 Adr이 관리되므로, 가속중에 양 바퀴의 가속도가 같다는 문제가 있었음.
	// MoveVP(180, 360)는 최대 속도에 도달하면 1:2의 속도비로 움직이나, 180에 도달하기 전까진 양 바퀴가 같은 가속도로 증가하는 문제.
	// 즉 MoveVP만으로는 회전 구현시 가속 직진하다가 마지막에 급격히 방향을 트는 식으로만 구현 가능.
	// 따라서, 부드러운 회전을 구현하기 위해선 양 바퀴의 가속도도 ratio 비율대로 설정되어야 함. 
	// 하나의 VParray로 다른 가속도를 구현하는 방법: 
	// 기존: 상변환 신호마다 Adr++,--하는 것에서, 상변환 신호 2번이 들어와야 Adr++, or 한번 들어올때 Adr=Adr+2같은 방식으로 설정시
	// 어레이 하나로 여러 가속도 구현 가능.   
	// MoveVP와 유사하게 상 변환은 interrupt에서. 목표 각도차, 최대 Adr만 넘겨주고 함수가 끝나는 방식. 
	
	// 사용법: MoveCurveRatio(...); while(~curveDone); (끝났단 신호는 interrupt가 알려줌) 
	// MoveVP(); etc....


	ResetStepCount(); // 현재 헤딩 기준으로 목표 회전각도만큼 회전하기 위해 초기화
	curveMode = 1;
	changeStepDiff = AngleToDiffStep(angle);
	isOuterRight = (changeStepDiff > 0) ? 1 : 0;
	ratio_num = num; 
	ratio_den = den;
	curveAccum = 0; 	// 초기화
	CurveVP(vmax); 		// 회전중 최대속도에 해당하는 VParray Adr 계산해 전달 
	phaseAdrO = (isOuterRight) ? phaseAdrR : phaseAdrL; // 커브모드 진입 1회시 초기화
	DBGV2(curveMode, changeAdrO);
}



void WaitMotionDone(int currentAdrR, int changeAdrR) 
{ // 디버깅용 함수. motionDone이 ISR에서 들어올때까지 출력함. 
// Done이 들어와 끝나면 motionDone 내리고 종료.
	motionDone = 0; // 최초 0 초기화.
    while(!motionDone){
        if(debugAdrL || debugAdrR){
            DBGV2(currentAdrR, changeAdrR);
            debugAdrL = 0;
            debugAdrR = 0;
        }
    }
    motionDone = 0;
}


void main()
{
	int delay = 1000;
	
	InitEXINTF();	// Asynchronous Bus Initialization
	InitTimer();	// Timer Initialization
	InitUART();		// UART Initialization
	InitINT();		// Interrupt Enable(External INT and Timer INT)
	InitUSBMon();	// USB Monitor Initialization

	MACRO_PRINT((tmp_string, "\r\n\r\n"));
	MACRO_PRINT((tmp_string, "Mechatronics Course %d\r\n", 2026));
	MACRO_PRINT((tmp_string, "FPGA Ver%2x.%02x\r\n", ((*FPGAVER>>8) & 0xFF), (*FPGAVER & 0xFF)));

	TFlag = 0;

	*FPGALED = 1;			// FPGA LED 1 : ON, 0 : OFF

	GIE();

	WaitTFlagCnt(100);

	while (0) { //양쪽 모터 풀스탭 회전
		*STEP_PHASE_L = 0x2;
		*STEP_PHASE_R = 0x2;		// A
		WaitTFlagCnt(delay);
		
		*STEP_PHASE_L = 0x8;
		*STEP_PHASE_R = 0x8;		// B
		WaitTFlagCnt(delay);

		*STEP_PHASE_L = 0x1;
		*STEP_PHASE_R = 0x1;		// /A
		WaitTFlagCnt(delay);

		*STEP_PHASE_L = 0x4;
		*STEP_PHASE_R = 0x4;		// /B
		WaitTFlagCnt(delay);

		*FPGALED = ~*FPGALED;
	}
	
	//여기부터 마이크로마우스 구현
	
	//값 초기화
	*STEP_PHASE_L = phase[0];
	*STEP_PHASE_R = phase[0];
	
	
	//작동지정하는 곳
		//Move()에 0 넣었을 때 코드가 어떻게 대응하는지 생각해봐야함
		//앞뒤 작동 확인용
		/*
		MoveL(-360.0);
		MoveR(-360.0);
		WaitTFlagCnt(1e5 * 1);
		MoveL(360);
		MoveR(360);
		WaitTFlagCnt(1e5 * 1);
		MoveL(0);
		MoveR(0);
		MACRO_PRINT((tmp_string, "DelayCntL: %d	DelayCntR: %d\r\n", DelayCntL, DelayCntR));
		*/
		
	
	while(0){
		MoveL(200);
		MoveR(200);
		//MACRO_PRINT((tmp_string, "DelayL: %d	InterruptCntL: %d\r\n", DelayCntL, cntL));
		WaitTFlagCnt(1000);
	}
	
	DBGV2(currentVel_R, currentVel_L);


	//VParray 확인용
	if(0){
	MoveL(180);
	MoveR(180);
	WaitTFlagCnt(1e5 * 1);
	currentVel_R = 180;
	currentVel_L = 180;
	}
	else{
	MoveL(0);
	MoveR(0);
	WaitTFlagCnt(1e5 * 1);
	currentVel_R = 0;
	currentVel_L = 0;
	}
	
	MakeVP(500);





	MoveVP(720*3, -720*3);
	WaitTFlagCnt(1e5 * 4);
	MoveVP(720*3, -720*3);
	WaitTFlagCnt(1e5 * 6);
	MoveVP(0, 0);
	WaitTFlagCnt(1e5 * 4);


	MoveVP(1080, 360);

	WaitTFlagCnt(1e5 * 2);
	
	MoveVP(180, 360);
	WaitTFlagCnt(1e5 * 2);
	
	MoveVP(180, -1080);
	WaitMotionDone(currentAdrR, changeAdrR);
	WaitTFlagCnt(1e5 * 2);



	MoveVP(0, 0);
	WaitTFlagCnt(1e5 * 2);
	
	MoveVP(180, -1080);
	WaitMotionDone(currentAdrR, changeAdrR);
	MoveVP(0, 0);
	WaitMotionDone(currentAdrR, changeAdrR);


/*
	MoveCurveRatio(30, 2, 1, 300);
	DBGV2(changeStepDiff, isOuterRight);
	DBGV2(curveMode, changeAdrO);
	
	motionDone = 0; // 최초 0 초기화.
    while(!motionDone){
        if(debugAdrL || debugAdrR){
            //DBGV2(currentAdrO, changeAdrO);
			//DBGV2(currentAdrR, changeAdrO); // AdrR에 입력이 안되는 문제 ? 왜 해결되지
			DBGV2(currentStepDiff, remainDiff); // currentStepDiff = 0 문제
			DBGV2(stepCountL, stepCountR); //
			//DBGV2(remainDiff, brakeDiff); //
            debugAdrL = 0;
            debugAdrR = 0;
        }
    }
    motionDone = 0;
*/
}



