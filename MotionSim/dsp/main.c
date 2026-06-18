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
#define WHEEL_R 0.0257 		// 바퀴의 반지름
#define WHEEL_BASE 0.111 		// 바퀴 사이의 거리 -> 왼쪽 끝에서 오른쪽 끝.
//(중심 - 중심으로 하면 덜 도는 문제 있음. 호 길이 R*theta에서 회전 반경 R은 결국 기구가 그리는 호의 길이이니 끝 - 끝이 맞는듯.)
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

#ifndef SIM_BUILD   /* PC digital-twin supplies tick-pumping versions (sim_main.c) */
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
#endif /* !SIM_BUILD */


//좌우 스텝모터 구동 구현

	//전역변수 구현
	unsigned int phase[4] = {0x2, 0x8, 0x1, 0x4};
	int DirL, DirR;	//방향지정변수, 1이 전진, -1이 후진 (Boolean으로 변경가능?)
	unsigned int DelayCntL, DelayCntR;	//대기할 스텝수
	int motorRun_L, motorRun_R;	//속도 0일때, 작동 중지 명령

void MoveL(float spdL){
	int i;

	DirL = 1;	// interrupt에서 PhaseAdr++, --를 PhaseAdr += Dir로 대체하기 위해 +-1로 설정
	DelayCntL = 0;	//스텝수 초기화
	motorRun_L = 1;

	// Move 명령이 바퀴 소유권을 가져감 -> 다른 모드 해제
	VP_ON = 0;
	curveMode = 0;

	//속도 0일시 작동중지
	if(spdL == 0){
		motorRun_L = 0;
		currentAdrL = 0;	// 정지: 유지속도 인덱스도 0으로
		return;
	}


	//interrupt.c로 값을 보냄
	DirL = (spdL >= 0) ? 1 : -1;  //입력값(속도)에서 방향 계산
	DelayCntL = (int)(1.8f/fabsf(spdL) * 100000.0f);	//속도입력에서 필요 대기스텝 계산, 180000.0f = 스텝각 1.8도 * 1sec/10us 변환

	// VP 핸드오프용: 현재 속도에 해당하는 Adr 갱신(다음 MoveVP가 이 속도에서 출발)
	for(i = 0; i < VPNUM; i++){
		if(VParray[i] <= DelayCntL){ currentAdrL = i; break; }
	}
}


void MoveR(float spdR){
	int i;

	//변수 설정
	DirR = 1;
	DelayCntR = 0;
	motorRun_R = 1;

	// Move 명령이 바퀴 소유권을 가져감 -> 다른 모드 해제
	VP_ON = 0;
	curveMode = 0;

	//속도 0일시 작동중지
	if(spdR == 0){
		motorRun_R = 0;
		currentAdrR = 0;	// 정지: 유지속도 인덱스도 0으로
		return;
	}

	//interrupt.c로 값을 보냄
	DirR = (spdR >= 0) ? 1 : -1;
	DelayCntR = (int)(180000.0f/fabsf(spdR));

	// VP 핸드오프용: 현재 속도에 해당하는 Adr 갱신(다음 MoveVP가 이 속도에서 출발)
	for(i = 0; i < VPNUM; i++){
		if(VParray[i] <= DelayCntR){ currentAdrR = i; break; }
	}
}




//VParray 생성 MakeVP()

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

//VParray 사용 움직임 구현 MoveVP()

//MoveVP() 전역변수 설정
int volatile currentAdrL = 0; //코드 실행 직전 속도의 Array 주소값
int volatile changeAdrL = 0;  //입력값(목표 속도)의 Array 주소값
int volatile currentAdrR = 0;
int volatile changeAdrR = 0;
int volatile currentAdrO = 0;
int volatile changeAdrO = 0;
int VP_ON = 0;	//VP 작동 상태를 확인



unsigned int changeDelayR, changeDelayL, changeDelayO; //입력값(목표 속도)의 Delay값


void MoveVP(float changeVel_L, float changeVel_R){

	/* 	//지역변수 설정(main에서 pulse로 컨트롤하는 버젼)
	int Vel_R, Vel_L;
	int sign_R, sign_L;
	*/
	int i;
	VP_ON = 1;
	curveMode = 0;		// VP가 바퀴 소유권을 가져감
	motionDone = 0;		// 연속 호출 대비: 이전 done 잔류 제거
	// Move 분기로 빠졌을 때 '직전 VP 속도 유지'가 동작하도록 motorRun 해제.
	// (motorRun이 1로 남아 있으면 Move 분기가 stale한 MoveL/R 딜레이를 씀)
	motorRun_L = 0;
	motorRun_R = 0;
	DirL = (changeVel_L >= 0) ? 1 : -1;
	DirR = (changeVel_R >= 0) ? 1 : -1;

	//목표값 주소 찾기 (속도 0이면 changeDelay 무한대 -> changeAdr=0(정지)로 처리)
	if(changeVel_L == 0){
		changeAdrL = 0;
	}
	else{
		changeDelayL = (int)(180000.0f / fabsf(changeVel_L));
		for(i = 0; i < VPNUM; i++){
			if (VParray[i] <= changeDelayL){ changeAdrL = i; break; }
		}
	}
	if(changeVel_R == 0){
		changeAdrR = 0;
	}
	else{
		changeDelayR = (int)(180000.0f / fabsf(changeVel_R));
		for(i = 0; i < VPNUM; i++){
			if (VParray[i] <= changeDelayR){ changeAdrR = i; break; }
		}
	}
	MACRO_PRINT((tmp_string, "changeDelayR: %d	changeDelayL: %d\r\n", changeDelayR, changeDelayL));

	MACRO_PRINT((tmp_string, "currentAdrR: %d	changeAdrR: %d\r\n ", currentAdrR, changeAdrR));
	MACRO_PRINT((tmp_string, "currentAdrL: %d	changeAdrL: %d\r\n", currentAdrL, changeAdrL));
	MACRO_PRINT((tmp_string, "DirL: %d DirR: %d	\r\n", DirL, DirR));
	VP_ON = 1;
}

void Rotate(float angle, float vmax){
	
}

// curveMode Adr 처리용
int DirO;

void CurveVP(float changeVel_O){ // MoveVP와 로직 동일. changeAdr만 찾아줌 

	int i;
	DirO = (changeVel_O >= 0) ? 1 : -1;

	changeDelayO = (int)(1.8f / fabsf(changeVel_O) * 100000.0f); // fabsf: 정수 abs의 float 절단/0나눗셈 방지
	MACRO_PRINT((tmp_string, "changeDelayO: %d\r\n", changeDelayO));
	for(i = 0; i < VPNUM; i++){
		if (VParray[i] <= changeDelayO){
			changeAdrO = i; //목표 Adr 구함
			break;
		}
	}
}	

// curveMode 처리용 전역변수(interrupt에서 사용 예정)
int volatile curveMode = 0; 
int volatile isOuterRight = 0;
int ratio_num, ratio_den;	// 바퀴의 속도비
int curveAccum;				// bresenham 누적 알고리즘용 
int volatile changeStepDiff;			// interrupt에 보내줄 목표step차
int volatile currentStepDiff; 		// interrupt에서 매번 갱신할 현재Step차
int stepCntRst;				// 양 바퀴 step차 계산용 stepCnt 초기화신호

float curveAccumErr = 0.0f;	// 각도->스텝 변환의 누적 소수부 오차(서브스텝). 호출 간 유지.

void ResetCurveErr(){		// 새 회전 시퀀스를 절대 기준으로 다시 시작하고 싶을 때 호출
	curveAccumErr = 0.0f;
}

void ResetStepCount(){
	stepCntRst = 1;
}


int AngleToDiffStep(float angle){
// 헤딩 회전 각도를 받아서 그 각도로 향하기 위한 바퀴간의 스텝 차이로 환산해주는 함수
// 용도: AngleToDiffStep(90.0) => 110 -> stepCountR-stepCountL = 110이어야 90도.
	float theta;
	float diffDist;
	float value;		// 이 각도에 필요한 '실수' 스텝차
	float ip, carry;	// 정수부 / 누적오차의 보정(정수)분
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
	value = diffDist / STEP_DIST;

	// (int)(value+0.5) 단순 반올림은 매 호출 소수부를 버리므로, 같은 각도를 여러 번 주면
	// 오차가 매번 같은 방향으로 누적되어 90도에서 어긋남.
	// -> 이번 호출의 소수부를 curveAccumErr에 누적하고, 1스텝 이상 쌓이면 보정 스텝으로 방출.
	//    (이미 쓰는 Bresenham과 같은 오차피드백 방식. modff: 정수부/소수부 분리.)
	curveAccumErr += modff(value, &ip);       // ip=정수부, 반환=소수부
	curveAccumErr  = modff(curveAccumErr, &carry); // carry=누적오차의 정수부(보정), 나머지는 유지
	targetDiffStep = (int)ip + (int)carry;

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
	
	// 사용법: MoveCurveRatio(...); while(!motionDone); (종료/curveMode=0은 ISR이 처리)
	// MoveVP(); etc....


	ResetStepCount();   // 현재 헤딩 기준으로 목표 회전각도만큼 회전하기 위해 초기화
	motionDone = 0;     // 이전 동작의 done 잔류 제거(연속 호출 대비)
	changeStepDiff = AngleToDiffStep(angle);
	ratio_num = num;
	ratio_den = den;
	curveAccum = 0; 	// 초기화
	CurveVP(vmax); 		// 회전중 최대속도에 해당하는 VParray Adr 계산해 전달, DirO 설정

	// 전진/후진(DirO)에 따라 outer 바퀴가 뒤집힘.
	// 같은 +angle이라도 forward는 R-outer, reverse는 L-outer여야 동일 방향(+) 헤딩 회전.
	// sign(angle) == sign(DirO) 이면 R-outer, 아니면 L-outer.
	isOuterRight = ((changeStepDiff > 0) == (DirO > 0)) ? 1 : 0;
	phaseAdrO = (isOuterRight) ? phaseAdrR : phaseAdrL; // 커브모드 진입 1회시 초기화

	// 종료 후 ISR이 VP/Move 분기에서 헛돌지 않도록 정지상태로 세팅
	VP_ON = 0;
	motorRun_L = 0;
	motorRun_R = 0;

	curveMode = 1;      // 모든 상태 세팅 후 마지막에 arm(ISR이 stale 상태로 진입하는 것 방지)
	DBGV2(curveMode, changeAdrO);
}



#ifndef SIM_BUILD   /* PC digital-twin supplies tick-pumping WaitMotionDone + main (sim_main.c) */
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

	//값 초기화
	*STEP_PHASE_L = phase[0];
	*STEP_PHASE_R = phase[0];


	//Test 01 : 모터 회전 확인
	/*
	delay = 1000;
	
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
	*/

//Test 02 : MoveL(), MoveR() 작동 확인
	/*
	MoveL(-360.0);
	MoveR(-360.0);
	WaitTFlagCnt(1e5 * 1);
	MoveL(360);
	MoveR(360);
	}
	else{
	MoveL(0);
	MoveR(0);
*/

//Test 03 : MoveVP 확인

	//속도 프로파일 생성(입력: 가속도)
	MakeVP(500);
	


	
	MoveVP(1080, 360);
	DBGV2(VP_ON, curveMode);
	WaitMotionDone(currentAdrR, changeAdrR);
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
    // curveMode=0 / motionDone 종료는 ISR이 처리(자체 종료). 별도 정리 불필요.
*/
}
#endif /* !SIM_BUILD */
