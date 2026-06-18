/* Interupt.c */

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
#include "interrupt.h"
#include "USBMon.h"


unsigned int TFlag = 0;
int cntL = 0;
int cntR = 0;
int cntO = 0;

int DelayCntO = 0; // curveMode일때의 바깥바퀴 상 딜레이 cnt
 
unsigned int phaseAdrL = 0;
unsigned int phaseAdrR = 0;
unsigned int phaseAdrO = 0;

//int save_DelayCntL;
//int save_DelayCntR;

int volatile pulseL, pulseR, pulseO;
int currentDirL, currentDirR, currentDirO;
int changeDirL, changeDirR, changeDirO;

// 각도 처리용 누적 step 카운터
int stepCountL = 0;
int stepCountR = 0;

int remainDiff, brakeDiff;
int motionDone = 0;

int debugAdrL = 0;
int debugAdrR = 0;

interrupt void ISRtimer0()
{

	TFlag = 1;
	*DOUT0 = ~(*DOUT0); 
		//Move() 구현
	if(!VP_ON && !curveMode){

		// MoveR/MoveL 단독 호출 대비: 이번에 Move로 명령되지 않은 바퀴는
		// 직전(MoveVP 등) 속도를 유지한다 -> currentAdr 기반으로 DelayCnt 유지.
		if(!motorRun_L) DelayCntL = VParray[currentAdrL];
		if(!motorRun_R) DelayCntR = VParray[currentAdrR];

		// 상전환은 양 바퀴 모두 수행. 단 "명령도 없고(currentAdr) 유지속도도 0"이면
		// 스텝을 내보내지 않아 실제로 정지한다(VParray[0]은 최저속이지 정지가 아님).
		if(motorRun_L || currentAdrL > 0){
			//MoveL() 작동위치
			if(cntL >= (DelayCntL-1)){ //상전환 딜레이 도달
				phaseAdrL = (phaseAdrL + DirL + 4) % 4;
				*STEP_PHASE_L = phase[phaseAdrL];
				cntL = 0;
			}
			else{
				cntL++;
			}
		}
		if(motorRun_R || currentAdrR > 0){
			//MoveR() 작동 위치
			if(cntR >= (DelayCntR-1)){ //상전환 딜레이 도달
				phaseAdrR = (phaseAdrR + DirR + 4) % 4;
				*STEP_PHASE_R = phase[phaseAdrR];
				cntR = 0;
			}
			else{
				cntR++;
			}
		}
	}
	else if(!curveMode){  // 가속 모드

		changeDirL = DirL;
		changeDirR = DirR;

		DelayCntL = VParray[currentAdrL]; // DelayCnt 초기화
		DelayCntR = VParray[currentAdrR];
		
		// MoveVP to interrupt
		// 방향에 따른 처리
		// ---- Left ----
		
		  // ratio 커브 모드가 아닐 경우 -> 각 바퀴가 독립적으로 상전환 카운터 구동
		if(pulseL){ // 다음 상 딜레이 인가
			pulseL = 0;
			if(currentDirL != changeDirL){ // 다른 방향일 때 -> 무조건 감속하여 adr = 0에서 방향 바꿈
				if(currentAdrL > 1) { // 1보다 크면 감속 // 0은 정지용.
					currentAdrL--; 
					debugAdrL = 1;		
				}
				else {
					currentDirL = changeDirL; // 1에서 방향 전환
				} 
			}
			else{ // 동일 방향일 때 -> changeAdr에 따라 가속 or 감속
				if(currentAdrL < changeAdrL){
					currentAdrL++;
				}
				else if(currentAdrL > changeAdrL){
					currentAdrL--;
				}
				else{// 같으면 Adr 변경 x, 딜레이 유지
					motionDone = 	(currentAdrL == changeAdrL) && 
									(currentAdrR == changeAdrR);
					VP_ON = !motionDone;
				}
				DelayCntL = VParray[currentAdrL];
			}
		}
		// 상전환 딜레이 카운팅 및 다음 상 변경 로직
		if((changeAdrL == 0) && (currentAdrL == changeAdrL))
		{ // 정지상태를 원하고(changeAdr == 0) 현재 거기까지 도달했을 때(currentAdr ==0) 
			cntL = 0; // pulse 발생 못하게 상 딜레이 카운트 정지
		}
		else{
			if(cntL >= (DelayCntL-1)){ // 상전환 딜레이 도달
				pulseL = 1;
				// 상변환 파트
				phaseAdrL = (phaseAdrL + currentDirL + 4) % 4;
				*STEP_PHASE_L = phase[phaseAdrL];
				//MACRO_PRINT((tmp_string, "phaseAdrL: %d	DirL: %d\r\n", phaseAdrL, DirL));
				cntL = 0;
				// 헤딩 방향 계산을 위한 누적 바퀴 카운터 R
				if(currentAdrL != 0) stepCountL += currentDirL;
			}
			else{
				cntL++;
			}
		}
		
		// ---- Right ---- (로직 동일)
		
		if(pulseR){ // 다음 상 딜레이 인가
			pulseR = 0;
			if(currentDirR != changeDirR){ // 다른 방향일 때 -> 무조건 감속하여 adr = 1에서 방향 바꿈
				if(currentAdrR > 1) { // 1보다 크면 감속
					currentAdrR--; 
					debugAdrR = 1;
				}
				else {
					currentDirR = changeDirR; // 1에서 방향 전환
				} 
			}
			else{ // 동일 방향일 때 -> changeAdr에 따라 가속 or 감속
				if(currentAdrR < changeAdrR){
					currentAdrR++;
					debugAdrR = 1;
				}
				else if(currentAdrR > changeAdrR){
					currentAdrR--;
				}
				else{// 같으면 Adr 변경 x, 딜레이 유지
					motionDone = 	(currentAdrL == changeAdrL) && 
									(currentAdrR == changeAdrR);
					VP_ON = !motionDone;
				}
				DelayCntR = VParray[currentAdrR];
			}
		}
		// 상전환 딜레이 카운팅 및 다음 상 변경 로직
		if((changeAdrR == 0) && (currentAdrR == changeAdrR))
		{ // 정지상태를 원하고(changeAdr == 0) 현재 거기까지 도달했을 때(currentAdr ==0) 
			cntR = 0; 
		}
		else{
			if(cntR >= (DelayCntR-1)){ 
				// 상전환 딜레이 도달
				pulseR = 1;
				// 상변환 파트
				phaseAdrR = (phaseAdrR + currentDirR + 4) % 4;
				*STEP_PHASE_R = phase[phaseAdrR];
				//MACRO_PRINT((tmp_string, "phaseAdrR: %d	DirR: %d\r\n", phaseAdrR, DirR));
				cntR = 0;
				// 방향 계산을 위한 누적 바퀴 카운터 R
				if(currentAdrR != 0) stepCountR += currentDirR;
			}
			else{
				cntR++;
			}
		}
	}
	else{ 
	///////////////////////////////////////////////////////////////////////////
	// curveMode == 1, 느린 바퀴가 빠른 바퀴 상전환에 동기화되어 기준 ratio마다 상전환. 
	//
		// 커브모드용 업데이트
		changeDirO = DirO;
		currentDirO = changeDirO;	// 회전 방향 적용(미할당시 0 -> 상이 안 바뀜)
		DelayCntO = VParray[currentAdrO];
		if(stepCntRst){ // CurveMode 사용 전 초기화용
			stepCntRst = 0;
			stepCountL = 0;
			stepCountR = 0;
		}
		if(pulseO){ // curveMode 초기 구현-> 동일 방향일 때만 작성 -> changeAdr에 따라 가속 or 감속
			pulseO = 0;
			debugAdrR = 1;	// 매 outer 펄스마다 디버그 게이트 open (가속/순항/감속 전구간 로그)
			if(currentAdrO < changeAdrO){
				currentAdrO++;
			}
			else if(currentAdrO > changeAdrO){
				currentAdrO--;
			}
			// 같으면 Adr 변경 x, 딜레이 유지.
			// 주의: 여기서 motionDone을 set하면 vmax 순항(currentAdrO==changeAdrO) 시점에
			//       회전 도중 done이 떠버림. 진짜 종료는 아래 (changeAdrO==0) 조건만 사용.
			DelayCntO = VParray[currentAdrO];
			
			curveAccum += ratio_den;

			// 종료 로직: step 모터에서 하던 감속 알고리즘 사용.
			// -> 목표거리까지 남은 step이 현재 adr과 같으면 감속 시작하는 형태.
			// 단, 지금은 확인할 대상이 목표step차 - 현재step차임.
			// 즉 remainDiff = abs(targetStepDiff - currentStepDiff)가 남은 각도에 대응하는 스텝수가 됨.
			// 이때, 단순히 현재 if(AdrO == remainDiff) -> 감속시작 처럼 구현시 remainDiff가 AdrO--마다 같이 1씩 변경되는게 아니라,
			//  ratio에 따라 다를 수 있음을 고려해야 함. 

			// ex) ratio가 0.25 <=> num:den == 4:1일 경우
			// 바깥바퀴 상변환 4번당 안쪽 1번 변경.
			// -> AdrO-- 가 네번 발생할 동안, stepDiff는 3번만 변경 -> AdrO가 0이 되어도  remainDiff != 0이 됨. 목표각도 도달 x.
			// 즉 AdrO == remainDiff / (1-ratio)이면 동작하게 해야 적절함.
		
			currentStepDiff = stepCountR - stepCountL;

			remainDiff = abs(changeStepDiff) - abs(currentStepDiff);
			brakeDiff = currentAdrO * (ratio_num - ratio_den) / ratio_num;

			if(brakeDiff < 1){ 
				// 
				brakeDiff = 1;
			}

			// 감속 시작
			if((remainDiff <= brakeDiff)){
				changeAdrO = 0; // 
			}

			// 종료: ISR 자체 종료(VP_ON 패턴과 동일하게 ISR이 curveMode를 내림).
			// caller는 MoveCurveRatio(); while(!motionDone); 만으로 사용 가능.
			if((currentAdrO == changeAdrO) && (changeAdrO == 0)){
				motionDone = 1;
				curveMode = 0;
				// 종료 후 ISR은 Move 분기로 빠짐. Move 분기는 currentAdr>0면 계속
				// 스텝을 내보내므로, 정지 상태를 위해 양 바퀴 속도 인덱스를 0으로 리셋.
				currentAdrL = 0;
				currentAdrR = 0;
			}
		}
		// 상전환 딜레이 카운팅 및 다음 상 변경 로직
		//////////  Outer //////////
		if((changeAdrO == 0) && (currentAdrO == 0)){
			// 목표 도달 후 adr=0까지 감속 완료 -> 펄스 발생 정지(VP 모드와 동일한 정지 가드).
			// currentAdrO=0은 정지가 아니라 VParray[0](최저속)이므로, 가드가 없으면
			// 바깥바퀴가 최저속으로 계속 상전환하여 멈추지 않음.
			cntO = 0;
		}
		else if(cntO >= (DelayCntO-1)){ // 상전환 딜레이 도달
			pulseO = 1;
			// Outer 상변환 파트
			// 상 idx 계산
			phaseAdrO = (phaseAdrO + currentDirO + 4) % 4;
			// Outer만 고정적으로 1++, inner는 Bresenham accum 방식으로.
			if(isOuterRight){
				currentAdrR 	= currentAdrO;
				phaseAdrR 		= phaseAdrO;
				*STEP_PHASE_R = phase[phaseAdrR];
				stepCountR++;
			}
			else{
				currentAdrL 	= currentAdrO;
				phaseAdrL 		= phaseAdrO;
				*STEP_PHASE_L = phase[phaseAdrL];
				stepCountL++;
			}
			//MACRO_PRINT((tmp_string, "phaseAdrL: %d	DirL: %d\r\n", phaseAdrL, DirL));
			cntO = 0;
		}
		else{
			cntO++;
		}
		// 상전환 딜레이 카운팅 및 다음 상 변경 로직  
		//////////  inner -> Outer에 종속 //////////
		// Bresenham accum 방식 쓰는 이유: 
			// inner vel = outer vel * ratio
			// main에선 1:2의 예시만 들었음. 그럼 1:N 비율만 가능?
			// 그렇지 않음. 바깥쪽 상변환을 O, 안쪽을 I라고 할 때, 
			// { O >> O,I >> O >> O,I >> O,I } >> { O >> O,I >> ....
			// 처럼 O:I 상변환 비율을 평균적으로 5:3으로 맞춰줌으로써 A:B 가능.
			// 이때, 100:73같은 복잡한 비율이면? 
			// { O,I >> O,I >> ...x73 >> O >> O >> ...x27 } >> ... 처럼 분포를 신경쓰지 않으면
			// 앞쪽에서 두 바퀴가 모두 등가속운동하다 뒤에 변화가 집중됨.
			// 이를 평균적으로 맞추기 위해 사용하는 로직이 Bresenham 알고리즘.
		if(curveAccum >= ratio_num){
			curveAccum -= ratio_num;
			if(isOuterRight){
				currentAdrL++;
				phaseAdrL = (phaseAdrL + currentDirO + 4) % 4;
				*STEP_PHASE_L = phase[phaseAdrL];
				stepCountL++;
			}
			else{
				currentAdrR++;
				phaseAdrR = (phaseAdrR + currentDirO + 4) % 4;
				*STEP_PHASE_R = phase[phaseAdrR];
				stepCountR++;
			}
		}
	}
	
}

interrupt void ISRNMI()
{
}

interrupt void ISRextint6()
{
	volatile unsigned int tmp;

	while((tmp = *intid_fifo & 0x0F) != 1) {
		if(tmp == 0x04) {
			while(*linestatus&0x01) {
				*(compacket.wr_ptr) = *txrx_divl & 0xFF;
				SendByte(*(compacket.wr_ptr));
				if(compacket.wr_ptr == (compacket.packet+SIZE_OF_COMPACKET-1)) {
					compacket.wr_ptr = compacket.packet;
				}
				else {
					compacket.wr_ptr++;
				}
			}
		}
		else if(tmp == 0x02) {
			if(respacket.char_num-- > 0) {
				*txrx_divl = respacket.string[respacket.char_to_send++];
			}
			else {
				*inten_divh = 0x01;			
				respacket.flag = 0;
			}
		}
		else if(tmp == 0x0C) {
			while(*linestatus & 0x01) {
				*(compacket.wr_ptr) = *txrx_divl & 0xFF;
				if(compacket.wr_ptr == (compacket.packet+SIZE_OF_COMPACKET-1)) {
					compacket.wr_ptr = compacket.packet;
				}
				else {
					compacket.wr_ptr++;
				}
			}
		}
	}
}
