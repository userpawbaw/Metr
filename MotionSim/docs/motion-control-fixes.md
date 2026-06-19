# 모션 제어 펌웨어 수정 내역 (MoveVP / MoveCurveRatio / Move)

> 대상 파일: `dsp/main.c`, `dsp/interrupt.c` (보드 펌웨어, C6701/C67x)
> 검증 환경: `sim/` 디지털 트윈 (펌웨어 `main.c`/`interrupt.c`를 `-DSIM_BUILD`로 그대로 컴파일, `sim_hw.c`가 가상 스텝모터/포즈 적분, `sim_main.c`가 시나리오 구동)
> 기준 커밋: `4858a1f` → 작업 브랜치 `claude/movecurveratio-isr-debug-t1tnes`

이 문서는 이번 세션에서 잡은 버그/개선을 **증상 → 근본 원인 → 수정 → 검증** 형식으로 정리한 것이다. 설계를 같이 한 다른 AI와 공유 + 보고서 작성용.

---

## 0. 요약 (TL;DR)

| # | 영역 | 증상 | 근본 원인 | 수정 |
|---|------|------|-----------|------|
| 1 | Curve | 회전 후 안 멈추고 계속 돎 | `curveMode` 종료 처리 부재 + outer 펄스 정지 가드 없음 | ISR terminal에서 `curveMode=0` + 정지 가드 |
| 2 | Curve | 물리적으로 안 움직임(상 고정) | `currentDirO` 미할당(=0) → 상 인덱스 불변 | `currentDirO = changeDirO` |
| 3 | Curve | 순항 중 done 조기 발생 | `motionDone=(currentAdrO==changeAdrO)`가 vmax 도달 시 점화 | 해당 대입 제거, terminal 조건만 사용 |
| 4 | Curve | 전·후진 회전 방향 | `isOuterRight`가 angle 부호만 봄 | `isOuterRight = (sign(angle)==sign(DirO))` |
| 5 | MoveVP | 가속→**중간 정지**→감속(중간 잘림) | 순항 도달 시 `VP_ON=!motionDone`로 VP를 꺼 Move 분기로 빠짐, motorRun=0이라 동결 | Move 분기가 직전 속도 유지 + 모드 소유권 정리 |
| 6 | Move | MoveVP 뒤 MoveR/L 단독 시 반대 바퀴 정지 | Move 분기가 `motorRun`인 바퀴만 구동 | 미명령 바퀴는 `currentAdr` 속도 유지, 양쪽 항상 상전환 |
| 7 | 공통 | 저속·소수 속도에서 깨짐 | `abs(float)` = 정수 abs(절단/0나눗셈) | `fabsf()` |
| 8 | Curve | 같은 각 반복 시 90°에 못 맞음 | `(int)(x+0.5)` 매 호출 소수부 버림 | `modff` 잔차 누적기(반올림) |
| 9 | Sim | 시뮬 각도가 펌웨어와 불일치 | 팀원이 펌웨어 기하 변경, 시뮬 미반영 | `sim_hw.h`/`viz_common.py` 미러링 |
| 10 | Curve | ISR 연산량(나눗셈) | `brakeDiff`의 `/ratio_num`(C6x 정수나눗셈=라이브러리 호출) | cross-multiply로 나눗셈 제거 |

---

## 1. ISR 구조 (전제)

`ISRtimer0()`는 100kHz(10µs)로 호출되며, 세 모드를 상호배타적으로 분기한다.

```c
if(!VP_ON && !curveMode){ /* Move: MoveL/MoveR, 고정속도 */ }
else if(!curveMode){      /* VP: 가감속 프로파일 */ }
else {                    /* Curve: ratio 회전(Bresenham) */ }
```

공유 상태(모든 모드가 읽고/쓰는 핵심 변수):
- `currentAdrL/R/O` : 현재 속도의 VParray 인덱스 (0 = 정지, 클수록 빠름)
- `changeAdrL/R/O`  : 목표 속도 인덱스
- `phaseAdrL/R/O`   : 스텝모터 상 인덱스
- `DirL/R/O`        : 방향(±1)
- `VP_ON`, `curveMode` : 모드 플래그
- `motionDone`      : "목표 도달" 알림 (main의 대기 루프용)

**설계 원칙(이번에 확립):** 모드 플래그는 *진입 함수*가 소유권을 세팅하고(예: `MoveVP`→`VP_ON=1; curveMode=0`), ISR은 목표 도달을 `motionDone`으로만 알린다. (curve만 예외적으로 ISR이 종료 시 `curveMode=0` — `VP_ON` 패턴과 동일.)

---

## 2. Curve 모드 (`MoveCurveRatio`)

### 2.1 회전 후 안 멈춤 + 안 움직임

**증상:** 비율대로 회전은 하지만 정지하지 않고 계속 돎. (시뮬에선 반대로 한 스텝도 안 나가고 원점 고정.)

**근본 원인 (복합):**
1. `currentDirO`가 한 번도 대입되지 않아 0 → `phaseAdrO = (phaseAdrO + currentDirO + 4)%4`가 불변 → 상이 안 바뀜(물리 회전 X). 보드에선 잔류값에 따라 우연히 돌기도 함(방향 미정의).
2. `currentAdrO==0`은 정지가 아니라 `VParray[0]`(최저속). 그런데 curve 분기엔 VP 모드 같은 "펄스 정지 가드"가 없어, 감속이 끝나도 outer 바퀴가 최저속으로 영원히 상전환.
3. `curveMode`를 0으로 되돌리는 코드가 어디에도 없어 ISR이 curve 분기에 영구히 갇힘.
   - 시뮬이 "멈춘 것처럼" 보였던 건 `while(!motionDone)` 루프가 tick 펌핑을 멈춰서일 뿐, 펌웨어가 모터를 세운 게 아니었음.

**수정:**
```c
// (1) 방향 적용
changeDirO = DirO;
currentDirO = changeDirO;

// (2)+(3) terminal에서 ISR 자체 종료 + 정지
if((currentAdrO == changeAdrO) && (changeAdrO == 0)){
    motionDone = 1;
    curveMode = 0;       // VP_ON 패턴과 동일하게 ISR이 모드 해제
    currentAdrL = 0;     // 종료 후 Move 분기로 빠져도 안 기어가게(아래 6장 참조)
    currentAdrR = 0;
}
// (2) outer 펄스 정지 가드
if((changeAdrO == 0) && (currentAdrO == 0)){
    cntO = 0;            // 펄스 발생 정지(VP 모드와 동일 방식)
}
else if(cntO >= (DelayCntO-1)){ ... }
```

이로써 호출부는 `MoveCurveRatio(...); while(!motionDone);` 만으로 사용 가능.

### 2.2 순항 중 motionDone 조기 발생

**근본 원인:** pulseO 블록의 "adr 동일" 가지에 `motionDone = (currentAdrO==changeAdrO)`가 있어, **vmax 순항 도달 시점**(가속 끝)에 done이 떠버림. 큰 각/높은 vmax로 순항 구간이 생기면 회전 도중 종료.

**수정:** 해당 대입 제거. 진짜 종료는 `(changeAdrO==0)` terminal 조건만 담당.

### 2.3 전진/후진 회전 방향

**요구:** 같은 `+angle`이라도 전진/후진에 따라 outer 바퀴가 바뀌어야 같은 방향으로 헤딩 회전(주차 시퀀스용).

**기구학:** $d\theta \propto \text{DirO}\cdot(\text{stepsR}-\text{stepsL})$. 따라서
`sign(dθ) == sign(angle)`을 만족하려면:

```c
// CurveVP(vmax)가 DirO를 먼저 설정한 뒤
isOuterRight = ((changeStepDiff > 0) == (DirO > 0)) ? 1 : 0;
```
- `sign(angle)==sign(DirO)` → R-outer, 아니면 L-outer.
- 정지 로직은 `abs()` 기반이라 stepCount를 부호로 추적하지 않아도 양방향 모두 동작(검증됨).

### 2.4 `MoveCurveRatio` 함수 정리(순서/초기화)

```c
ResetStepCount();
motionDone = 0;                         // 연속 호출 시 done 잔류 제거
changeStepDiff = AngleToDiffStep(angle);
ratio_num = num; ratio_den = den;
curveAccum = 0;
CurveVP(vmax);                          // DirO, changeAdrO 설정
isOuterRight = ((changeStepDiff>0)==(DirO>0)) ? 1 : 0;
phaseAdrO = isOuterRight ? phaseAdrR : phaseAdrL;
VP_ON = 0; motorRun_L = 0; motorRun_R = 0;  // 종료 후 헛돌지 않게
curveMode = 1;                          // 모든 상태 세팅 후 '맨 마지막'에 arm
```
포인트: `curveMode=1`을 마지막에 둬서 ISR이 stale 상태로 curve 분기에 진입하는 것을 방지.

---

## 3. MoveVP "중간이 잘리는" 버그 (5번)

**증상:** `MoveVP(360,360); … ; MoveVP(0,0)`에서 가속 후 일정 속도에서 **갑자기 정지**, 한참 뒤 다시 비슷한 속도로 점프 후 감속. (가감속은 하는데 중간이 잘림.)

**근본 원인:** VP 분기의 "adr 동일" 가지에 `VP_ON = !motionDone;`가 있었음. 순항(목표 속도) 도달 = `motionDone=1` = `VP_ON=0` → 다음 tick부터 `!VP_ON && !curveMode`가 참이라 **Move 분기로 빠짐**. 그런데 `MoveVP`는 `motorRun`을 세팅하지 않아 Move 분기가 바퀴를 동결 → 다음 `MoveVP` 호출이 `VP_ON`을 다시 켤 때까지 멈춤.

**검증(시뮬 `vptest`):**
```
reached cruise: currentAdrR=71 changeAdrR=71 VP_ON=0
after +1s hold: dRight=0   ← 순항 중인데 스텝 0개 (동결, 버그)
```
수정 후: `after +1s hold: dRight=200` (= 360°/s 순항 유지).

**수정 방향(팀 요청: 2분기 구조 유지):** ISR이 자기 모드를 끄지 않게 하고, Move 분기가 "직전 속도 유지"를 떠맡게 함(6장). 모드 소유권은 진입 함수가 정리:
- `MoveVP`: `VP_ON=1; curveMode=0; motionDone=0; motorRun_L=motorRun_R=0;`
- `MoveR/MoveL`: `VP_ON=0; curveMode=0; motorRun=1;`

---

## 4. Move 분기: 미명령 바퀴 속도 유지 (6번)

**요구:** `MoveVP(360,360)` 후 `MoveR(720)`를 단독 호출하면 → R만 720으로, **L은 360을 유지**해야 함. 기존 Move 분기는 `if(motorRun_L)`인 바퀴만 구동해서 반대 바퀴가 멈춤.

**수정 (구조 유지, 분기 내부만 변경):**
```c
if(!VP_ON && !curveMode){
    // 이번에 Move로 명령 안 된 바퀴는 직전(VP) 속도 유지
    if(!motorRun_L) DelayCntL = VParray[currentAdrL];
    if(!motorRun_R) DelayCntR = VParray[currentAdrR];

    // 상전환은 양 바퀴 모두 수행. 단 (명령X && 유지속도==0)이면 스텝 안 냄 = 진짜 정지.
    if(motorRun_L || currentAdrL > 0){ /* cntL 상전환, DirL 사용 */ }
    if(motorRun_R || currentAdrR > 0){ /* cntR 상전환, DirR 사용 */ }
}
```
핵심 가드 `currentAdr > 0`: `currentAdr==0 ⇔ 정지`라는 불변식을 만든다. 이게 없으면 `VParray[0]`(최저속)으로 영원히 기어감(= `MoveVP(0,0)`이 안 멈추는 회귀). 그래서 2.1의 curve terminal에서 `currentAdrL/R=0` 리셋, MoveR/L(0)에서 `currentAdr=0` 세팅이 함께 필요.

**보너스:** 이 변경으로 3장의 MoveVP 순항 동결도 같이 해결됨(순항 시 `VP_ON`이 떨어져도 Move 분기가 `currentAdr` 속도로 계속 구동).

**부수 수정:** Move 분기 좌측 바퀴가 `DirR`을 쓰던 버그 → `DirL`.

### 4.1 모드 간 핸드오프 (MoveR↔MoveVP)

다음 `MoveVP`가 "직전 실제 속도에서" 가감속하려면, `MoveR/MoveL`도 자기 속도에 해당하는 `currentAdr`을 갱신해야 한다(MoveVP와 동일 룩업):
```c
DelayCntR = (int)(180000.0f / fabsf(spdR));
for(i=0;i<VPNUM;i++){ if(VParray[i] <= DelayCntR){ currentAdrR = i; break; } }
```

**검증(시뮬 `handoff`): `VP(360,360)→R(720)→VP(240,240)→VP(0,0)`**
| 구간 | L | R | 환산 |
|------|---|---|------|
| 순항 유지 | 200 | 200 | 360 / 360°/s |
| MoveR(720) | 200 | 400 | 360 / **720**°/s |
| VP(240,240) | 133 | 134 | 240 / 240°/s |
| VP(0,0) | 0 | 0 | 정지 |

---

## 5. `abs(float)` → `fabsf()` (7번)

`abs`는 `int abs(int)`라 float 인자가 정수로 **절단**된다. `|속도|<1`이면 0 → `180000/0` 0나눗셈. `MoveL/MoveR/MoveVP/CurveVP`의 속도→딜레이 계산을 모두 `fabsf()`로 교체. (정수 속도에선 증상이 잠복.)

추가: `MoveVP`에서 `changeVel==0`이면 `changeAdr=0`으로 분기해 `180000/0`을 원천 차단.

---

## 6. 각도 정밀도: 반올림 잔차 누적 (8번)

**증상:** 3×30° 주차가 정확히 90°에 안 맞음.

**근본 원인:** `targetDiffStep = (int)(value + 0.5f)`는 매 호출 소수부를 버린다. 같은 각을 반복하면 같은 방향 오차가 매번 누적.

**수정 (`AngleToDiffStep`, 오차피드백 — 기존 Bresenham과 같은 사상):**
```c
float value = diffDist / STEP_DIST;     // 실수 스텝차
curveAccumErr += modff(value, &ip);            // 이번 호출 소수부 누적 (ip=정수부)
curveAccumErr  = modff(curveAccumErr, &carry); // carry=누적오차의 정수부(보정 스텝)
targetDiffStep = (int)ip + (int)carry;
```
- `curveAccumErr` 초기값 **0.5** → 누적합을 floor가 아닌 **반올림**(장기 드리프트 0).
- 이식성: 보드 런타임에 `modff`가 없으면 `modf`(double) 또는 `ip=floorf(value); frac=value-ip;`로 대체.

**검증:** 주차 `88.36° → 90.02°` (각 세그먼트 30.01/30.00/30.01).

---

## 7. 시뮬-펌웨어 기하 동기화 (9번)

펌웨어 기하 재보정(`WHEEL_R 0.05→0.0257`, `WHEEL_BASE 0.11→0.111`, 끝-끝 기준) 후 시뮬(`sim_hw.h`)·플롯(`viz_common.py`)이 옛 값이라 디지털 트윈이 어긋났다. 동일 값으로 미러링.

부수효과: 해상도가 30°당 36→**72스텝**으로 올라가며 예전의 "물리 stepDiff가 목표보다 1 적던" 저해상도 양자화 artifact가 사라짐(이제 target=firmware=physical).

---

## 8. ISR 연산량: brake 조건의 나눗셈 제거 (10번)

**문제:** `brakeDiff = currentAdrO*(num-den)/num`의 정수 나눗셈. C6701(C67x)은 **정수 나눗셈 HW가 없어** 라이브러리 호출(수십 cycle). 매 outer 펄스마다 발생.

**해법:** 감속 조건 `remainDiff ≤ currentAdrO·(num−den)/num`의 양변에 `num(>0)`을 곱해 나눗셈 제거.
```c
brakeDiff = currentAdrO * (ratio_num - ratio_den);          // 분자(나눗셈 없음, 로그용)
if( (remainDiff <= 1) || (remainDiff * ratio_num <= brakeDiff) ) changeAdrO = 0;
```
- `remainDiff<=1` 항이 기존 `if(brakeDiff<1)brakeDiff=1` 클램프를 **정수 등가**로 보존.
- 정수 곱 2 + 비교, 나눗셈 0. 오버플로 안전(값 범위 ≪ int).
- **Bresenham은 부적합:** 일회성 비교엔 누적기/분기만 늘어 손해. cross-multiply가 최소 연산.
- 동작 동일성 검증: 주차 30.01/60.01/90.02°, curve 30.01°+정지.

---

## 9. 참고 공식 (보고서용)

**스텝차 1당 헤딩 변화** (차동 구동):
$$\Delta\theta = n\cdot\frac{\text{STEP\_DIST}}{\text{WHEEL\_BASE}}\ [\text{rad}], \quad \text{STEP\_DIST}=\text{WHEEL\_R}\cdot\frac{1.8\pi}{180}$$
도 단위로는 π·180이 소거되어:
$$\frac{\Delta\theta}{n}=\frac{1.8\cdot\text{WHEEL\_R}}{\text{WHEEL\_BASE}}\ [\text{deg/stepDiff}]$$
- 현재값 = `1.8·0.0257/0.111 = 0.4168 °/stepDiff` (72 stepDiff → 30.01°)
- 역변환(=`AngleToDiffStep`): `n = θ·WHEEL_BASE / (1.8·WHEEL_R)`

**속도→딜레이/Adr:** `DelayCnt = 180000/|속도(deg/s)|` (10µs tick, 1.8°/step 기준). `VParray[i]=sqrt(1.8/(2·accel·(i+1)))·1e5`, Adr은 `VParray[i] ≤ DelayCnt`인 첫 i.

---

## 10. 시뮬 재현 시나리오

```sh
cd sim && make
./motion_sim curve      # 단일 30° ratio 회전 → 정지 (post-stop dL=dR=0)
./motion_sim parking    # 전진+30/후진+30/전진+30 → ~90° 직각주차
./motion_sim revfwd     # 후진→전진 핸드오프
./motion_sim vptest     # MoveVP 순항 동결 재현/정지 검증
./motion_sim handoff    # VP→R→VP→정지 (미명령 바퀴 유지 검증)
./motion_sim movevp     # 기존 MoveVP 시퀀스
```

---

## 11. 남은/주의 항목

- **MoveVP(0,0)의 방향 강제:** `DirR=(0>=0)?1:-1=1`이라 직전이 후진이면 adr=1에서 방향이 한 번 뒤집히는 미세 quirk(속도 0이라 무해).
- **Curve 후진(reverse) 정밀 추적:** 현재 정지 로직은 `abs()` 기반이라 양방향 동작하나, 부호 있는 헤딩 적산이 필요해지면 curve의 `stepCount++`를 `+= currentDirO`로 바꿔야 함(현재는 forward/같은방향 누적 가정).
- **`modff` 이식성:** 보드 런타임 라이브러리 확인 필요(8장).
- **`while(~motionDone)` 금지:** 비트 NOT은 항상 참 → 무한루프. `while(!motionDone)`.
- **디버그 출력(DBGV2):** ISR 주기(10µs) 내 UART 출력 과다 시 다음 tick을 놓침 → main 루프에서 플래그 보고 출력하는 현재 구조 유지. 또한 `WaitMotionDone(int,int)`의 파라미터는 전역을 가려(shadow) 호출시점 스냅샷이 됨 — 디버그 시 전역 직접 참조/포인터 사용.
