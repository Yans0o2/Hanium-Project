// === 3모터 PID 속도제어 (Encoder-only) + 램프 + PWM 슬루 + 램프 중 I동결 ===
// + 타임라인: WAIT(40s) → RUN(70s) → STOP
// + Anti-chatter pack: 최소구동PWM, 온/오프 유지시간, 방향반전 가드
// 출력: "M1 tgt/appl / rpm / pwm / Err=±.. | ...  Kp.. Ki.. Kd.. ramp.. slew.. [IRQ/s=..,..,..] | phase=.. t=.."

#include <Arduino.h>
 
/* ================= TIMING ================= */
const float WAIT_SECONDS = 40.0f;
const float RUN_SECONDS  = 70.0f;

/* ================= USER CONFIG ================= */
float Kp = 4.0f;
float Ki = 1.1f;
float Kd = 0.15f;

// 램프(목표 완만 상승) — RUN 진입 순간부터 시작
float RAMP_TIME_S = 0.7f;
bool  FREEZE_I_WHILE_RAMP = true;

// PWM 슬루(한 사이클당 변화 최대치)
const int PWM_SLEW_PER_CYCLE = 50;

// 목표 RPM
const bool  USE_SINGLE_TARGET = true;
const float TARGET_RPM_ALL = 200.0f;
const float TARGET_RPM_1   = 200.0f;
const float TARGET_RPM_2   = 200.0f;
const float TARGET_RPM_3   = 200.0f;

// 주기/필터
const unsigned long SPEED_DT_MS = 50;
const unsigned long CTRL_DT_MS  = 50;
const unsigned long PRINT_DT_MS = 200;
const float RPM_LPF_ALPHA = 0.5f;
const float D_LPF_ALPHA   = 0.7f;

// PID 기타
const float I_CLAMP = 150.0f;
const bool  ANTI_WINDUP_ON_SAT = true;
const int   PWM_MAX = 255;
const int   PWM_DEADBAND = 0;

/* ===== Anti-chatter 추가 파라미터 ===== */
const int   PWM_MIN_RUN          = 25;   // 비0이면 최소 이 정도는 주기(정지마찰 극복)
const unsigned long MIN_ON_TIME_MS  = 150; // 비0 유지 최소 시간
const unsigned long MIN_OFF_TIME_MS = 120; // 0 유지 최소 시간
const unsigned long SIGN_FLIP_GUARD_MS = 150; // 반전 전 0으로 머무는 시간
const float ERR_DEADBAND_RPM = 5.0f;     // 아주 작은 오차는 0 취급(불필요한 들썩임 방지)

/* ================= PIN MAP ================= */
const int ENA1  = 10;   // PWM (M1)
const int IN1_1 = 22;
const int IN2_1 = 23;

const int ENB2  = 7;    // PWM (M2)
const int IN3_2 = 26;
const int IN4_2 = 27;

const int ENA2  = 5;    // PWM (M3)
const int IN1_2 = 30;
const int IN2_2 = 31;

// Encoders
const int ENC1_A = 2;   const int ENC1_B = 36;
const int ENC2_A = 3;   const int ENC2_B = 38;
const int ENC3_A = 18;  const int ENC3_B = 40;

// Encoder params
float COUNTS_PER_REV_1 = 300.0f;
float COUNTS_PER_REV_2 = 300.0f;
float COUNTS_PER_REV_3 = 300.0f;

const unsigned long EDGE_DEGLITCH_US = 120;
// 하드웨어에 맞게 부호 반전
bool DIR_INV[3] = { true, true, true };

/* ================= STATE ================= */
volatile long encCount1=0, encCount2=0, encCount3=0;
volatile unsigned long lastEdgeUs1=0, lastEdgeUs2=0, lastEdgeUs3=0;
volatile unsigned long irqCount1=0, irqCount2=0, irqCount3=0;

long lastCount1=0, lastCount2=0, lastCount3=0;

float rpm[3]   = {0,0,0};  // raw
float rpm_f[3] = {0,0,0};  // filtered

// 목표 & 램프
float userTarget[3]   = {0,0,0};
float appliedTarget[3]= {0,0,0};
float rampStartRPM[3] = {0,0,0};
unsigned long rampStartMs[3] = {0,0,0};

// PID 내부
float Iterm[3]    = {0,0,0};
float prevMeas[3] = {0,0,0};
float dMeas_f[3]  = {0,0,0};
int   pwmCmd[3]   = {0,0,0};
int   pwmPrev[3]  = {0,0,0};

// Anti-chatter 상태
int lastSign[3] = {0,0,0};                      // -1/0/+1 (최종 출력 부호)
unsigned long lastSignChangeMs[3] = {0,0,0};    // 마지막 비0 부호 변경 시각
unsigned long lastZeroMs[3]       = {0,0,0};    // 마지막 0 출력 시각

unsigned long lastSpeedMs=0, lastCtrlMs=0, lastPrintMs=0, lastIRQPrintMs=0;
bool PRINT_IRQ_RATE = true;

/* ================= Timeline ================= */
enum Phase { WAIT=0, RUN=1, STOP=2 };
Phase phase = WAIT;
unsigned long phaseStartMs = 0;

/* ================= UTIL ================= */
inline int clampInt(int v,int lo,int hi){ if(v<lo) return lo; if(v>hi) return hi; return v; }
inline float clampf(float v,float lo,float hi){ if(v<lo) return lo; if(v>hi) return hi; return v; }
void printSigned(float v, uint8_t prec){ if (v>=0) Serial.print('+'); Serial.print(v,prec); }
int slewTo(int prev, int target, int step){
  if (target > prev + step) return prev + step;
  if (target < prev - step) return prev - step;
  return target;
}
inline int sgn(int x){ return (x>0) - (x<0); }

void setMotorRaw(int pwmPin, int inA, int inB, int pwmSigned){
  int mag = pwmSigned; if(mag<0) mag=-mag; if(mag>255) mag=255;
  if (pwmSigned>=0){ digitalWrite(inA,HIGH); digitalWrite(inB,LOW); }
  else             { digitalWrite(inA,LOW);  digitalWrite(inB,HIGH); }
  analogWrite(pwmPin, mag);
}
void setMotor(int idx, int pwmSigned){
  switch(idx){
    case 1: setMotorRaw(ENA1, IN1_1, IN2_1, pwmSigned); break;
    case 2: setMotorRaw(ENB2, IN3_2, IN4_2, pwmSigned); break;
    case 3: setMotorRaw(ENA2, IN1_2, IN2_2, pwmSigned); break;
  }
}

/* ================= ENCODER ISR ================= */
void enc1_isr(){ unsigned long now=micros(); if(now-lastEdgeUs1<EDGE_DEGLITCH_US) return; lastEdgeUs1=now;
  int b=digitalRead(ENC1_B); int d=(b?-1:+1); if(DIR_INV[0]) d=-d; encCount1+=d; irqCount1++; }
void enc2_isr(){ unsigned long now=micros(); if(now-lastEdgeUs2<EDGE_DEGLITCH_US) return; lastEdgeUs2=now;
  int b=digitalRead(ENC2_B); int d=(b?-1:+1); if(DIR_INV[1]) d=-d; encCount2+=d; irqCount2++; }
void enc3_isr(){ unsigned long now=micros(); if(now-lastEdgeUs3<EDGE_DEGLITCH_US) return; lastEdgeUs3=now;
  int b=digitalRead(ENC3_B); int d=(b?-1:+1); if(DIR_INV[2]) d=-d; encCount3+=d; irqCount3++; }

/* ================= SPEED ESTIMATION ================= */
void updateSpeeds(){
  unsigned long now = millis();
  unsigned long dt = now - lastSpeedMs;
  if (dt < SPEED_DT_MS) return;
  lastSpeedMs = now;

  long c1,c2,c3; noInterrupts(); c1=encCount1; c2=encCount2; c3=encCount3; interrupts();
  long d1=c1-lastCount1; lastCount1=c1;
  long d2=c2-lastCount2; lastCount2=c2;
  long d3=c3-lastCount3; lastCount3=c3;

  const float k = 1000.0f / (float)dt;
  float cps1=d1*k, cps2=d2*k, cps3=d3*k;

  rpm[0] = (COUNTS_PER_REV_1>0 ? (cps1/COUNTS_PER_REV_1)*60.0f : 0.0f);
  rpm[1] = (COUNTS_PER_REV_2>0 ? (cps2/COUNTS_PER_REV_2)*60.0f : 0.0f);
  rpm[2] = (COUNTS_PER_REV_3>0 ? (cps3/COUNTS_PER_REV_3)*60.0f : 0.0f);

  for(int i=0;i<3;i++){
    rpm_f[i] = RPM_LPF_ALPHA*rpm_f[i] + (1.0f-RPM_LPF_ALPHA)*rpm[i];
  }
}

/* ================= RAMP (S-curve) ================= */
bool rampActive(int i, unsigned long now){
  float Tms = RAMP_TIME_S * 1000.0f;
  return (Tms > 1.0f) && ((now - rampStartMs[i]) < Tms);
}
void updateAppliedTargets(){
  unsigned long now = millis();
  float Tms = RAMP_TIME_S * 1000.0f;
  for(int i=0;i<3;i++){
    if (Tms <= 1.0f){ appliedTarget[i] = userTarget[i]; continue; }
    float t = clampf((float)(now - rampStartMs[i]) / Tms, 0.0f, 1.0f);
    float s = t*t*(3.0f - 2.0f*t); // S-curve
    appliedTarget[i] = rampStartRPM[i] + s*(userTarget[i] - rampStartRPM[i]);
  }
}

/* ================= PID + Anti-chatter ================= */
void controlStep(int i, float sp, float meas, float meas_prev, float dt, bool freezeI, unsigned long nowMs){
  // 작은 오차는 0 취급(불필요한 들썩임 방지)
  float err = sp - meas;
  if (fabs(err) < ERR_DEADBAND_RPM) err = 0.0f;

  // I (anti-windup + 램프 중 동결)
  if (!freezeI && (!ANTI_WINDUP_ON_SAT || (abs(pwmCmd[i]) < PWM_MAX-1))) {
    Iterm[i] += err * dt;
    Iterm[i] = clampf(Iterm[i], -I_CLAMP, +I_CLAMP);
  }

  // D = d(meas)/dt (LPF)
  float dMeas = (meas - meas_prev) / dt;
  dMeas_f[i] = D_LPF_ALPHA*dMeas_f[i] + (1.0f-D_LPF_ALPHA)*dMeas;

  float u = Kp*err + Ki*Iterm[i] - Kd*dMeas_f[i];

  // 데드밴드 & 포화
  int cmd = (int)round(u);
  if (abs(cmd) < PWM_DEADBAND) cmd = 0;
  cmd = clampInt(cmd, -PWM_MAX, PWM_MAX);

  // PWM 슬루 제한
  cmd = slewTo(pwmPrev[i], cmd, PWM_SLEW_PER_CYCLE);

  // ===== Anti-chatter 처리 시작 =====
  int desiredSign = sgn(cmd);
  int prevSign    = sgn(pwmPrev[i]);

  // 1) 방향반전 가드: 반전 전에 반드시 0 상태로 SIGN_FLIP_GUARD_MS 유지
  if (desiredSign != 0 && lastSign[i] != 0 && desiredSign != lastSign[i]) {
    // 반전 의도 → 우선 0 출력으로 전환
    cmd = 0;
    desiredSign = 0;
  }

  // 2) 최소 ON/OFF 유지시간 + 최소구동PWM
  if (desiredSign == 0) {
    // 0 상태 유지시간 확보
    if ((nowMs - lastZeroMs[i]) < MIN_OFF_TIME_MS) {
      cmd = 0; // 강제 유지
    }
  } else {
    // 비0 상태 — 최소 구동 PWM 보장
    if (abs(cmd) < PWM_MIN_RUN) cmd = desiredSign * PWM_MIN_RUN;

    // 켠 지 얼마 안 됐으면 너무 빨리 0으로 떨어지지 않도록 유지
    if (lastSign[i] == desiredSign && (nowMs - lastSignChangeMs[i]) < MIN_ON_TIME_MS) {
      if (abs(cmd) < PWM_MIN_RUN) cmd = desiredSign * PWM_MIN_RUN;
    }
  }

  // 최종 포화
  cmd = clampInt(cmd, -PWM_MAX, PWM_MAX);

  // ===== 상태 갱신(부호/시간) =====
  int finalSign = sgn(cmd);
  if (finalSign == 0 && pwmPrev[i] != 0) {
    lastZeroMs[i] = nowMs;               // 0으로 막 떨어진 시점
  }
  if (finalSign != 0 && finalSign != lastSign[i]) {
    lastSign[i] = finalSign;
    lastSignChangeMs[i] = nowMs;         // 비0 부호가 바뀐 시점(ON 시작 포함)
  }
  // ===== Anti-chatter 처리 끝 =====

  pwmPrev[i] = cmd;
  pwmCmd[i]  = cmd;
  setMotor(i+1, pwmCmd[i]);
}

void controlPID(){
  unsigned long now = millis();
  static unsigned long last = 0;
  if (now - last < CTRL_DT_MS) return;
  float dt = (last==0) ? (CTRL_DT_MS/1000.0f) : ((now - last)/1000.0f);
  last = now;

  updateAppliedTargets();

  bool f1 = FREEZE_I_WHILE_RAMP && rampActive(0, now);
  bool f2 = FREEZE_I_WHILE_RAMP && rampActive(1, now);
  bool f3 = FREEZE_I_WHILE_RAMP && rampActive(2, now);

  controlStep(0, appliedTarget[0], rpm_f[0], prevMeas[0], dt, f1, now);
  controlStep(1, appliedTarget[1], rpm_f[1], prevMeas[1], dt, f2, now);
  controlStep(2, appliedTarget[2], rpm_f[2], prevMeas[2], dt, f3, now);

  prevMeas[0]=rpm_f[0]; prevMeas[1]=rpm_f[1]; prevMeas[2]=rpm_f[2];
}

/* ================= Timeline helpers ================= */
void enterWait(){
  phase = WAIT; phaseStartMs = millis();
  setMotor(1,0); setMotor(2,0); setMotor(3,0);
  for(int i=0;i<3;i++){ Iterm[i]=0; dMeas_f[i]=0; pwmCmd[i]=0; pwmPrev[i]=0; lastSign[i]=0; lastSignChangeMs[i]=0; lastZeroMs[i]=millis(); }
}
void enterRun(){
  phase = RUN; phaseStartMs = millis();
  // RUN 진입 순간 속도에서 램프 시작
  rampStartRPM[0]=rpm_f[0]; rampStartRPM[1]=rpm_f[1]; rampStartRPM[2]=rpm_f[2];
  rampStartMs[0]=rampStartMs[1]=rampStartMs[2]=millis();
  // D/상태 초기화
  prevMeas[0]=rpm_f[0]; prevMeas[1]=rpm_f[1]; prevMeas[2]=rpm_f[2];
  for(int i=0;i<3;i++){ Iterm[i]=0; dMeas_f[i]=0; pwmPrev[i]=0; lastSign[i]=0; lastSignChangeMs[i]=0; lastZeroMs[i]=millis(); }
}
void enterStop(){
  phase = STOP; phaseStartMs = millis();
  setMotor(1,0); setMotor(2,0); setMotor(3,0);
}

/* ================= PRINT ================= */
const __FlashStringHelper* phaseName(Phase p){
  switch(p){ case WAIT: return F("WAIT"); case RUN: return F("RUN"); case STOP: return F("STOP"); }
  return F("?");
}
void maybePrint(){
  unsigned long now = millis();
  if (now - lastPrintMs < PRINT_DT_MS) return;
  lastPrintMs = now;

  float e1 = userTarget[0] - rpm[0];
  float e2 = userTarget[1] - rpm[1];
  float e3 = userTarget[2] - rpm[2];

  Serial.print(F("M1 "));
  Serial.print(userTarget[0],1); Serial.print('/');
  Serial.print(appliedTarget[0],1); Serial.print(F(" / "));
  Serial.print(rpm[0],1);    Serial.print(F(" / "));
  Serial.print(pwmCmd[0]);   Serial.print(F(" / Err="));
  printSigned(e1, 1);

  Serial.print(F(" | M2 "));
  Serial.print(userTarget[1],1); Serial.print('/');
  Serial.print(appliedTarget[1],1); Serial.print(F(" / "));
  Serial.print(rpm[1],1);    Serial.print(F(" / "));
  Serial.print(pwmCmd[1]);   Serial.print(F(" / Err="));
  printSigned(e2, 1);

  Serial.print(F(" | M3 "));
  Serial.print(userTarget[2],1); Serial.print('/');
  Serial.print(appliedTarget[2],1); Serial.print(F(" / "));
  Serial.print(rpm[2],1);    Serial.print(F(" / "));
  Serial.print(pwmCmd[2]);   Serial.print(F(" / Err="));
  printSigned(e3, 1);

  Serial.print(F("  Kp=")); Serial.print(Kp,2);
  Serial.print(F(" Ki="));  Serial.print(Ki,3);
  Serial.print(F(" Kd="));  Serial.print(Kd,3);
  Serial.print(F(" ramp=")); Serial.print(RAMP_TIME_S,2);
  Serial.print(F(" slew=")); Serial.print(PWM_SLEW_PER_CYCLE);

  if (PRINT_IRQ_RATE && (now - lastIRQPrintMs >= 1000)) {
    lastIRQPrintMs = now;
    static unsigned long li1=0,li2=0,li3=0;
    unsigned long i1,i2,i3; noInterrupts(); i1=irqCount1; i2=irqCount2; i3=irqCount3; interrupts();
    unsigned long di1=i1-li1, di2=i2-li2, di3=i3-li3;
    li1=i1; li2=i2; li3=i3;
    Serial.print(F("  [IRQ/s=")); Serial.print(di1); Serial.print(',');
    Serial.print(di2); Serial.print(','); Serial.print(di3); Serial.print(']');
  }

  Serial.print(F("  | phase=")); Serial.print(phaseName(phase));
  Serial.print(F(" t=")); Serial.print((millis()-phaseStartMs)/1000.0f,1);
  Serial.println();
}

/* ================= SETUP/LOOP ================= */
void setup(){
  Serial.begin(115200);

  pinMode(ENA1,OUTPUT); pinMode(IN1_1,OUTPUT); pinMode(IN2_1,OUTPUT);
  pinMode(ENB2,OUTPUT); pinMode(IN3_2,OUTPUT); pinMode(IN4_2,OUTPUT);
  pinMode(ENA2,OUTPUT); pinMode(IN1_2,OUTPUT); pinMode(IN2_2,OUTPUT);
  setMotor(1,0); setMotor(2,0); setMotor(3,0);

  pinMode(ENC1_A,INPUT_PULLUP); pinMode(ENC1_B,INPUT_PULLUP);
  pinMode(ENC2_A,INPUT_PULLUP); pinMode(ENC2_B,INPUT_PULLUP);
  pinMode(ENC3_A,INPUT_PULLUP); pinMode(ENC3_B,INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC1_A), enc1_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), enc2_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC3_A), enc3_isr, RISING);

  delay(200);

  // 목표 지정
  if (USE_SINGLE_TARGET) {
    userTarget[0]=userTarget[1]=userTarget[2]=TARGET_RPM_ALL;
  } else {
    userTarget[0]=TARGET_RPM_1; userTarget[1]=TARGET_RPM_2; userTarget[2]=TARGET_RPM_3;
  }

  enterWait();

  Serial.println(F("== PID speed control + ramp + slew + anti-chatter + timeline =="));
}

void loop(){
  updateSpeeds();

  unsigned long now = millis();
  switch(phase){
    case WAIT:
      setMotor(1,0); setMotor(2,0); setMotor(3,0);
      if (now - phaseStartMs >= (unsigned long)(WAIT_SECONDS*1000.0f)) enterRun();
      break;

    case RUN:
      if (now - phaseStartMs >= (unsigned long)(RUN_SECONDS*1000.0f)) enterStop();
      else controlPID();
      break;

    case STOP:
      setMotor(1,0); setMotor(2,0); setMotor(3,0);
      break;
  }

  maybePrint();
}
