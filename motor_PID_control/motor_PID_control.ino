// === 3모터 PID 속도제어 (Encoder-only) + 램프 + PWM 슬루 + 램프 중 I동결 ===
// - 제어는 rpm_f(필터) 사용, 표시는 raw rpm
// - 목표는 현재속도→사용자목표로 RAMP_TIME_S 동안 S-curve로 상승
// - 출력 형식: "M1 tgt/appl / rpm / pwm / Err=±.. | M2 ... | M3 ...  Kp.. Ki.. Kd.. ramp.. slew.. [IRQ/s=..,..,..]"

#include <Arduino.h>

// ================= USER CONFIG =================
float Kp = 2.5f;
float Ki = 0.2f;     // 처음엔 0으로 시작 권장. 튀면 0으로.
float Kd = 0.08f;    // 튀면 0으로 조금씩 올리기 (0.02~0.1)

// 램프(목표 완만 상승)
float RAMP_TIME_S = 2.0f;   // 목표까지 도달 시간
bool  FREEZE_I_WHILE_RAMP = true; // 램프 진행 중 적분 동결

// 출력 슬루(한 사이클당 PWM 변화 최대치)
const int PWM_SLEW_PER_CYCLE = 20;      // 50ms 기준 ≈ 400 PWM/s

// 목표 RPM: 단일/모터별
const bool  USE_SINGLE_TARGET = true;
const float TARGET_RPM_ALL = 80.0f;
const float TARGET_RPM_1   = 200.0f;
const float TARGET_RPM_2   = 200.0f;
const float TARGET_RPM_3   = 200.0f;

// 주기/필터
const unsigned long SPEED_DT_MS = 50;
const unsigned long CTRL_DT_MS  = 50;
const unsigned long PRINT_DT_MS = 200;

const float RPM_LPF_ALPHA = 0.5f;   // 0~0.95 (클수록 느림/부드러움)
const float D_LPF_ALPHA   = 0.7f;   // D(측정미분) LPF

// PID 기타
const float I_CLAMP = 150.0f;
const bool  ANTI_WINDUP_ON_SAT = true;
const int   PWM_MAX = 255;
const int   PWM_DEADBAND = 0;

// ================= PIN MAP (네 최신 맵) =================
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

// ===== Encoder params =====
// A:RISING(×1) 기준 — 실제 값에 맞게 보정 필요
float COUNTS_PER_REV_1 = 300.0f;
float COUNTS_PER_REV_2 = 300.0f;
float COUNTS_PER_REV_3 = 300.0f;

const unsigned long EDGE_DEGLITCH_US = 120;
// 부호 반전(너가 요구했던 반전 유지) — 필요시 모터별 조정
bool DIR_INV[3] = { true, true, true };

// ================= STATE =================
volatile long encCount1=0, encCount2=0, encCount3=0;
volatile unsigned long lastEdgeUs1=0, lastEdgeUs2=0, lastEdgeUs3=0;
volatile unsigned long irqCount1=0, irqCount2=0, irqCount3=0;

long lastCount1=0, lastCount2=0, lastCount3=0;

float rpm[3]   = {0,0,0};  // raw (표시)
float rpm_f[3] = {0,0,0};  // 필터 (제어)

// 목표 & 램프
float userTarget[3]   = {0,0,0}; // 사용자가 의도한 최종 목표
float appliedTarget[3]= {0,0,0}; // 램프 적용된 목표(제어에 사용)
float rampStartRPM[3] = {0,0,0};
unsigned long rampStartMs[3] = {0,0,0};

// PID 내부
float Iterm[3]    = {0,0,0};
float prevMeas[3] = {0,0,0};  // D on measurement
float dMeas_f[3]  = {0,0,0};
int   pwmCmd[3]   = {0,0,0};  // 실제 출력
int   pwmPrev[3]  = {0,0,0};  // 슬루용 저장

unsigned long lastSpeedMs=0, lastCtrlMs=0, lastPrintMs=0, lastIRQPrintMs=0;
bool PRINT_IRQ_RATE = true;

// ================= UTIL =================
inline int clampInt(int v,int lo,int hi){ if(v<lo) return lo; if(v>hi) return hi; return v; }
inline float clampf(float v,float lo,float hi){ if(v<lo) return lo; if(v>hi) return hi; return v; }
void printSigned(float v, uint8_t prec){ if (v>=0) Serial.print('+'); Serial.print(v,prec); }
int slewTo(int prev, int target, int step){
  if (target > prev + step) return prev + step;
  if (target < prev - step) return prev - step;
  return target;
}

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

// ================= ENCODER ISR (A:RISING + B방향 + 디글리치) =================
void enc1_isr(){
  unsigned long now = micros();
  if (now - lastEdgeUs1 < EDGE_DEGLITCH_US) return;
  lastEdgeUs1 = now;
  int b = digitalRead(ENC1_B);
  int d = (b ? -1 : +1);
  if (DIR_INV[0]) d = -d;
  encCount1 += d; irqCount1++;
}
void enc2_isr(){
  unsigned long now = micros();
  if (now - lastEdgeUs2 < EDGE_DEGLITCH_US) return;
  lastEdgeUs2 = now;
  int b = digitalRead(ENC2_B);
  int d = (b ? -1 : +1);
  if (DIR_INV[1]) d = -d;
  encCount2 += d; irqCount2++;
}
void enc3_isr(){
  unsigned long now = micros();
  if (now - lastEdgeUs3 < EDGE_DEGLITCH_US) return;
  lastEdgeUs3 = now;
  int b = digitalRead(ENC3_B);
  int d = (b ? -1 : +1);
  if (DIR_INV[2]) d = -d;
  encCount3 += d; irqCount3++;
}

// ================= SPEED ESTIMATION =================
void updateSpeeds(){
  unsigned long now = millis();
  unsigned long dt = now - lastSpeedMs;
  if (dt < SPEED_DT_MS) return;
  lastSpeedMs = now;

  long c1,c2,c3;
  noInterrupts(); c1=encCount1; c2=encCount2; c3=encCount3; interrupts();

  long d1=c1-lastCount1; lastCount1=c1;
  long d2=c2-lastCount2; lastCount2=c2;
  long d3=c3-lastCount3; lastCount3=c3;

  const float k = 1000.0f / (float)dt; // ms→s
  float cps1=d1*k, cps2=d2*k, cps3=d3*k;

  rpm[0] = (COUNTS_PER_REV_1>0 ? (cps1/COUNTS_PER_REV_1)*60.0f : 0.0f);
  rpm[1] = (COUNTS_PER_REV_2>0 ? (cps2/COUNTS_PER_REV_2)*60.0f : 0.0f);
  rpm[2] = (COUNTS_PER_REV_3>0 ? (cps3/COUNTS_PER_REV_3)*60.0f : 0.0f);

  for(int i=0;i<3;i++){
    rpm_f[i] = RPM_LPF_ALPHA*rpm_f[i] + (1.0f-RPM_LPF_ALPHA)*rpm[i];
  }
}

// ================= RAMP (S-curve) =================
void startRampAllFromCurrent(){
  unsigned long now = millis();
  rampStartMs[0]=rampStartMs[1]=rampStartMs[2]=now;
  rampStartRPM[0]=rpm_f[0]; rampStartRPM[1]=rpm_f[1]; rampStartRPM[2]=rpm_f[2];
}
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
    // S-curve
    float s = t*t*(3.0f - 2.0f*t);
    appliedTarget[i] = rampStartRPM[i] + s*(userTarget[i] - rampStartRPM[i]);
  }
}

// ================= PID (D on measurement, slew, I freeze while ramp) =================
void controlStep(int i, float sp, float meas, float meas_prev, float dt, bool freezeI){
  float err = sp - meas;

  // I (anti-windup + 램프중 동결 옵션)
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
  pwmPrev[i] = cmd;

  pwmCmd[i] = cmd;
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

  controlStep(0, appliedTarget[0], rpm_f[0], prevMeas[0], dt, f1);
  controlStep(1, appliedTarget[1], rpm_f[1], prevMeas[1], dt, f2);
  controlStep(2, appliedTarget[2], rpm_f[2], prevMeas[2], dt, f3);

  prevMeas[0]=rpm_f[0]; prevMeas[1]=rpm_f[1]; prevMeas[2]=rpm_f[2];
}

// ================= PRINT =================
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
  Serial.println();
}

// ================= SETUP/LOOP =================
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

  // 첫 속도 샘플 몇 번 받고 램프 시작
  updateSpeeds(); delay(SPEED_DT_MS);
  updateSpeeds(); delay(SPEED_DT_MS);
  startRampAllFromCurrent();

  Serial.println(F("== PID speed control + ramp + slew (enc A:RISING, sign inverted) =="));
}

void loop(){
  updateSpeeds();
  controlPID();
  maybePrint();
}
