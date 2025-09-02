// === 보드 호환: ESP 전용 IRAM_ATTR가 없으면 빈 매크로로 처리 ===
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

/***** 모터 핀맵 (최신) *****/
// 모터1 (L298N #1)
const int ENA1  = 10;  // PWM
const int IN1_1 = 22;
const int IN2_1 = 23;

// 모터2 (L298N #2)
const int ENB2  = 7;   // PWM
const int IN3_2 = 26;
const int IN4_2 = 27;

// 모터3 (L298N #2)  // ⚠ ENA2=5 (3→5로 변경: ENC1_B=3과 충돌 방지)
const int ENA2  = 5;   // PWM
const int IN1_2 = 30;
const int IN2_2 = 31;

/***** 엔코더 핀맵 *****/
// M1: A=2,  B=3
// M2: A=19, B=18
// M3: A=21, B=20
const int ENC_A[3] = {2, 19, 21};
const int ENC_B[3] = {3, 18, 20};

// 엔코더 해상도(카운트/rev)
const float COUNTS_PER_REV = 600.0f;

/***** 출력 주기(속도 샘플링 간격) *****/
const unsigned long SAMPLE_MS = 500;

/***** 모터별 극성(+1 동일, -1 반대) — 테스트 후 맞춰주세요 *****/
int kPol[3] = { +1, +1, -1 };  // 예: {+1, -1, +1}

/***** 부팅 시 적용할 기본 PWM (원하면 0으로 시작 후 시리얼에서 s로 설정) *****/
int pwmCmd[3] = { 120, 120, 120 }; // -255~255

/***** 내부 상태 *****/
const int EN[3]  = { ENA1, ENB2, ENA2 };
const int INA[3] = { IN1_1, IN3_2, IN1_2 };
const int INB[3] = { IN2_1, IN4_2, IN2_2 };

volatile long encCnt[3] = {0,0,0};  // 누적 카운트
long prevCnt[3] = {0,0,0};
unsigned long tPrev = 0;

int clamp255(long v){ return (int)max(-255L, min(255L, v)); }

void writeMotorPWM(int idx, int pwmSigned){
  int mag = abs(pwmSigned);
  if (pwmSigned > 0){
    digitalWrite(INA[idx], HIGH);
    digitalWrite(INB[idx], LOW);
  } else if (pwmSigned < 0){
    digitalWrite(INA[idx], LOW);
    digitalWrite(INB[idx], HIGH);
  } else {
    digitalWrite(INA[idx], LOW);
    digitalWrite(INB[idx], LOW); // coast
  }
  analogWrite(EN[idx], mag);
}

void applyAllPWM(){
  for(int i=0;i<3;i++){
    int out = clamp255(kPol[i]*pwmCmd[i]);
    writeMotorPWM(i, out);
  }
}

void stopAll(){
  pwmCmd[0]=pwmCmd[1]=pwmCmd[2]=0;
  applyAllPWM();
}

// ==== ISR 프로토타입 ====
void IRAM_ATTR enc_isr0();
void IRAM_ATTR enc_isr1();
void IRAM_ATTR enc_isr2();

/***** 쿼드러처: A CHANGE, B 읽어 방향판정 *****/
void IRAM_ATTR enc_isr0(){
  int a = digitalRead(ENC_A[0]);
  int b = digitalRead(ENC_B[0]);
  if (a == b) encCnt[0]++; else encCnt[0]--;
}
void IRAM_ATTR enc_isr1(){
  int a = digitalRead(ENC_A[1]);
  int b = digitalRead(ENC_B[1]);
  if (a == b) encCnt[1]++; else encCnt[1]--;
}
void IRAM_ATTR enc_isr2(){
  int a = digitalRead(ENC_A[2]);
  int b = digitalRead(ENC_B[2]);
  if (a == b) encCnt[2]++; else encCnt[2]--;
}

void printHelp(){
  Serial.println(F("== 3-모터 엔코더 속도 모니터 =="));
  Serial.println(F("명령:"));
  Serial.println(F("  s p1 p2 p3   -> 각 모터 PWM 설정 (예: s 120 120 120)"));
  Serial.println(F("  0            -> 모든 모터 정지"));
  Serial.println(F("극성 kPol: {+1, -1}로 모터 배선 방향 맞추기"));
}

void setup(){
  Serial.begin(115200);

  // 핀 설정
  for(int i=0;i<3;i++){
    pinMode(EN[i], OUTPUT);
    pinMode(INA[i], OUTPUT);
    pinMode(INB[i], OUTPUT);
    analogWrite(EN[i], 0);           // 안전: EN을 0으로
    digitalWrite(INA[i], LOW);
    digitalWrite(INB[i], LOW);
  }

  for(int i=0;i<3;i++){
    pinMode(ENC_A[i], INPUT_PULLUP);
    pinMode(ENC_B[i], INPUT_PULLUP);
  }

  // 인터럽트 등록 (A채널 CHANGE)
  attachInterrupt(digitalPinToInterrupt(ENC_A[0]), enc_isr0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A[1]), enc_isr1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A[2]), enc_isr2, CHANGE);

  printHelp();

  // ⬇⬇⬇ 핵심: 부팅 시 원하는 PWM을 실제로 적용
  applyAllPWM();

  tPrev = millis();
}

void handleSerial(){
  if(!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if(line.length()==0) return;

  if(line == "0"){
    stopAll();
    Serial.println(F("STOP ALL (PWM=0)"));
    return;
  }

  if(line[0]=='s' || line[0]=='S'){
    int p1, p2, p3;
    if (sscanf(line.c_str()+1, "%d %d %d", &p1, &p2, &p3) == 3){
      pwmCmd[0]=p1; pwmCmd[1]=p2; pwmCmd[2]=p3;
      applyAllPWM();
      Serial.print(F("SET PWM = "));
      Serial.print(pwmCmd[0]); Serial.print(',');
      Serial.print(pwmCmd[1]); Serial.print(',');
      Serial.println(pwmCmd[2]);
    } else {
      Serial.println(F("형식: s p1 p2 p3  (예: s 120 120 120)"));
    }
    return;
  }

  // "120 120 120" 혹은 "120,120,120" 형식도 허용
  {
    int p1, p2, p3;
    if (sscanf(line.c_str(), "%d %d %d", &p1, &p2, &p3) == 3 ||
        sscanf(line.c_str(), "%d,%d,%d", &p1, &p2, &p3) == 3){
      pwmCmd[0]=p1; pwmCmd[1]=p2; pwmCmd[2]=p3;
      applyAllPWM();
      Serial.print(F("SET PWM = "));
      Serial.print(pwmCmd[0]); Serial.print(',');
      Serial.print(pwmCmd[1]); Serial.print(',');
      Serial.println(pwmCmd[2]);
      return;
    }
  }

  Serial.println(F("알 수 없는 명령. 예: s 120 120 120  또는 0"));
}

void loop(){
  handleSerial();

  unsigned long now = millis();
  if (now - tPrev >= SAMPLE_MS){
    float dt = (now - tPrev) / 1000.0f;
    tPrev = now;

    long c[3];
    noInterrupts();
    c[0] = encCnt[0];
    c[1] = encCnt[1];
    c[2] = encCnt[2];
    interrupts();

    long d0 = c[0] - prevCnt[0];
    long d1 = c[1] - prevCnt[1];
    long d2 = c[2] - prevCnt[2];
    prevCnt[0] = c[0];
    prevCnt[1] = c[1];
    prevCnt[2] = c[2];

    float cps1 = d0 / dt, cps2 = d1 / dt, cps3 = d2 / dt;
    float rpm1 = cps1 * 60.0f / COUNTS_PER_REV;
    float rpm2 = cps2 * 60.0f / COUNTS_PER_REV;
    float rpm3 = cps3 * 60.0f / COUNTS_PER_REV;

    char dir1 = (rpm1>=0)? 'F':'R';
    char dir2 = (rpm2>=0)? 'F':'R';
    char dir3 = (rpm3>=0)? 'F':'R';

    int out1 = clamp255(kPol[0]*pwmCmd[0]);
    int out2 = clamp255(kPol[1]*pwmCmd[1]);
    int out3 = clamp255(kPol[2]*pwmCmd[2]);

    Serial.print(F("PWM=M1:")); Serial.print(out1);
    Serial.print(F(" M2:"));     Serial.print(out2);
    Serial.print(F(" M3:"));     Serial.print(out3);

    Serial.print(F(" | M1 cps=")); Serial.print(cps1,1);
    Serial.print(F(" rpm="));      Serial.print(rpm1,1);
    Serial.print(F(" dir="));      Serial.print(dir1);

    Serial.print(F(" | M2 cps=")); Serial.print(cps2,1);
    Serial.print(F(" rpm="));      Serial.print(rpm2,1);
    Serial.print(F(" dir="));      Serial.print(dir2);

    Serial.print(F(" | M3 cps=")); Serial.print(cps3,1);
    Serial.print(F(" rpm="));      Serial.print(rpm3,1);
    Serial.print(F(" dir="));      Serial.print(dir3);

    Serial.print(F(" | cnts: "));
    Serial.print(c[0]); Serial.print(',');
    Serial.print(c[1]); Serial.print(',');
    Serial.println(c[2]);
  }
}
