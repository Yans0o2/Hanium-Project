// === 3모터 동일 PWM + 엔코더 속도 모니터 (Arduino Mega) === 
// - 인터럽트: A핀 CHANGE, B핀 상태로 방향 판별(쿼드라처, X2)
// - 출력: 500ms마다 cps(Counts/s), rpm, dir(F/R)
// - Mega 외부인터럽트: 2(INT0), 3(INT1), 18(INT5), 19(INT4), 20(INT3), 21(INT2)

// ===== 모터 핀맵 =====
// 모터1 (L298N #1 단독)
const int ENA1  = 10;   // PWM
const int IN1_1 = 22;
const int IN2_1 = 23;

// 모터2 (L298N #2)
const int ENB2  = 7;    // PWM
const int IN3_2 = 26;
const int IN4_2 = 27;

// 모터3 (L298N #2)
// ⚠ 최근 변경: ENA2=5 (ENC1_B=3 충돌 회피)
const int ENA2  = 5;    // PWM
const int IN1_2 = 30;
const int IN2_2 = 31;

// ===== 엔코더 핀맵 (최근 버전) =====
// 1모터당: A=인터럽트핀, B=일반 디지털핀
// A: {2(INT0), 3(INT1), 18(INT5)} / B: {36, 38, 40}
const int ENC1_A = 2;    // 외부인터럽트
const int ENC1_B = 36;   // 일반 디지털
const int ENC2_A = 3;    // 외부인터럽트
const int ENC2_B = 38;   // 일반 디지털
const int ENC3_A = 18;   // 외부인터럽트
const int ENC3_B = 40;   // 일반 디지털

// ===== 설정 =====
int TEST_PWM = 150;                      // 세 모터 동일 PWM ( -255 ~ +255 )
const unsigned long INTERVAL_MS = 500;   // 출력 주기(ms)

// A핀 CHANGE(X2) 기준 한 바퀴당 카운트 수
//  COUNTS_PER_REV = PPR(A채널) × 2 × 기어비
const float COUNTS_PER_REV_1 = 600.0;
const float COUNTS_PER_REV_2 = 600.0;
const float COUNTS_PER_REV_3 = 600.0;

// ===== 내부 변수 =====
volatile long encCount1 = 0;
volatile long encCount2 = 0;
volatile long encCount3 = 0;

long lastCount1 = 0, lastCount2 = 0, lastCount3 = 0;
unsigned long lastPrintMs = 0;

// ===== 모터 제어 유틸 =====
void setMotorRaw(int pwmPin, int inA, int inB, int pwmSigned) {
  int mag = pwmSigned;
  if (mag < 0) mag = -mag;
  if (mag > 255) mag = 255;

  if (pwmSigned >= 0) {
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
  } else {
    digitalWrite(inA, LOW);
    digitalWrite(inB, HIGH);
  }
  analogWrite(pwmPin, mag);
}

void setMotor(int idx, int pwmSigned) {
  switch (idx) {
    case 1: setMotorRaw(ENA1, IN1_1, IN2_1, pwmSigned); break;
    case 2: setMotorRaw(ENB2, IN3_2, IN4_2, pwmSigned); break;
    case 3: setMotorRaw(ENA2, IN1_2, IN2_2, pwmSigned); break;
  }
}

// ===== 인터럽트 서비스 루틴 (A CHANGE, B로 방향) =====
void enc1_isr() {
  int a = digitalRead(ENC1_A);
  int b = digitalRead(ENC1_B);
  if (a == b) encCount1++; else encCount1--;
}
void enc2_isr() {
  int a = digitalRead(ENC2_A);
  int b = digitalRead(ENC2_B);
  if (a == b) encCount2++; else encCount2--;
}
void enc3_isr() {
  int a = digitalRead(ENC3_A);
  int b = digitalRead(ENC3_B);
  if (a == b) encCount3++; else encCount3--;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  // 모터 핀
  pinMode(ENA1, OUTPUT);  pinMode(IN1_1, OUTPUT);  pinMode(IN2_1, OUTPUT);
  pinMode(ENB2, OUTPUT);  pinMode(IN3_2, OUTPUT);  pinMode(IN4_2, OUTPUT);
  pinMode(ENA2, OUTPUT);  pinMode(IN1_2, OUTPUT);  pinMode(IN2_2, OUTPUT);

  // 엔코더 핀
  pinMode(ENC1_A, INPUT_PULLUP);  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);  pinMode(ENC2_B, INPUT_PULLUP);
  pinMode(ENC3_A, INPUT_PULLUP);  pinMode(ENC3_B, INPUT_PULLUP);

  // 인터럽트 연결 (A핀만 사용)
  attachInterrupt(digitalPinToInterrupt(ENC1_A), enc1_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), enc2_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC3_A), enc3_isr, CHANGE);

  // 초기 정지
  setMotor(1, 0); setMotor(2, 0); setMotor(3, 0);

  delay(500);
  Serial.println(F("== 3-모터 엔코더 속도 모니터 (INT+DIGITAL per motor) =="));
  Serial.println(F("명령: '+'/'-' PWM±1, '0' 정지, 'r' 역방향 토글"));
}

void loop() {
  // 런타임 PWM 조정
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '+') { TEST_PWM += 1; if (TEST_PWM > 255) TEST_PWM = 255; }
    else if (c == '-') { TEST_PWM -= 1; if (TEST_PWM < -255) TEST_PWM = -255; }
    else if (c == '0') { TEST_PWM = 0; }
    else if (c == 'r' || c == 'R') { TEST_PWM = -TEST_PWM; }
  }

  // 동일 PWM 적용 (필요하면 아래 라인만 조정)
  setMotor(1, TEST_PWM);
  setMotor(2, TEST_PWM);
  setMotor(3, TEST_PWM);

  // 주기 출력
  unsigned long now = millis();
  if (now - lastPrintMs >= INTERVAL_MS) {
    lastPrintMs = now;

    long c1, c2, c3;
    noInterrupts();
    c1 = encCount1; c2 = encCount2; c3 = encCount3;
    interrupts();

    long d1 = c1 - lastCount1; lastCount1 = c1;
    long d2 = c2 - lastCount2; lastCount2 = c2;
    long d3 = c3 - lastCount3; lastCount3 = c3;

    const float k = 1000.0f / (float)INTERVAL_MS; // ms → s
    float cps1 = d1 * k, cps2 = d2 * k, cps3 = d3 * k;

    float rpm1 = (COUNTS_PER_REV_1 > 0) ? (cps1 / COUNTS_PER_REV_1 * 60.0f) : 0.0f;
    float rpm2 = (COUNTS_PER_REV_2 > 0) ? (cps2 / COUNTS_PER_REV_2 * 60.0f) : 0.0f;
    float rpm3 = (COUNTS_PER_REV_3 > 0) ? (cps3 / COUNTS_PER_REV_3 * 60.0f) : 0.0f;

    char dir1 = (d1 >= 0) ? 'F' : 'R';
    char dir2 = (d2 >= 0) ? 'F' : 'R';
    char dir3 = (d3 >= 0) ? 'F' : 'R';

    Serial.print(F("PWM=")); Serial.print(TEST_PWM);
    Serial.print(F(" | M1 cps=")); Serial.print(cps1, 1);
    Serial.print(F(" rpm=")); Serial.print(rpm1, 1);
    Serial.print(F(" dir=")); Serial.print(dir1);

    Serial.print(F(" | M2 cps=")); Serial.print(cps2, 1);
    Serial.print(F(" rpm=")); Serial.print(rpm2, 1);
    Serial.print(F(" dir=")); Serial.print(dir2);

    Serial.print(F(" | M3 cps=")); Serial.print(cps3, 1);
    Serial.print(F(" rpm=")); Serial.print(rpm3, 1);
    Serial.print(F(" dir=")); Serial.print(dir3);

    Serial.print(F(" | cnts: ")); Serial.print(c1); Serial.print(',');
    Serial.print(c2); Serial.print(','); Serial.println(c3);
  }
}
