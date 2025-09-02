/*
  Mega 2560 + MPU-9250 (no external libs)
  - Roll/Pitch: Complementary (gyro + accel)
  - Yaw: AK8963 mag with tilt compensation (fallback: gyro integrate)
  Serial: 115200 bps
*/

#include <Wire.h>

// ---- I2C addresses ----
uint8_t MPU_ADDR = 0x68;      // AD0=GND: 0x68, AD0=VCC: 0x69
const uint8_t AK8963_ADDR = 0x0C;

// ---- MPU-9250 registers ----
const uint8_t REG_PWR_MGMT_1   = 0x6B;
const uint8_t REG_CONFIG       = 0x1A;
const uint8_t REG_GYRO_CONFIG  = 0x1B;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_ACCEL_CONFIG2= 0x1D;
const uint8_t REG_INT_PIN_CFG  = 0x37;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;
const uint8_t REG_GYRO_XOUT_H  = 0x43;

// ---- AK8963 registers ----
const uint8_t AK_REG_ST1   = 0x02;
const uint8_t AK_REG_XL    = 0x03; // X L, X H, Y L, Y H, Z L, Z H
const uint8_t AK_REG_ST2   = 0x09;
const uint8_t AK_REG_CNTL1 = 0x0A;
const uint8_t AK_REG_ASAX  = 0x10; // ASA X,Y,Z

// ---- scales ----
const float ACC_LSB_2G  = 16384.0f;   // LSB per g (±2g)
const float GYRO_LSB_250= 131.0f;     // LSB per dps (±250dps)
// AK8963 16-bit: 0.15 uT/LSB
const float MAG_LSB_16  = 0.15f;      // microTesla per LSB

// ---- filters / fusion ----
const float CF_ALPHA = 0.98f;         // complementary filter coeff
const float DECLINATION_DEG = 0.0f;   // 자기편차(도): 필요시 지역값 반영

// ---- state ----
float roll=0, pitch=0, yaw=0;         // degrees
float gBiasX=0, gBiasY=0, gBiasZ=0;   // dps bias
float magAdjX=1.0f, magAdjY=1.0f, magAdjZ=1.0f; // factory ASA
unsigned long lastMicros=0;
bool magReady=false;

static inline float rad2deg(float r){return r*57.295779513f;}
static inline float deg2rad(float d){return d*0.01745329252f;}
static inline float wrap180(float a){ while(a>180)a-=360; while(a<-180)a+=360; return a; }

// ---- I2C helpers ----
bool write8(uint8_t addr, uint8_t reg, uint8_t data){
  Wire.beginTransmission(addr);
  Wire.write(reg); Wire.write(data);
  return Wire.endTransmission()==0;
}
bool readBytes(uint8_t addr, uint8_t reg, uint8_t n, uint8_t *buf){
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false)!=0) return false;
  uint8_t got = Wire.requestFrom((int)addr, (int)n, (int)true);
  if (got!=n) return false;
  for(uint8_t i=0;i<n;i++) buf[i]=Wire.read();
  return true;
}
int16_t be16(uint8_t hi, uint8_t lo){ return (int16_t)((hi<<8)|lo); }

// ---- init MPU & AK ----
bool mpuInit(){
  // try 0x68 then 0x69
  for(uint8_t aTry=0;aTry<2;aTry++){
    uint8_t addr = (aTry==0)?0x68:0x69;
    if(write8(addr, REG_PWR_MGMT_1, 0x80)){ delay(100);  // reset
      if(write8(addr, REG_PWR_MGMT_1, 0x01)){            // clock PLL
        MPU_ADDR=addr;
        break;
      }
    }
  }
  // basic config
  if(!write8(MPU_ADDR, REG_CONFIG,        0x03)) return false; // DLPF
  if(!write8(MPU_ADDR, REG_GYRO_CONFIG,   0x00)) return false; // ±250dps
  if(!write8(MPU_ADDR, REG_ACCEL_CONFIG,  0x00)) return false; // ±2g
  if(!write8(MPU_ADDR, REG_ACCEL_CONFIG2, 0x03)) return false; // DLPF
  // bypass enable -> talk to AK8963 directly
  if(!write8(MPU_ADDR, REG_INT_PIN_CFG,   0x02)) return false; // BYPASS_EN
  delay(10);
  return true;
}
bool ak8963Init(){
  // power down
  if(!write8(AK8963_ADDR, AK_REG_CNTL1, 0x00)) return false; delay(10);
  // enter fuse ROM access to read ASA
  if(!write8(AK8963_ADDR, AK_REG_CNTL1, 0x0F)) return false; delay(10);
  uint8_t asa[3];
  if(!readBytes(AK8963_ADDR, AK_REG_ASAX, 3, asa)) return false;
  magAdjX = ( (asa[0]-128)*0.5f/128.0f ) + 1.0f;
  magAdjY = ( (asa[1]-128)*0.5f/128.0f ) + 1.0f;
  magAdjZ = ( (asa[2]-128)*0.5f/128.0f ) + 1.0f;
  // power down
  if(!write8(AK8963_ADDR, AK_REG_CNTL1, 0x00)) return false; delay(10);
  // 16-bit, continuous mode 2 (100 Hz): 0x16
  if(!write8(AK8963_ADDR, AK_REG_CNTL1, 0x16)) return false; delay(10);
  return true;
}

// quick gyro bias (place still)
void calibrateGyro(uint16_t N=400){
  long sx=0, sy=0, sz=0;
  uint8_t b[6];
  for(uint16_t i=0;i<N;i++){
    if(readBytes(MPU_ADDR, REG_GYRO_XOUT_H, 6, b)){
      sx += be16(b[0],b[1]);
      sy += be16(b[2],b[3]);
      sz += be16(b[4],b[5]);
    }
    delay(3);
  }
  gBiasX = (sx/(float)N) / GYRO_LSB_250;
  gBiasY = (sy/(float)N) / GYRO_LSB_250;
  gBiasZ = (sz/(float)N) / GYRO_LSB_250;
}

void setup(){
  Serial.begin(115200);
  delay(200);
  Serial.println(F("Mega + MPU-9250 R/P/Y (no libs)"));

  Wire.begin();             // Mega: SDA=20, SCL=21
  Wire.setClock(100000);    // 100 kHz부터 (안정)
  delay(50);

  if(!mpuInit()){
    Serial.println(F("[ERR] MPU init failed. Check wiring & power.")); while(1){delay(1000);}
  }
  if(!ak8963Init()){
    Serial.println(F("[WARN] AK8963 init failed. Yaw will drift (gyro only)."));
    magReady=false;
  } else {
    magReady=true;
  }

  calibrateGyro(300);
  lastMicros = micros();
  Serial.println(F("Printing: Roll, Pitch, Yaw (deg)"));
}

void loop(){
  // timing
  unsigned long now = micros();
  float dt = (now - lastMicros) * 1e-6f;
  if (dt < 0.005f){ return; } // ~200 Hz max
  lastMicros = now;

  // read accel & gyro
  uint8_t bg[14];
  if(!readBytes(MPU_ADDR, REG_ACCEL_XOUT_H, 14, bg)) return;
  int16_t axr=be16(bg[0],bg[1]),  ayr=be16(bg[2],bg[3]),  azr=be16(bg[4],bg[5]);
  int16_t gxr=be16(bg[8],bg[9]), gyr=be16(bg[10],bg[11]), gzr=be16(bg[12],bg[13]);

  // scale
  float ax = axr / ACC_LSB_2G;
  float ay = ayr / ACC_LSB_2G;
  float az = azr / ACC_LSB_2G;
  float gx = (gxr / GYRO_LSB_250) - gBiasX; // dps
  float gy = (gyr / GYRO_LSB_250) - gBiasY;
  float gz = (gzr / GYRO_LSB_250) - gBiasZ;

  // accel-only angles (deg)
  float rollAcc  = rad2deg( atan2f(ay, az) );
  float pitchAcc = rad2deg( atan2f(-ax, sqrtf(ay*ay + az*az) ) );

  // complementary filter (gyro integration + accel correction)
  roll  = CF_ALPHA*(roll  + gx*dt) + (1.0f-CF_ALPHA)*rollAcc;
  pitch = CF_ALPHA*(pitch + gy*dt) + (1.0f-CF_ALPHA)*pitchAcc;

  // yaw
  float yawOut;
  bool usedMag=false;
  if (magReady){
    // check data ready
    uint8_t st1;
    if (readBytes(AK8963_ADDR, AK_REG_ST1, 1, &st1) && (st1 & 0x01)){
      uint8_t mb[7];
      if (readBytes(AK8963_ADDR, AK_REG_XL, 7, mb)){
        // little-endian: XL,XH,YL,YH,ZL,ZH,ST2
        int16_t mxr = (int16_t)((mb[1]<<8)|mb[0]);
        int16_t myr = (int16_t)((mb[3]<<8)|mb[2]);
        int16_t mzr = (int16_t)((mb[5]<<8)|mb[4]);
        uint8_t st2 = mb[6];
        if (!(st2 & 0x08)){ // overflow bit
          // apply factory sensitivity & convert to uT
          float mx = (mxr * MAG_LSB_16 * magAdjX);
          float my = (myr * MAG_LSB_16 * magAdjY);
          float mz = (mzr * MAG_LSB_16 * magAdjZ);

          // tilt compensation
          float cr = cosf( deg2rad(roll) ),  sr = sinf( deg2rad(roll) );
          float cp = cosf( deg2rad(pitch) ), sp = sinf( deg2rad(pitch) );
          float Xh = mx*cp + mz*sp;
          float Yh = mx*sr*sp + my*cr - mz*sr*cp;

          float yawMag = rad2deg( atan2f(Yh, Xh) ) + DECLINATION_DEG;
          yawOut = wrap180(yawMag);
          usedMag = true;
        }
      }
    }
  }
  if (!usedMag){
    yawOut = wrap180(yaw + gz*dt); // integrate gyro Z if mag not ready
  }
  yaw = yawOut;

  // print
  static unsigned long lastP=0; 
  if (millis()-lastP > 50){ // ~20Hz print
    lastP = millis();
    Serial.print(roll,1);  Serial.print(", ");
    Serial.print(pitch,1); Serial.print(", ");
    Serial.print(yaw,1);
    if (!usedMag) Serial.print("  (yaw:gyro)");
    Serial.println();
  }
}
