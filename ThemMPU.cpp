#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>

// ================== PIN ==================
#define SDA_PIN   16   // AS5600 (Wire)
#define SCL_PIN   17
#define MPU_SDA   23   // MPU6050 (Wire1, bus rieng)
#define MPU_SCL   22
#define LED_PIN   2
#define FAULT_PIN 27   // nFAULT cua DRV8313 (muc thap = co loi), da doi khoi GPIO23

#define MPU_ADDR  0x68   // AD0 noi GND = 0x68, noi 3.3V = 0x69
#define TILT_ABOUT_X 1   // 1: nghieng quanh truc X (goc = atan2(ay,az), gyro X)
                         // 0: nghieng quanh truc Y (goc = atan2(-ax, sqrt(ay^2+az^2)), gyro Y)

BLDCMotor motor = BLDCMotor(10);
BLDCDriver3PWM driver = BLDCDriver3PWM(18, 19, 21, 4);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// ================== BIEN ==================
float target = 0;     // V đặt vào trục q
float dirSet = 1;     // 1 = CW, -1 = CCW

// MPU6050
bool  mpuOk = false;
bool  tiltInit = false;
float tiltAngle = 0;     // do, sau loc bu
float tiltRate  = 0;     // do/s
float angleOffset = 0;   // goc tham chieu (lenh G)
float accX = 0, accY = 0, accZ = 0;   // g
float gyroBias = 0;      // do/s
unsigned long mpuGood = 0, mpuBad = 0, lastMpuUs = 0, lastPollUs = 0;

// chan doan
unsigned long lastPrint = 0, loops = 0;
unsigned long statusOk = 0, statusErr = 0, jumpCnt = 0;
float lastAng = 0;
bool haveLast = false;
unsigned long faultCnt = 0, maxDtUs = 0, lastLoopUs = 0;

Commander command = Commander(Serial);
void onT(char* c) { command.scalar(&target, c); target = constrain(target, -2.5f, 2.5f); }
void onZ(char* c) { command.scalar(&motor.zero_electric_angle, c); }
void onD(char* c) { command.scalar(&dirSet, c);
                    motor.sensor_direction = (dirSet >= 0) ? Direction::CW : Direction::CCW; }
void onG(char* c) { angleOffset = tiltAngle;
                    Serial.printf("Dat goc tham chieu = %.2f do (G: dat tai vi tri hien tai)\n", angleOffset); }

// ================== AS5600 (Wire) ==================
bool readBytes(uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(0x36);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)0x36, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

bool waitSensor(unsigned long timeoutMs) {
  unsigned long t0 = millis();
  while (millis() - t0 < timeoutMs) {
    Wire.beginTransmission(0x36);
    if (Wire.endTransmission() == 0) return true;
    Serial.print(".");
    delay(200);
  }
  return false;
}

void halt(const char* msg) {
  Serial.println(msg);
  pinMode(LED_PIN, OUTPUT);
  while (true) { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(100); }
}

// ================== MPU6050 (Wire1) ==================
bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire1.beginTransmission(MPU_ADDR);
  Wire1.write(reg);
  Wire1.write(val);
  return Wire1.endTransmission() == 0;
}

bool mpuRead(uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire1.beginTransmission(MPU_ADDR);
  Wire1.write(reg);
  if (Wire1.endTransmission(false) != 0) return false;
  if (Wire1.requestFrom((uint8_t)MPU_ADDR, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire1.read();
  return true;
}

bool mpuInit() {
  uint8_t who;
  if (!mpuRead(0x75, &who, 1)) return false;          // WHO_AM_I
  Serial.printf("MPU WHO_AM_I = 0x%02X (MPU6050 thuong la 0x68)\n", who);
  if (!mpuWrite(0x6B, 0x00)) return false;            // thuc day, bo che do ngu
  delay(100);
  mpuWrite(0x1A, 0x03);                               // bo loc so ~44 Hz
  mpuWrite(0x1B, 0x08);                               // gyro  +-500 do/s  (65.5 LSB)
  mpuWrite(0x1C, 0x08);                               // accel +-4 g       (8192 LSB/g)
  return true;
}

void calibGyro() {
  Serial.println("Giu yen MPU 2 giay de hieu chinh gyro...");
  float sum = 0; int n = 0;
  for (int i = 0; i < 300; i++) {
    uint8_t b[14];
    if (mpuRead(0x3B, b, 14)) {
#if TILT_ABOUT_X
      int16_t g = (int16_t)((b[8] << 8) | b[9]);
#else
      int16_t g = (int16_t)((b[10] << 8) | b[11]);
#endif
      sum += g / 65.5f; n++;
    }
    delay(7);
  }
  if (n > 0) gyroBias = sum / n;
  Serial.printf("gyro bias = %.2f do/s (%d mau)\n", gyroBias, n);
}

bool mpuUpdate() {
  uint8_t b[14];
  if (!mpuRead(0x3B, b, 14)) { mpuBad++; return false; }
  int16_t ax = (int16_t)((b[0] << 8) | b[1]);
  int16_t ay = (int16_t)((b[2] << 8) | b[3]);
  int16_t az = (int16_t)((b[4] << 8) | b[5]);
  int16_t gx = (int16_t)((b[8] << 8) | b[9]);
  int16_t gy = (int16_t)((b[10] << 8) | b[11]);
  accX = ax / 8192.0f; accY = ay / 8192.0f; accZ = az / 8192.0f;

#if TILT_ABOUT_X
  float accAng  = atan2f(accY, accZ) * RAD_TO_DEG;
  float gyroRaw = gx / 65.5f;
#else
  float accAng  = atan2f(-accX, sqrtf(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  float gyroRaw = gy / 65.5f;
#endif

  unsigned long now = micros();
  float dt = (now - lastMpuUs) * 1e-6f;
  lastMpuUs = now;
  tiltRate = gyroRaw - gyroBias;
  if (!tiltInit || dt <= 0 || dt > 0.1f) {
    tiltAngle = accAng;            // khoi tao / lay lai khi mat mau lau
    tiltInit = true;
  } else {
    tiltAngle = 0.98f * (tiltAngle + tiltRate * dt) + 0.02f * accAng;   // loc bu
  }
  mpuGood++;
  return true;
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(FAULT_PIN, INPUT_PULLUP);

  // --- AS5600 ---
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Serial.print("Cho AS5600 (0x36)");
  if (!waitSensor(10000)) halt("\nKHONG THAY AS5600 sau 10 giay! Kiem tra day/nguon.");
  Serial.println(" OK");
  sensor.init(&Wire);

  // --- MPU6050: lam truoc khi motor giat luc initFOC ---
  Wire1.begin(MPU_SDA, MPU_SCL);
  Wire1.setClock(400000);
  mpuOk = mpuInit();
  if (mpuOk) calibGyro();
  else Serial.println("MPU6050 KHONG tra loi (0x68). Van chay tiep nhung khong co MPU.");

  // --- Driver + motor ---
  driver.voltage_power_supply = 12;
  driver.voltage_limit = 12;
  driver.init();
  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);

  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;
  motor.voltage_limit = 2.5;
  motor.voltage_sensor_align = 2.2;
  motor.sensor_direction = Direction::CW;     // D1 da cho ket qua dung

  motor.init();
  if (!motor.initFOC()) {
    motor.disable();
    halt("initFOC FAILED! (cam bien / nam cham / day dien)");
  }
  Serial.printf("zero_elec=%.2f\n", motor.zero_electric_angle);

  command.add('T', onT, "dien ap q");
  command.add('Z', onZ, "zero elec angle");
  command.add('D', onD, "chieu cam bien 1/-1");
  command.add('G', onG, "dat goc tham chieu MPU");
  Serial.println("T<v>: dien ap | Z<rad>: zero | D1/D-1: chieu | G: dat goc MPU hien tai = 0");
}

// ================== LOOP ==================
void loop() {
  unsigned long nowUs = micros();
  if (lastLoopUs != 0 && nowUs - lastLoopUs > maxDtUs) maxDtUs = nowUs - lastLoopUs;
  lastLoopUs = nowUs;
  if (digitalRead(FAULT_PIN) == LOW) faultCnt++;

  motor.loopFOC();

  // goc AS5600 nhay bat thuong giua 2 vong FOC
  float a = sensor.getMechanicalAngle();
  if (haveLast) {
    float d = fabsf(a - lastAng);
    if (d > _PI) d = _2PI - d;
    if (d > 0.3f) jumpCnt++;
  }
  lastAng = a;
  haveLast = true;

  motor.move(target);
  command.run();

  // doc MPU6050 moi 5 ms (200 Hz)
  if (mpuOk && micros() - lastPollUs >= 5000) {
    lastPollUs = micros();
    mpuUpdate();
  }
  loops++;

  if (millis() - lastPrint >= 500) {
    float hz = loops * 1000.0f / (millis() - lastPrint);
    lastPrint = millis(); loops = 0;
    Serial.printf("T=%.2f v=%.2f rad/s | loop=%.0f Hz dt max=%lu us | ", target, motor.shaft_velocity, hz, maxDtUs);

    uint8_t st, agc, ang[2];
    if (!readBytes(0x0B, &st, 1) || !readBytes(0x1A, &agc, 1) || !readBytes(0x0C, ang, 2)) {
      statusErr++;
      Serial.print("AS5600: KHONG TRA LOI");
    } else if ((st & 0x18) == 0x18) {
      statusErr++;
      Serial.print("AS5600: DOC LOI (0xFF)");
    } else {
      statusOk++;
      int raw = ((ang[0] & 0x0F) << 8) | ang[1];
      Serial.printf("AS5600: OK ML=%d MH=%d AGC=%u raw=%d", (st >> 4) & 1, (st >> 3) & 1, agc, raw);
    }
    Serial.printf(" | nhay=%lu loi=%lu/%lu | nFAULT thap=%lu vong\n",
                  jumpCnt, statusErr, statusOk + statusErr, faultCnt);

    if (mpuOk) {
      Serial.printf("   MPU: goc=%.2f do | toc do nghieng=%.1f do/s | acc=%.2f,%.2f,%.2f g | ok/loi=%lu/%lu\n",
                    tiltAngle - angleOffset, tiltRate, accX, accY, accZ, mpuGood, mpuBad);
    } else {
      Serial.println("   MPU: khong co");
    }
    jumpCnt = 0; maxDtUs = 0; faultCnt = 0; mpuGood = 0; mpuBad = 0;
    lastLoopUs = 0;   // khong tinh thoi gian in vao dt max
  }
}