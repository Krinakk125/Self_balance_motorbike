#include <Arduino.h>
#include <Wire.h>

#define SDA_PIN 16
#define SCL_PIN 17
#define I2C_HZ  100000

// Doc n byte tu AS5600, false neu chip khong tra loi
bool readBytes(uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(0x36);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)0x36, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

unsigned long okCnt = 0, errCnt = 0, badCnt = 0, mlCnt = 0, mhCnt = 0, lastPrint = 0;
int rawMin = 4096, rawMax = -1;
uint8_t agcLast = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_HZ);
  Serial.println("\n=== Dem loi doc AS5600 (moi 1 giay) ===");
}

void loop() {
  uint8_t st, agc, ang[2];
  if (!readBytes(0x0B, &st, 1) || !readBytes(0x1A, &agc, 1) || !readBytes(0x0C, ang, 2)) {
    errCnt++;                       // I2C khong tra loi
  } else if ((st & 0x18) == 0x18) {
    badCnt++;                       // ML va MH cung 1: vo ly, la doc loi
  } else {
    okCnt++;
    int raw = ((ang[0] & 0x0F) << 8) | ang[1];
    if (raw < rawMin) rawMin = raw;
    if (raw > rawMax) rawMax = raw;
    agcLast = agc;
    if (st & 0x10) mlCnt++;
    if (st & 0x08) mhCnt++;
  }
  delay(2);

  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    if (okCnt > 0) {
      Serial.printf("OK=%lu | loiI2C=%lu | ML&MH cung 1=%lu | ML=%lu MH=%lu | AGC=%u | raw min=%d max=%d (dao dong %d)\n",
                    okCnt, errCnt, badCnt, mlCnt, mhCnt, agcLast, rawMin, rawMax, rawMax - rawMin);
    } else {
      Serial.printf("KHONG co lan doc hop le | loiI2C=%lu | ML&MH cung 1=%lu\n", errCnt, badCnt);
    }
    okCnt = errCnt = badCnt = mlCnt = mhCnt = 0;
    rawMin = 4096; rawMax = -1;
  }
}