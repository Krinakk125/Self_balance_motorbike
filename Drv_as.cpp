#include <Wire.h>
#include <SimpleFOC.h>

#define SDA_PIN 16
#define SCL_PIN 17

BLDCMotor motor = BLDCMotor(10);
BLDCDriver3PWM driver = BLDCDriver3PWM(18, 19, 21, 4);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

float target = 0;     // V đặt vào trục q
float dirSet = 1;     // 1 = CW, -1 = CCW
Commander command = Commander(Serial);
void onT(char* c) { command.scalar(&target, c); target = constrain(target, -2.5f, 2.5f); }
void onZ(char* c) { command.scalar(&motor.zero_electric_angle, c); }
void onD(char* c) { command.scalar(&dirSet, c);
                    motor.sensor_direction = (dirSet >= 0) ? Direction::CW : Direction::CCW; }

unsigned long lastPrint = 0, loops = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  sensor.init(&Wire);

  driver.voltage_power_supply = 12;
  driver.voltage_limit = 12;
  driver.init();
  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);

  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;                 
  motor.voltage_limit = 2.5;
  motor.voltage_sensor_align = 2.2;
  motor.sensor_direction = Direction::CW;     // sửa theo kết quả Test 1

  motor.init();
  motor.initFOC();
  Serial.printf("zero_elec=%.2f\n", motor.zero_electric_angle);

  command.add('T', onT, "dien ap q");
  command.add('Z', onZ, "zero elec angle");
  command.add('D', onD, "chieu cam bien 1/-1");
  Serial.println("T<v>: dien ap | Z<rad>: zero | D1/D-1: chieu");
}

void loop() {
  motor.loopFOC();
  motor.move(target);
  command.run();
  loops++;

  if (millis() - lastPrint >= 500) {
    float hz = loops * 1000.0f / (millis() - lastPrint);
    lastPrint = millis(); loops = 0;
    Serial.printf("T=%.2f v=%.2f rad/s | loop=%.0f Hz\n", target, motor.shaft_velocity, hz);
  }
}