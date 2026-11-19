#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>

#define SDA_PIN 16
#define SCL_PIN 17

// ===== Trần an toàn (không vượt được bằng lệnh Serial) =====
const float MAX_VOLTAGE  = 3.0f;   // V, đúng mức đã chạy được ở chế độ torque
const float MAX_VELOCITY = 20.0f;  // rad/s

BLDCMotor motor = BLDCMotor(10);
BLDCDriver3PWM driver = BLDCDriver3PWM(18, 19, 21, 4);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

float target = 0;     // tốc độ mục tiêu (rad/s)
float dirSet = 1;     // 1 = CW, -1 = CCW
Commander command = Commander(Serial);

void onT(char* c) { command.scalar(&target, c);
                    target = constrain(target, -MAX_VELOCITY, MAX_VELOCITY); }
void onP(char* c) { command.scalar(&motor.PID_velocity.P, c); }
void onI(char* c) { command.scalar(&motor.PID_velocity.I, c); }
void onK(char* c) { command.scalar(&motor.PID_velocity.D, c); }   // K = hệ số D
void onF(char* c) { command.scalar(&motor.LPF_velocity.Tf, c); }  // F = bộ lọc tốc độ
void onV(char* c) { float v = motor.voltage_limit;
                    command.scalar(&v, c);
                    v = constrain(v, 0.0f, MAX_VOLTAGE);
                    motor.voltage_limit = v;
                    motor.PID_velocity.limit = v; }
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

  driver.voltage_power_supply = 12;      // đo VM thực tế rồi sửa
  driver.voltage_limit = 12;
  driver.init();
  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);

  // --- Phần giữ nguyên như code torque đã chạy được ---
  motor.torque_controller = TorqueControlType::voltage;
  motor.voltage_sensor_align = 2.2;
  motor.sensor_direction = Direction::CW;

  // --- Phần mới: vòng tốc độ có PID ---
  motor.controller = MotionControlType::velocity;
  motor.velocity_limit = MAX_VELOCITY;
  motor.voltage_limit = 2.0;             // trần điện áp do PID xuất ra (V)

  motor.PID_velocity.P = 0.1;            // V trên mỗi rad/s sai số
  motor.PID_velocity.I = 1.0;
  motor.PID_velocity.D = 0;
  motor.PID_velocity.limit = motor.voltage_limit;
  motor.PID_velocity.output_ramp = 10;   // V/s, giảm giật khi đổi tốc độ
  motor.LPF_velocity.Tf = 0.02;          // lọc tốc độ

  motor.init();
  motor.initFOC();
  Serial.printf("zero_elec=%.2f\n", motor.zero_electric_angle);

  command.add('T', onT, "toc do rad/s");
  command.add('P', onP, "PID P");
  command.add('I', onI, "PID I");
  command.add('K', onK, "PID D");
  command.add('F', onF, "loc toc do Tf");
  command.add('V', onV, "tran dien ap");
  command.add('Z', onZ, "zero elec angle");
  command.add('D', onD, "chieu cam bien 1/-1");
  Serial.println("T:toc do | P,I,K:PID | F:loc | V:tran ap | Z:zero | D1/D-1:chieu");
}

void loop() {
  
  motor.loopFOC();            // vòng mô-men (FOC) - giữ nguyên
  motor.move(target);         // vòng tốc độ PID, kết quả đặt vào Uq
  command.run();
  loops++;

  if (millis() - lastPrint >= 500) {
    float hz = loops * 1000.0f / (millis() - lastPrint);
    lastPrint = millis(); loops = 0;
    Serial.printf("T=%.2f v=%.2f rad/s | Uq=%.2f V | loop=%.0f Hz\n",
                  target, motor.shaft_velocity, motor.voltage.q, hz);
  }
}