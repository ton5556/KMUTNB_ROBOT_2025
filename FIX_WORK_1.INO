#include <Wire.h>

#define LINE_FOLLOWER_I2C_ADDR 0x78

// Motor pins
const uint8_t FL_PWM = 10;
const uint8_t FR_PWM = 9;
const uint8_t BL_PWM = 6;
const uint8_t BR_PWM = 11;

const uint8_t FL_DIR = 12;
const uint8_t FR_DIR = 8;
const uint8_t BL_DIR = 7;
const uint8_t BR_DIR = 13;

const uint8_t PWM_MIN = 10;

// ค่าและบัฟเฟอร์ข้อมูลเซนเซอร์
uint8_t rec_data[4];
uint8_t data = 0;

// ค่าชดเชยมอเตอร์ (ปรับได้ตามต้องการ)
int motorCompensation = 15; // เพิ่มกำลังให้มอเตอร์ตัวใดตัวหนึ่งเพื่อสมดุล

// -------------------- Motor Control --------------------
void setMotor(uint8_t pwmPin, uint8_t dirPin, int speed, uint8_t direction) {
  speed = constrain(speed, 0, 255);
  digitalWrite(dirPin, direction);
  if (speed == 0) {
    analogWrite(pwmPin, 0);
  } else {
    // แปลงช่วงความเร็วจาก 0..255 เป็น PWM_MIN..255 เพื่อไม่ให้สั่นตอนความเร็วต่ำ
    int out = map(speed, 0, 255, PWM_MIN, 255);
    analogWrite(pwmPin, out);
  }
}

void moveForward(int speed) {
  setMotor(FL_PWM, FL_DIR, speed, 1);
  setMotor(FR_PWM, FR_DIR, speed, 0);
  // ชดเชยมอเตอร์หลังซ้ายตามตัวแปร motorCompensation
  setMotor(BL_PWM, BL_DIR, speed + motorCompensation, 0);
  setMotor(BR_PWM, BR_DIR, speed, 1);
}

void turnLeft(int speed) {
  setMotor(FL_PWM, FL_DIR, speed, 0);
  setMotor(FR_PWM, FR_DIR, 0, 0);
  setMotor(BL_PWM, BL_DIR, 0, 0);
  setMotor(BR_PWM, BR_DIR, speed, 0);
}

void turnRight(int speed) {
  setMotor(FL_PWM, FL_DIR, 0, 1);
  setMotor(FR_PWM, FR_DIR, speed, 1);
  setMotor(BL_PWM, BL_DIR, speed, 1);
  setMotor(BR_PWM, BR_DIR, 0, 1);
}

void stopMotors() {
  setMotor(FL_PWM, FL_DIR, 0, 1);
  setMotor(FR_PWM, FR_DIR, 0, 1);
  setMotor(BL_PWM, BL_DIR, 0, 1);
  setMotor(BR_PWM, BR_DIR, 0, 1);
}

// -------------------- I2C Read Sensor --------------------
bool WireWriteByte(uint8_t val) {
  Wire.beginTransmission(LINE_FOLLOWER_I2C_ADDR);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

bool WireReadDataByte(uint8_t reg, uint8_t &val) {
  if (!WireWriteByte(reg)) return false;
  Wire.requestFrom(LINE_FOLLOWER_I2C_ADDR, (uint8_t)1);
  if (Wire.available()) {
    val = Wire.read();
    return true;
  }
  return false;
}

// -------------------- I2C Read Sensor (with inversion fix & debug) --------------------
void Sensor_Receive() {
  uint8_t d;
  if (WireReadDataByte(1, d)) {
    data = d; // raw data from I2C (bit=1 ตามที่โมดูลส่ง)
    // many modules output 0 = detect line, 1 = no line
    // invert here so rec_data[i] == 1 means "sensor i detects line"
    rec_data[0] = ((data & 0x01) == 0) ? 1 : 0;
    rec_data[1] = (((data >> 1) & 0x01) == 0) ? 1 : 0;
    rec_data[2] = (((data >> 2) & 0x01) == 0) ? 1 : 0;
    rec_data[3] = (((data >> 3) & 0x01) == 0) ? 1 : 0;
  } else {
    // อ่านไม่สำเร็จ: ตั้งเป็นไม่มีการตรวจจับ
    data = 0xFF; // แสดงว่าอ่านล้มเหลว (ทุกบิตเป็น 1)
    rec_data[0] = rec_data[1] = rec_data[2] = rec_data[3] = 0;
  }
}


// -------------------- Tracking Only --------------------
void Tracking_Line() {
  // --- ตรงกลางโดนเส้น → วิ่งตรง ---
  if (rec_data[1] == 1 && rec_data[2] == 1) {
    moveForward(85);
    return;
  }

  // --- เอียงขวา ---
  if (rec_data[1] == 1 && rec_data[2] == 0) {
    turnRight(85);
    return;
  }

  // --- เอียงซ้าย ---
  if (rec_data[1] == 0 && rec_data[2] == 1) {
    turnLeft(85);
    return;
  }

  // --- ขึ้นขอบซ้าย (เซ็นเซอร์ตัวแรกไม่ตรวจจับ แต่ 2,3,4 ตรวจจับ) ---
  if (rec_data[0] == 0 && rec_data[1] == 1 && rec_data[2] == 1 && rec_data[3] == 1) {
    turnRight(85);
    return;
  }

  // --- ขึ้นขอบขวา ---
  if (rec_data[0] == 1 && rec_data[1] == 1 && rec_data[2] == 1 && rec_data[3] == 0) {
    turnLeft(85);
    return;
  }

  // --- ทั้งหมดตรวจจับ (อาจเป็นจุดตัดหรือสิ้นสุดเส้น) ---
  if (rec_data[0] == 1 && rec_data[1] == 1 && rec_data[2] == 1 && rec_data[3] == 1) {
    stopMotors();
    Serial.println("STOP - all sensors on line");
    return;
  }

  // --- ถ้าไม่มีเซนเซอร์ใดตรวจจับ (หลุดเส้น) ให้หยุดชั่วคราว ---
  if (rec_data[0] == 0 && rec_data[1] == 0 && rec_data[2] == 0 && rec_data[3] == 0) {
    stopMotors();
    Serial.println("LOST - no sensors on line");
    return;
  }
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);

  pinMode(FL_DIR, OUTPUT);
  pinMode(FR_DIR, OUTPUT);
  pinMode(BL_DIR, OUTPUT);
  pinMode(BR_DIR, OUTPUT);

  pinMode(FL_PWM, OUTPUT);
  pinMode(FR_PWM, OUTPUT);
  pinMode(BL_PWM, OUTPUT);
  pinMode(BR_PWM, OUTPUT);

  Wire.begin();
  Wire.setClock(400000);
}
void loop() {
  Sensor_Receive();
  Tracking_Line();

  // แสดงทั้ง raw data และสถานะที่ตีความแล้ว
  Serial.print("RAW hex: 0x"); Serial.print(data, HEX);
  Serial.print("  | Interpreted sensors (1=on line): ");
  Serial.print(rec_data[0]); Serial.print(",");
  Serial.print(rec_data[1]); Serial.print(",");
  Serial.print(rec_data[2]); Serial.print(",");
  Serial.println(rec_data[3]);

  delay(150);
}

