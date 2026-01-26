/***************************************************
 * Arduino Mega 2560 + Cytron MD10C R3
 * 4 Mecanum Wheels - QR Code Control
 * รับคำสั่งจาก Raspberry Pi 5 ผ่าน Serial
 ***************************************************/

// ================= PIN DEFINITIONS =================
// Front Left
#define M1_PWM 14
#define M1_DIR 18

// Front Right
#define M2_PWM 15
#define M2_DIR 19

// Rear Left
#define M3_PWM 16
#define M3_DIR 20

// Rear Right
#define M4_PWM 17
#define M4_DIR 21

// ================= MOTOR DIRECTION SETTING =================
// 1 = forward correct, 0 = reverse
#define M1_INV 0
#define M2_INV 1
#define M3_INV 0
#define M4_INV 1

// ================= SPEED SETTINGS =================
int SPEED = 200;

// ================= SERIAL VARIABLES =================
int currentCommand = 0;
bool isMoving = false;

// ================= LED INDICATOR =================
#define LED_PIN 13

// ================= MOTOR CONTROL (จากโค้ดเดิมของคุณ) =================
void setMotor(int pwmPin, int dirPin, int inv, int speed) {
  speed = constrain(speed, -255, 255);

  int dir = (speed >= 0) ? 1 : 0;

  // invert direction if needed
  if (inv == 0) dir = !dir;

  digitalWrite(dirPin, dir);
  analogWrite(pwmPin, abs(speed));
}

// ================= MOVEMENT FUNCTIONS (จากโค้ดเดิมของคุณ) =================
void stopMotor() {
  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 0);
  analogWrite(M3_PWM, 0);
  analogWrite(M4_PWM, 0);
  isMoving = false;
  Serial.println("STATUS: STOPPED");
}

void moveForward(int spd) {
  setMotor(M1_PWM, M1_DIR, M1_INV, spd);
  setMotor(M2_PWM, M2_DIR, M2_INV, spd);
  setMotor(M3_PWM, M3_DIR, M3_INV, spd);
  setMotor(M4_PWM, M4_DIR, M4_INV, spd);
  Serial.println("STATUS: MOVING FORWARD");
}

void moveBackward(int spd) {
  moveForward(-spd);
  Serial.println("STATUS: MOVING BACKWARD");
}

void turnLeft(int spd) {
  setMotor(M1_PWM, M1_DIR, M1_INV, -spd);
  setMotor(M3_PWM, M3_DIR, M3_INV, -spd);
  setMotor(M2_PWM, M2_DIR, M2_INV, spd);
  setMotor(M4_PWM, M4_DIR, M4_INV, spd);
  Serial.println("STATUS: TURNING LEFT");
}

void turnRight(int spd) {
  setMotor(M1_PWM, M1_DIR, M1_INV, spd);
  setMotor(M3_PWM, M3_DIR, M3_INV, spd);
  setMotor(M2_PWM, M2_DIR, M2_INV, -spd);
  setMotor(M4_PWM, M4_DIR, M4_INV, -spd);
  Serial.println("STATUS: TURNING RIGHT");
}

// ================= QR COMMAND HANDLER =================
void handleQRCommand(int destination) {
  currentCommand = destination;
  
  Serial.print("RECEIVED: ");
  Serial.println(destination);
  
  // คุณสามารถปรับการเคลื่อนที่ตาม QR Code ที่อ่านได้ที่นี่
  switch(destination) {
    case 0:  // STOP / HOME
      stopMotor();
      break;
      
    case 1:  // POS_A_COL1 - เมื่ออ่าน QR "1" ให้วิ่งหน้า
      moveForward(SPEED);
      isMoving = true;
      Serial.println("DESTINATION: POS_A_COL1");
      break;
      
    case 2:  // POS_A_COL2
      moveForward(SPEED);
      isMoving = true;
      Serial.println("DESTINATION: POS_A_COL2");
      break;
      
    case 3:  // POS_B_COL1
      turnRight(SPEED);
      isMoving = true;
      Serial.println("DESTINATION: POS_B_COL1");
      break;
      
    case 4:  // POS_B_COL2
      moveForward(SPEED);
      isMoving = true;
      Serial.println("DESTINATION: POS_B_COL2");
      break;
      
    case 5:  // POS_C_COL1
      moveBackward(SPEED);
      isMoving = true;
      Serial.println("DESTINATION: POS_C_COL1");
      break;
      
    case 6:  // POS_C_COL2
      turnLeft(SPEED);
      isMoving = true;
      Serial.println("DESTINATION: POS_C_COL2");
      break;
      
    case 7:  // POS_D_COL1
      moveBackward(SPEED);
      isMoving = true;
      Serial.println("DESTINATION: POS_D_COL1");
      break;
      
    case 8:  // POS_D_COL2
      moveForward(SPEED);
      isMoving = true;
      Serial.println("DESTINATION: POS_D_COL2");
      break;
      
    default:
      Serial.print("UNKNOWN COMMAND: ");
      Serial.println(destination);
      stopMotor();
      break;
  }
}

// ================= SERIAL COMMUNICATION =================
void readSerialCommand() {
  if (Serial.available() > 0) {
    // อ่านคำสั่งเป็นตัวเลข
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.length() > 0) {
      int command = input.toInt();
      
      // LED กระพริบเมื่อได้รับคำสั่ง
      digitalWrite(LED_PIN, HIGH);
      
      handleQRCommand(command);
      
      delay(50);
      digitalWrite(LED_PIN, LOW);
    }
  }
}

// ================= SETUP =================
void setup() {
  // เริ่ม Serial Communication (115200 ต้องตรงกับ Pi)
  Serial.begin(115200);
  while (!Serial) {
    ; // รอให้ Serial เชื่อมต่อ
  }
  
  Serial.println("===================================");
  Serial.println("Arduino Mega 2560 - QR Robot Control");
  Serial.println("Waiting for commands from Pi...");
  Serial.println("===================================");
  
  // ตั้งค่า Motor Pins
  pinMode(M1_DIR, OUTPUT);
  pinMode(M2_DIR, OUTPUT);
  pinMode(M3_DIR, OUTPUT);
  pinMode(M4_DIR, OUTPUT);

  pinMode(M1_PWM, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  pinMode(M3_PWM, OUTPUT);
  pinMode(M4_PWM, OUTPUT);
  
  // LED Indicator
  pinMode(LED_PIN, OUTPUT);

  // หยุดมอเตอร์ทั้งหมด
  stopMotor();
  
  // กระพริบ LED 3 ครั้งเพื่อบอกว่าพร้อมแล้ว
  for(int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
  
  Serial.println("READY!");
}

// ================= MAIN LOOP =================
void loop() {
  // อ่านคำสั่งจาก Serial (จาก Raspberry Pi)
  readSerialCommand();
  
  delay(10);
}
