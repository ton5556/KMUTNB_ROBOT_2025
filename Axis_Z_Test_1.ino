#define PUL 9
#define DIR 8

#define PULSE_PER_MM 80  

void moveMM(float mm, bool dir) {
  long pulses = mm * PULSE_PER_MM;

  digitalWrite(DIR, dir);

  for (long i = 0; i < pulses; i++) {
    digitalWrite(PUL, HIGH);
    delayMicroseconds(50);
    digitalWrite(PUL, LOW);
    delayMicroseconds(50);
  }
}

void p1() {
  moveMM(20, LOW);
  delay(3000);
  moveMM(20, HIGH);
  delay(500);
}

void p2() {
  moveMM(175, LOW);  // เดินหน้า 10cm
  delay(300);
  moveMM(20, LOW);
  delay(3000);
  moveMM(195, HIGH);
  delay(300);
}

void p3() {
  moveMM(350, LOW);
  delay(300);
  moveMM(20, LOW);
  delay(3000);
  moveMM(370, HIGH);
  delay(300);
}


void setup() {
  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);
}

void loop() {
  p1();
  p2();
  p3();
}
