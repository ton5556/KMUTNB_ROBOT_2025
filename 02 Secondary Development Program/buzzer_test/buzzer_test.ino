/**
 * @file buzzer_test.ino
 * @brief 音乐播放(Play music)
 * @author Anonymity(Anonymity@hiwonder.com)
 * @version V1.0
 * @date 2024-04-23
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "tone.h"

/* 超级玛丽(Super Mary) */
static int song[98] = {
  NOTE_E4, NOTE_E4, NOTE_E4, NOTE_C4, NOTE_E4, NOTE_G4, NOTE_G3,
  NOTE_C4, NOTE_G3, NOTE_E3, NOTE_A3, NOTE_B3, NOTE_AS3, NOTE_A3, NOTE_G3, NOTE_E4, NOTE_G4, NOTE_A4, NOTE_F4, NOTE_G4, NOTE_E4, NOTE_C4, NOTE_D4, NOTE_B3,
  NOTE_C4, NOTE_G3, NOTE_E3, NOTE_A3, NOTE_B3, NOTE_AS3, NOTE_A3, NOTE_G3, NOTE_E4, NOTE_G4, NOTE_A4, NOTE_F4, NOTE_G4, NOTE_E4, NOTE_C4, NOTE_D4, NOTE_B3,
  NOTE_G4, NOTE_FS4, NOTE_E4, NOTE_DS4, NOTE_E4, NOTE_GS3, NOTE_A3, NOTE_C4, NOTE_A3, NOTE_C4, NOTE_D4, NOTE_G4, NOTE_FS4, NOTE_E4, NOTE_DS4, NOTE_E4, NOTE_C5, NOTE_C5, NOTE_C5,
  NOTE_G4, NOTE_FS4, NOTE_E4, NOTE_DS4, NOTE_E4, NOTE_GS3, NOTE_A3, NOTE_C4, NOTE_A3, NOTE_C4, NOTE_D4, NOTE_DS4, NOTE_D4, NOTE_C4,
  NOTE_C4, NOTE_C4, NOTE_C4, NOTE_C4, NOTE_D4, NOTE_E4, NOTE_C4, NOTE_A3, NOTE_G3, NOTE_C4, NOTE_C4, NOTE_C4, NOTE_C4, NOTE_D4, NOTE_E4,
  NOTE_C4, NOTE_C4, NOTE_C4, NOTE_C4, NOTE_D4, NOTE_E4, NOTE_C4, NOTE_A3, NOTE_G3
  };

/* 节拍(rhythm) */
static int noteDurations[98] = {
  8,4,4,8,4,2,2,
  3,3,3,4,4,8,4,8,8,8,4,8,4,3,8,8,3,
  3,3,3,4,4,8,4,8,8,8,4,8,4,3,8,8,2,
  8,8,8,4,4,8,8,4,8,8,3,8,8,8,4,4,4,8,2,
  8,8,8,4,4,8,8,4,8,8,3,3,3,1,
  8,4,4,8,4,8,4,8,2,8,4,4,8,4,1,
  8,4,4,8,4,8,4,8,2
};


const static uint8_t buzzerPin = 3;    ///< 按键状态检测(detect button status)
const static uint8_t keyPin = A3;
bool keyState;                              ///< 按键状态检测(detect button status)
bool taskStart = 0;
void setup() {
  pinMode(keyPin, INPUT);
  Serial.begin(9600);
  // 设置串行端口读取数据的超时时间(Set the timeout for serial port data reading)
  Serial.setTimeout(500);
}

void loop() {
  keyState = analogRead(keyPin);  //检测按键状态(detect button status)
  if (!keyState) taskStart = 1;
  if (taskStart)
  {
    tune_task();    // 播放音乐(Play music)
    taskStart = 0;
  }
}


void tune_task(void) {
  for (int thisNote = 0; thisNote <98; thisNote++)
  {
    int noteDuration = 1000/noteDurations[thisNote];// 计算每个节拍的时间，以一个节拍一秒为例，四分之一拍就是1000/4毫秒，八分之一拍就是1000/8毫秒(Calculate the time for each beat, assuming one beat per second, a quarter note is 1000/4 milliseconds, and an eighth note is 1000/8 milliseconds)
    tone(buzzerPin, song[thisNote],noteDuration);
    int pauseBetweenNotes = noteDuration * 1.10; //每个音符间的停顿间隔，以该音符的130%为佳(The pause interval between each note, 130% of the duration of this note is recommended)
    delay(pauseBetweenNotes);
    noTone(buzzerPin);
  }
}
