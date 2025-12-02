/**
 * @file rgb_module.ino
 * @brief RGB实验(RGB control)
 * @version V1.0
 * @date 2024-04-22
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "FastLED.h"

const static uint8_t ledPin = 2;
const static uint8_t keyPin = 3;

static CRGB rgbs[1];

bool keyState;          ///< 按键状态检测(Detect button status)

void Rgb_Show(uint8_t rValue,uint8_t gValue,uint8_t bValue);

void setup() {
  Serial.begin(9600);        // 初始化串口通信(Initialize serial communication)
  
  pinMode(keyPin, INPUT);
  FastLED.addLeds<WS2812, ledPin, GRB>(rgbs, 1);
  Rgb_Show(255,255,255);

}

void loop() {
  keyState = analogRead(keyPin);
  if(keyState) Rgb_Show(255,0,0);
  else Rgb_Show(255,255,255);
  delay(100);
}

 /**
 * @brief 设置RGB灯的颜色(Set the color of the RGB light)
 * @param rValue;gValue;bValue;
 * @arg 三个入口参数取值分别为:0~255;(Three input parameters are respectively: 0~255;)
 * @retval None
 * @note (255,0,0)红色 (0,255,0)绿色 (0,0,255)蓝色 (255,255,255)白色((255,0,0) red; (0,255,0) green; (0,0,255) blue; (255,255,255) white)
 */
void Rgb_Show(uint8_t rValue,uint8_t gValue,uint8_t bValue) {
  rgbs[0].r = rValue;
  rgbs[0].g = gValue;
  rgbs[0].b = bValue;
  FastLED.show();
}
