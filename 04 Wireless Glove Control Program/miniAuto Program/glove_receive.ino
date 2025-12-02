/**
 * @file glove_receive.ino
 * @brief 体感手套控制(Wireless glove control)
 * @author Anonymity(Anonymity@hiwonder.com)
 * @version V1.0
 * @date 2024-04-22
 *
 * @copyright Copyright (c) 2023
 *
 * 10-12 [0] --|||--[1] 9-8
 *        |          |
 *        |          |
 *        |          |
 *        |          |
 * 11-13 [3] -------[2] 6-7
 */
#include <Arduino.h>
#include "FastLED.h"

#define FRAME_HEADER            0x55

struct blue_info{
  uint8_t rec_num; //需要接收的数量(The number of bytes to be received)
  uint8_t func; 
  int8_t buf[128];
};

struct blue_info rec_oj;
struct blue_info resule_oj;

const static uint8_t ledPin = 2;
const static uint8_t pwm_min = 50;
const static uint8_t motorpwmPin[4] = { 10, 9, 6, 11} ;
const static uint8_t motordirectionPin[4] = { 12, 8, 7, 13};
static CRGB rgbs[1];

void Motor_Init(void);
void receive_handlder(void);
void Derection_Control(void);
void Rgb_Show(uint8_t rValue,uint8_t gValue,uint8_t bValue);
void Velocity_Controller(uint16_t angle, uint8_t velocity,int8_t rot);
void Motors_Set(int8_t Motor_0, int8_t Motor_1, int8_t Motor_2, int8_t Motor_3);

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(500);              ///< 设置串行端口读取数据的超时时间(Set timeout of reading data from serial port)
  FastLED.addLeds<WS2812, ledPin, RGB>(rgbs, 1);
  Rgb_Show(255,255,255);
  Motor_Init();
}

void loop() {
  // put your main code here, to run repeatedly:
  receive_handlder();
  Derection_Control();
}

 /* 手套数据接收处理函数(Glove data receiving and handling function) */
void receive_handlder() {
  static uint8_t step = 0;
  static uint8_t head_count = 0;
  static uint8_t data_count = 0;
  while (Serial.available() > 0)
  {
    char rx = Serial.read();
    switch(step)
    {
      case 0: //帧头(Frame header)
        if(rx == FRAME_HEADER)
        {
          head_count++;
          if(head_count > 1)
          {
            step++;
            head_count = 0;
          }
        }else{
          head_count = 0;
        }
        break;
      case 1: //接收数(Number of bytes to receive)
        if(rx > 0 || rx < 128)
        {
          rec_oj.rec_num = rx;
          step++;
        }else{
          step = 0;
        }
        break;
      case 2: //功能号(Function number)
        if(rx > 0)
        {
          rec_oj.func = rx;
          step++;
        }else{
          step = 0;
        }
        break;
      case 3:
        rec_oj.buf[data_count] = rx;
        data_count++;
        if(data_count > rec_oj.rec_num - 2)
        {
          resule_oj.rec_num = rec_oj.rec_num;
          resule_oj.func = rec_oj.func;
          memcpy(resule_oj.buf , rec_oj.buf , rec_oj.rec_num -2);
          data_count = 0;
          step = 0;
        }
        break;
      default:
        Serial.println("-def-");
        step = 0;
        break;
    }

  }
  // Serial.print(resule_oj.buf[0]);
  // Serial.print(",");
  // Serial.println(resule_oj.buf[1]); //-100,-100Back -100,100Left 100,-100Right 100,100forward 0,0Stop
}

 /**
 * @brief 设置RGB灯的颜色(Set the color of RGB LED)
 * @param rValue;gValue;bValue;
 * @arg 三个入口参数取值分别为:0~255;(Three input parameters range from 0 to 255)
 * @retval None
 * @note (255,0,0)绿色 (0,255,0)红色 (0,0,255)蓝色 (255,255,255)白色((255,0,0)green; (0,255,0)red (0,0,255)blue; (255,255,255)white)
 */
void Rgb_Show(uint8_t rValue,uint8_t gValue,uint8_t bValue) {
  rgbs[0].r = rValue;
  rgbs[0].g = gValue;
  rgbs[0].b = bValue;
  FastLED.show();
}

 /* 手套方向控制函数(Glove direction control function) */
void Derection_Control() {
  if(resule_oj.buf[0] == 0 && resule_oj.buf[1] == 0)
    Velocity_Controller( 0, 0, 0);
  if(resule_oj.buf[0] == 100 && resule_oj.buf[1] == 100)
    Velocity_Controller( 0, 100, 0);
  if(resule_oj.buf[0] == -100 && resule_oj.buf[1] == 100)
    Velocity_Controller( 90, 100, 0);
  if(resule_oj.buf[0] == 100 && resule_oj.buf[1] == -100)
    Velocity_Controller( 270, 100, 0); 
  if(resule_oj.buf[0] == -100 && resule_oj.buf[1] == -100)
    Velocity_Controller( 180, 100, 0);
}

 /* 电机初始化函数(Motor initialization function) */
void Motor_Init(void){
  for(uint8_t i = 0; i < 4; i++){
    pinMode(motordirectionPin[i], OUTPUT);
  }
  Velocity_Controller( 0, 0, 0);
}

/**
 * @brief 速度控制函数(Speed control function)
 * @param angle   用于控制小车的运动方向，小车以车头为0度方向，逆时针为正方向。(Used to control the direction of the car, with the front of the car as the 0 degree direction and counterclockwise as the positive direction)
 *                取值为0~359(Range from 0 to 359)
 * @param velocity   用于控制小车速度，取值为0~100。(Used to control the speed of the car, with values from 0 to 100)
 * @param rot     用于控制小车的自转速度，取值为-100~100，若大于0小车有一个逆
 *                 时针的自转速度，若小于0则有一个顺时针的自转速度。(Used to control the rotation speed of the car, with values from -100 to 100. If it is greater than 0, the car has a counterclockwise rotation speed; if it is less than 0, the car has a clockwise rotation speed)
 * @param drift   用于决定小车是否开启漂移功能，取值为0或1，若为0则开启，反之关闭。(Used to determine whether the car has drift function. Takes values of 0 or 1. If it is 0, the drift function is enabled; otherwise, it is disabled)
 * @retval None
 */
void Velocity_Controller(uint16_t angle, uint8_t velocity,int8_t rot) {
  int8_t velocity_0, velocity_1, velocity_2, velocity_3;
  float speed = 1;
  angle += 90;
  float rad = angle * PI / 180;
  if (rot == 0) speed = 1;///< 速度因子(Speed factor)
  else speed = 0.5; 
  velocity /= sqrt(2);
  velocity_0 = (velocity * sin(rad) - velocity * cos(rad)) * speed + rot * speed;
  velocity_1 = (velocity * sin(rad) + velocity * cos(rad)) * speed - rot * speed;
  velocity_2 = (velocity * sin(rad) - velocity * cos(rad)) * speed - rot * speed;
  velocity_3 = (velocity * sin(rad) + velocity * cos(rad)) * speed + rot * speed;
  Motors_Set(velocity_0, velocity_1, velocity_2, velocity_3);
}
 
/**
 * @brief PWM与轮子转向设置函数(PWM and wheel direction setting function)
 * @param Motor_x   作为PWM与电机转向的控制数值。根据麦克纳姆轮的运动学分析求得。(The control value for PWM and motor direction. Obtained based on the kinematic analysis of the Mecanum wheel)
 * @retval None
 */
void Motors_Set(int8_t Motor_0, int8_t Motor_1, int8_t Motor_2, int8_t Motor_3) {
  int8_t pwm_set[4];
  int8_t motors[4] = { Motor_0, Motor_1, Motor_2, Motor_3};
  bool direction[4] = { 1, 0, 0, 1};///< 前进 左1 右0(Forward: Left 1, Right 0)
  for(uint8_t i; i < 4; ++i) {
    if(motors[i] < 0) direction[i] = !direction[i];
    else direction[i] = direction[i];

    if(motors[i] == 0) pwm_set[i] = 0;
    else pwm_set[i] = map(abs(motors[i]), 0, 100, pwm_min, 255);

    digitalWrite(motordirectionPin[i], direction[i]); 
    analogWrite(motorpwmPin[i], pwm_set[i]); 
  }
}