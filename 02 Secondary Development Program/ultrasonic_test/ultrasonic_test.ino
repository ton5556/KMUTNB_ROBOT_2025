/**
 * @file ultrasonic_test.ino
 * @brief 超声波距离测试(Ultrasonic ranging)
 * @author Anonymity(Anonymity@hiwonder.com)
 * @version V1.0
 * @date 2024-04-23
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "Ultrasound.h"

#define FILTER_N 3                ///< 滤波法数组容量(Filter array capacity)

int Filter_Value;
int filter_buf[FILTER_N + 1];

Ultrasound ultrasound;            ///< 实例化超声波类(Instantiate the ultrasound class)

int Filter(void);
int ultrasonic_distance(void);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  ultrasonic_distance();
}

/* 递推平均滤波法(Recursive average filtering method)*/
int Filter() {
  int i;
  int filter_sum = 0;
  filter_buf[FILTER_N] = ultrasound.GetDistance();///< 读取超声波测值(Read the ultrasonic ranging value)
  for(i = 0; i < FILTER_N; i++) {
    filter_buf[i] = filter_buf[i + 1];///< 所有数据左移，低位仍掉(Shift all data to the left, and clear the low bit)
    filter_sum += filter_buf[i];
  }
  return (int)(filter_sum / FILTER_N);
}

int ultrasonic_distance(){
  uint8_t s;
  uint16_t distance = Filter();///< 获得滤波器输出值(Get the output value of the filter)
  Serial.print("Distance: ");///< 获取并且串口打印距离，单位mm(Get and print the distance via serial port, unit: mm)
  Serial.print(distance);
  Serial.println(" mm");

  if (distance > 0 && distance <= 80){
      ultrasound.Breathing(1, 0, 0, 1, 0, 0);///< 呼吸灯模式，周期0.1s，颜色红色(Red breathing light mode, in 0.1s)
  }
   
  else if (distance > 80 && distance <= 180){
      s = map(distance,80,180,0,255);
      ultrasound.Color((255-s), 0, 0, (255-s), 0, 0);///< 红色渐变(Red gradient)
  }
   
   else if (distance > 180 && distance <= 320){
      s = map(distance,180,320,0,255);
      ultrasound.Color(0, 0, s, 0, 0, s);///< 蓝色渐变(Blue gradient)
  }
   
   else if (distance > 320 && distance <= 500){
      s = map(distance,320,500,0,255);
      ultrasound.Color(0, s, 255-s, 0, s, 255-s);///< 绿色渐变(Green gradient)
  }
  else if (distance > 500){
      ultrasound.Color(0, 255, 0, 0, 255, 0);///< 绿色(Green)
  }
  return distance;  
}