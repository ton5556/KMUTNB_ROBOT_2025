/**
 * @file ultrasonic_following.ino
 * @author Anonymity(Anonymity@hiwonder.com)
 * @brief 超声波跟随(Ultrasonic following)
 * @version V1.0
 * @date 2024-04-23
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "Ultrasound.h"

#define STOP 0
#define FORWARD 1
#define BACKWARD 2
#define FILTER_N 3                //递推平均滤波法(Recursive average filtering method)

Ultrasound ultrasound;  //实例化超声波类(Instantiate the ultrasound class)

const static uint8_t pwm_min = 50;
const static uint8_t motorpwmPin[4] = { 10, 9, 6, 11} ;
const static uint8_t motordirectionPin[4] = { 12, 8, 7, 13};

uint16_t dis;
uint16_t ultrasonic_distance(void);
int filter_buf[FILTER_N + 1];
int Filter(void);
void Motor_Init(void);
void Velocity_Controller(uint16_t angle, uint8_t velocity,int8_t rot,bool drift);
void Motors_Set(int8_t Motor_0, int8_t Motor_1, int8_t Motor_2, int8_t Motor_3);

void setup() {
  Serial.begin(9600);
  Motor_Init();
}

void loop() {
  ultrasonic_distance();
  dis = ultrasonic_distance();
  if(dis >=700) Velocity_Controller( 0, 0, 0, 0);
  if(dis >= 300 && dis < 700) Velocity_Controller( 0,50, 0, 0);
  if(dis >= 200 && dis < 300) Velocity_Controller( 0, 0, 0, 0);
  if(dis < 200) Velocity_Controller( 180, 50, 0, 0); 
}

 /**
 * @brief 滤波(filter)
 * @param filter_sum / FILTER_N
 * @arg None
 * @retval None
 * @note None
 */
int Filter() {
  int i;
  int filter_sum = 0;
  filter_buf[FILTER_N] = ultrasound.GetDistance();     //读取超声波测值(Read the ultrasonic ranging value)
  for(i = 0; i < FILTER_N; i++) {
    filter_buf[i] = filter_buf[i + 1];               // 所有数据左移，低位仍掉(Shift all data to the left, and clear the low bit)
    filter_sum += filter_buf[i];
  }
  return (int)(filter_sum / FILTER_N);
}

/* 超声波距离数据获取(Obtain ultrasonic distance data) */
uint16_t ultrasonic_distance(){
  uint8_t s;
  uint16_t distance = Filter();         // 获得滤波器输出值(Get the output value of the filter)
  Serial.print("Distance: ");Serial.print(distance);Serial.println(" mm"); //获取并且串口打印距离，单位mm(Get and print the distance via serial port, unit: mm)

  if (distance > 0 && distance <= 80){
      ultrasound.Breathing(1, 0, 0, 1, 0, 0);       //呼吸灯模式，周期0.1s，颜色红色(Red breathing light mode, in 0.1s)
   }
   
  else if (distance > 80 && distance <= 180){
      s = map(distance,80,180,0,255);
      ultrasound.Color((255-s), 0, 0, (255-s), 0, 0); //红色渐变(Red gradient)
   }
   
   else if (distance > 180 && distance <= 320){
      s = map(distance,180,320,0,255);
      ultrasound.Color(0, 0, s, 0, 0, s);            //蓝色渐变(Blue gradient)
   }
   
   else if (distance > 320 && distance <= 500){
      s = map(distance,320,500,0,255);
      ultrasound.Color(0, s, 255-s, 0, s, 255-s);            //绿色渐变(Green gradient)
   }
  else if (distance > 500){
      ultrasound.Color(0, 255, 0, 0, 255, 0);        //绿色(Green)
   }
  return distance;  
}

 /* 电机初始化函数(Motor initialization function) */
void Motor_Init(void){
  for(uint8_t i = 0; i < 4; i++){
    pinMode(motordirectionPin[i], OUTPUT);
  }
  Velocity_Controller( 0, 0, 0, 0);
}

/**
 * @brief 速度控制函数(Speed control function)
 * @param angle   用于控制小车的运动方向，小车以车头为0度方向，逆时针为正方向。("angle" controls the robot's motion direction, with the front of the robot as 0 degrees and counterclockwise as the positive direction)
 *                取值为0~359(range from 0 to 359)
 * @param velocity   用于控制小车速度，取值为0~100。("velocity" controls the robot's speed, with a value range of 0 to 100)
 * @param rot     用于控制小车的自转速度，取值为-100~100，若大于0小车有一个逆("rot" controls the robot's self-rotation speed, with a value range of -100 to 100)
 *                 时针的自转速度，若小于0则有一个顺时针的自转速度。(If it is greater than 0, the robot has a counterclockwise self-rotation speed. If it is less than 0, the robot has a clockwise self-rotation speed)
 * @param drift   用于决定小车是否开启漂移功能，取值为0或1，若为0则开启，反之关闭。("drift" determines whether the robot enables drift. Value range: 0 or 1. If it is 0, drift is enabled; otherwise, it is disabled)
 * @retval None
 */
void Velocity_Controller(uint16_t angle, uint8_t velocity,int8_t rot,bool drift) {
  int8_t velocity_0, velocity_1, velocity_2, velocity_3;
  float speed = 1;
  angle += 90;
  float rad = angle * PI / 180;
  if (rot == 0) speed = 1;///< 速度因子(Speed factor)
  else speed = 0.5; 
  velocity /= sqrt(2);
  if (drift) {
    velocity_0 = (velocity * sin(rad) - velocity * cos(rad)) * speed;
    velocity_1 = (velocity * sin(rad) + velocity * cos(rad)) * speed;
    velocity_2 = (velocity * sin(rad) - velocity * cos(rad)) * speed - rot * speed * 2;
    velocity_3 = (velocity * sin(rad) + velocity * cos(rad)) * speed + rot * speed * 2;
  } else {
    velocity_0 = (velocity * sin(rad) - velocity * cos(rad)) * speed + rot * speed;
    velocity_1 = (velocity * sin(rad) + velocity * cos(rad)) * speed - rot * speed;
    velocity_2 = (velocity * sin(rad) - velocity * cos(rad)) * speed - rot * speed;
    velocity_3 = (velocity * sin(rad) + velocity * cos(rad)) * speed + rot * speed;
  }
  Motors_Set(velocity_0, velocity_1, velocity_2, velocity_3);
}
 
/**
 * @brief PWM与轮子转向设置函数(PWM and wheel turning setting function)
 * @param Motor_x   作为PWM与电机转向的控制数值。根据麦克纳姆轮的运动学分析求得。("Motor_x" is the control value for PWM and motor rotating. Calculated based on the kinematics analysis of mecanum wheels)
 * @retval None
 */
void Motors_Set(int8_t Motor_0, int8_t Motor_1, int8_t Motor_2, int8_t Motor_3) {
  int8_t pwm_set[4];
  int8_t motors[4] = { Motor_0, Motor_1, Motor_2, Motor_3};
  bool direction[4] = { 1, 0, 0, 1};///< 前进 左1 右0(Forward; left 1; right 0)
  for(uint8_t i; i < 4; ++i) {
    if(motors[i] < 0) direction[i] = !direction[i];
    else direction[i] = direction[i];

    if(motors[i] == 0) pwm_set[i] = 0;
    else pwm_set[i] = map(abs(motors[i]), 0, 100, pwm_min, 255);

    digitalWrite(motordirectionPin[i], direction[i]); 
    analogWrite(motorpwmPin[i], pwm_set[i]); 
  }
}
