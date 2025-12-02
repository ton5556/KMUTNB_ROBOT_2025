/**
 * @file tracking_test.ino
 * @author Anonymity(Anonymity@hiwonder.com)
 * @brief 巡线（Line following）
 * @version V1.0
 * @date 2024-04-23
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <Wire.h>
#include "FastLED.h"

/* 巡线传感器的iic地址(I2C address of the line follower) */
#define LINE_FOLLOWER_I2C_ADDR 0x78 

const static uint8_t ledPin = 2;
const static uint8_t keyPin = 3;
const static uint8_t pwm_min = 50;
const static uint8_t motorpwmPin[4] = { 10, 9, 6, 11 };
const static uint8_t motordirectionPin[4] = { 12, 8, 7, 13 };
const static uint8_t TRACKING = 4;

static CRGB rgbs[1];
static uint8_t modestate = TRACKING;

uint8_t state = 0;
uint8_t data;
uint8_t rec_data[4];

bool keyState;  // 按键状态检测(Detect button status)
bool taskStart = 0;
bool WireWriteByte(uint8_t val);
bool WireReadDataByte(uint8_t reg, uint8_t &val);

void Motor_Init(void);
void Velocity_Controller(uint16_t angle, uint8_t velocity, int8_t rot, bool drift);
void Motors_Set(int8_t Motor_0, int8_t Motor_1, int8_t Motor_2, int8_t Motor_3);
void Sensor_Receive(void);
void Tracking_Line_Task(void);
void Task_Dispatcher(void);
void Rgb_Show(uint8_t rValue, uint8_t gValue, uint8_t bValue);



void setup() {
  /* 配置通信(condigure communication) */
  Serial.begin(9600);

  pinMode(keyPin, INPUT);
  FastLED.addLeds<WS2812, ledPin, GRB>(rgbs, 1);
  Rgb_Show(255, 255, 255);

  Wire.begin();
  Motor_Init();
}

void loop() {
  keyState = analogRead(keyPin);
  if (!keyState) taskStart = 1;
  if (taskStart) {
    Sensor_Receive();
    Task_Dispatcher();
  }
}

bool WireWriteByte(uint8_t val) {
  Wire.beginTransmission(LINE_FOLLOWER_I2C_ADDR); /* 选择地址开始传输(Select address to start transmission) */
  Wire.write(val);                                //发送数据(Send data)
  if (Wire.endTransmission() != 0) {
    // Serial.println("false");  /* 发送失败(fail to send) */
    return false;
  }
  // Serial.println("true");   /* 发送成功(send successfully) */
  return true;
}
/* 按字节读数据(Read data based on byte) */
bool WireReadDataByte(uint8_t reg, uint8_t &val) {
  if (!WireWriteByte(reg)) { /* 给传感器发送读/写信号(Send read/write signal to line follower) */
    return false;
  }
  Wire.requestFrom(LINE_FOLLOWER_I2C_ADDR, 1); /* 接收到传感器的应答信号(the response signal of the line follower is received) */
  while (Wire.available()) {                   /* 开始读取数据(start to read data) */
    val = Wire.read();
  }
  return true;
}

/* 电机初始化函数(Motor initialization function) */
void Motor_Init(void) {
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(motordirectionPin[i], OUTPUT);
  }
  Velocity_Controller(0, 0, 0, 0);
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
void Velocity_Controller(uint16_t angle, uint8_t velocity, int8_t rot, bool drift) {
  int8_t velocity_0, velocity_1, velocity_2, velocity_3;
  float speed = 1;
  angle += 90;
  float rad = angle * PI / 180;
  if (rot == 0) speed = 1;  // 速度因子(Speed factor)
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
  int8_t motors[4] = { Motor_0, Motor_1, Motor_2, Motor_3 };
  bool direction[4] = { 1, 0, 0, 1 };  // 前进 左1 右0(Forward; left 1; right 0)
  for (uint8_t i; i < 4; ++i) {
    if (motors[i] < 0) direction[i] = !direction[i];
    else direction[i] = direction[i];

    if (motors[i] == 0) pwm_set[i] = 0;
    else pwm_set[i] = map(abs(motors[i]), 0, 100, pwm_min, 255);

    digitalWrite(motordirectionPin[i], direction[i]);
    analogWrite(motorpwmPin[i], pwm_set[i]);
  }
}

/* 获取传感器数据(Obtain the sensor data) */
void Sensor_Receive(void) {
  WireReadDataByte(1, data);
  rec_data[0] = data & 0x01;
  rec_data[1] = (data >> 1) & 0x01;
  rec_data[2] = (data >> 2) & 0x01;
  rec_data[3] = (data >> 3) & 0x01;
}

void Tracking_Line_Task(void) {
  Rgb_Show(255, 0, 0);
  if (rec_data[1] == 1 && rec_data[2] == 1) {
    Velocity_Controller(0, 80, 0, 0);
  }
  if (rec_data[1] == 1 && rec_data[2] == 0) {
    Velocity_Controller(0, 80, 65, 0);
  }
  if (rec_data[1] == 0 && rec_data[2] == 1) {
    Velocity_Controller(0, 80, -65, 0);
  }
  while (rec_data[1] == 0 && rec_data[2] == 0) {
    Sensor_Receive();
    Velocity_Controller(0, 0, 0, 0);
  }
}

/* 任务调度(Task dispatcher) */
void Task_Dispatcher() {
  switch (modestate) {
    case TRACKING:
      Tracking_Line_Task();
      break;
  }
}

/**
 * @brief 设置RGB灯的颜色(Set the color of RGB LED)
 * @param rValue;gValue;bValue;
 * @arg 三个入口参数取值分别为:0~255;(Three input parameter value: 0 to 255)
 * @retval None
 * @note (255,0,0)红色 (0,255,0)绿色 (0,0,255)蓝色 (255,255,255)白色((255,0,0)red; (0,255,0)green; (0,0,255)blue; (255,255,255)white)
 */
void Rgb_Show(uint8_t rValue, uint8_t gValue, uint8_t bValue) {
  rgbs[0].r = rValue;
  rgbs[0].g = gValue;
  rgbs[0].b = bValue;
  FastLED.show();
}