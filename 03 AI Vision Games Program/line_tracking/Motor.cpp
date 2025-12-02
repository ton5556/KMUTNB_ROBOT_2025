#include "Motor.h"

static uint32_t previousTime_us = 0;

const static uint8_t motorpwmPin[4] = { FRONT_LEFT_WHEEL_PWM_PIN,  \
                                        FRONT_RIGHT_WHEEL_PWM_PIN, \
                                        REAR_RIGHT_WHEEL_PWM_PIN,  \
                                        REAR_LEFT_WHEEL_PWM_PIN} ;

const static uint8_t motordirectionPin[4] = { FRONT_LEFT_WHEEL_DIRECTION_PIN,  \
                                              FRONT_RIGHT_WHEEL_DIRECTION_PIN, \
                                              REAR_RIGHT_WHEEL_DIRECTION_PIN,  \
                                              REAR_LEFT_WHEEL_DIRECTION_PIN};

static float invSqrt(float number)
{
    volatile long i;
    volatile float x, y;
    volatile const float f = 1.5F;

    x = number * 0.5F;
    y = number;
    i = * (( long * ) &y);
    i = 0x5f375a86 - ( i >> 1 );
    y = * (( float * ) &i);
    y = y * ( f - ( x * y * y ) );
    return y;
}

/* TImer1定时器PWM输出(Timer1 timer PWM output) */
static void Motors_Set(int16_t Motor_0, int16_t Motor_1, int16_t Motor_2, int16_t Motor_3)
{
  int16_t pwm_set[4];
  int16_t motors[4] = { Motor_0, Motor_1, Motor_2, Motor_3};
  /* 前进 左边轮子需置1 右边轮子需置0(Forward:the left wheel needs to be set to 1 and the right wheel needs to be set to 0) */
  bool direction[4] = { 1, 0, 0, 1};                            
  for(uint8_t i; i < 4; ++i)
  {
    if(motors[i] < 0) direction[i] = !direction[i];
    else direction[i] = direction[i];

    if(motors[i] == 0) pwm_set[i] = 0;
    else pwm_set[i] = map(abs(motors[i]), 0, 100, 0, 255);

    digitalWrite(motordirectionPin[i], direction[i]); 
    analogWrite(motorpwmPin[i], pwm_set[i]); 
  }
}

static void PWM_Out(uint8_t PWM_Pin, int8_t DutyCycle) { //定义一个脉冲函数(define a pulse width function)
  uint32_t currentTime_us = micros();
  int highTime = (PWM_PERIOD/100) * DutyCycle;
  int lowTime = PWM_PERIOD - highTime;

  if ((currentTime_us - previousTime_us) <= highTime) 
  {  
    digitalWrite(PWM_Pin, HIGH);
  }
  else digitalWrite(PWM_Pin, LOW);
  if (currentTime_us - previousTime_us >= PWM_PERIOD) 
  {
    previousTime_us = currentTime_us;
  }
}

/* 软件模拟PWM输出(soft analog PWM output) */
static void _Motors_Set(int16_t Motor_0, int16_t Motor_1, int16_t Motor_2, int16_t Motor_3)
{
  int8_t pwm_set[4];
  int8_t motors[4] = { Motor_0, Motor_1, Motor_2, Motor_3};
  bool direction[4] = { 1, 0, 0, 1};///< 前进 左1 右0(forward; left 1; right 0)
  for(uint8_t i; i < 4; ++i) {
    if(motors[i] < 0) direction[i] = !direction[i];
    else direction[i] = direction[i];

    if(motors[i] == 0) pwm_set[i] = 0;
    else pwm_set[i] = abs(motors[i]);

    digitalWrite(motordirectionPin[i], direction[i]); 
    PWM_Out(motorpwmPin[i], pwm_set[i]);
  }
}

void Velocity_Controller(uint16_t angle, uint8_t velocity,int8_t rot, uint8_t mode)
{
  int16_t velocity_0, velocity_1, velocity_2, velocity_3;
  /* 速度因子(speed factor) */
  float speed = 1;                                               
  /* 设定初始方向(set initial direction) */
  angle += 90;                                                 
  float rad = angle * PI / 180;
  if (rot == 0) speed = 1;
  else speed = 0.5; 
  velocity *= invSqrt(2);
  velocity_0 = (velocity * sin(rad) - velocity * cos(rad)) * speed + rot * speed;
  velocity_1 = (velocity * sin(rad) + velocity * cos(rad)) * speed - rot * speed;
  velocity_2 = (velocity * sin(rad) - velocity * cos(rad)) * speed - rot * speed;
  velocity_3 = (velocity * sin(rad) + velocity * cos(rad)) * speed + rot * speed;
  switch (mode)
  {
  case PWM_CONTROL:
    Motors_Set(velocity_0, velocity_1, velocity_2, velocity_3);
    break;

  case SIMULATE_PWM_CONTROL:
    _Motors_Set(velocity_0, velocity_1, velocity_2, velocity_3);
    break; 

  default:
    break;
  }
}


void Motors_Initialize(void)
{
  for(uint8_t i = 0; i < 4; i++)
  {
    pinMode(motordirectionPin[i], OUTPUT);
    pinMode(motorpwmPin[i], OUTPUT);
  }
  Velocity_Controller( 0, 0, 0, SIMULATE_PWM_CONTROL);
}



