#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "Arduino.h"

/* PWM周期为20ms，即为50Hz(PWM period is 20ms, that is, 50Hz) */
#define PWM_PERIOD 20000 

#define PWM_CONTROL 0
#define SIMULATE_PWM_CONTROL 1

#define FRONT_LEFT_WHEEL_PWM_PIN            10
#define FRONT_RIGHT_WHEEL_PWM_PIN            9
#define REAR_LEFT_WHEEL_PWM_PIN             11
#define REAR_RIGHT_WHEEL_PWM_PIN             6

#define FRONT_LEFT_WHEEL_DIRECTION_PIN      12
#define FRONT_RIGHT_WHEEL_DIRECTION_PIN      8
#define REAR_LEFT_WHEEL_DIRECTION_PIN       13
#define REAR_RIGHT_WHEEL_DIRECTION_PIN       7

// #define PWM_MIN_LIMIT 50

/*电机初始化(Initialize motor)*/
void Motors_Initialize(void);

/**
 * @brief 速度控制函数(Speed control function)
 * @param angle      用于控制小车的运动方向，小车以车头为0度方向，逆时针为正方向，取值为0~359。("angle" controls the robot's motion direction, with the front of the robot as 0 degrees and counterclockwise as the positive direction. It ranges from 0 to 359)
 * @param velocity   用于控制小车速度，取值为0~100。("velocity" controls the robot's speed, with a value range of 0 to 100)
 * @param rot        用于控制小车的自转速度，取值为-100~100，若大于0小车有一个逆时针的自转速度，若小于0则有一个顺时针的自转速度。("rot" controls the robot's self-rotation speed, with a value range of -100 to 100. If it is greater than 0, the robot has a counterclockwise self-rotation speed. If it is less than 0, the robot has a clockwise self-rotation speed)
 * @param mode       mode == PWM_CONTROL，则选择定时器输出PWM。(mode == PWM_CONTROL selects the timer to output PWM signals)
 *                   mode == SIMULATE_PWM_CONTROL，则选择软件模拟输出PWM模式，当使用舵机时，需要使用该模式。(mode == SIMULATE_PWM_CONTROL selects the software simulation output PWM mode. This mode is required when using a servo motor)
 * @retval None
 */
void Velocity_Controller(uint16_t angle, uint8_t velocity, int8_t rot, uint8_t mode);
#endif