#ifndef __TRACKING_CALCULATION_H_
#define __TRACKING_CALCULATION_H_
#include <Arduino.h>
#include <PID_v1.h>

#define IMAGE_WIDTH_SIZE 160
#define IMAGE_LENGTH_SIZE 120

#define AGGRESSIVE_KP 0.5
#define AGGRESSIVE_KI 0
#define AGGRESSIVE_KD 0.2

#define CONSERVATIVE_KP 0.2
#define CONSERVATIVE_KI 0
#define CONSERVATIVE_KD 0.01

typedef struct
{
  double Setpoint;
  double Input;
  double Output;
  double aggKp;
  double aggKi;
  double aggKd;
  double consKp; 
  double consKi;
  double consKd;
}PID_Parameter_t;

typedef enum
{
  NO_ERR = 0,
  ERR
}err_check;



/*PID参数初始化(Initialize PID parameter)*/
void Calculation_Init(void);

/**
 * @brief 位置控制函数(position control function)
 * @param *current_point           指向当前x轴坐标点的指针(a pointer to the current x-axis coordinate)
 * @param *set_speed               指向设置转向速度的指针(a pointer to set turning speed)
 * @param *target_point        用于控制小车的自转速度，取值为-100~100，若大于0小车有一个逆时针的自转速度，若小于0则有一个顺时针的自转速度。(Used to control the rotation speed of the car, ranging from -100 to 100. If it is greater than 0, the car has a counterclockwise rotation speed; if it is less than 0, it has a clockwise rotation speed)
 * @retval err_check
 */
err_check Position_Control(int *current_point, int8_t *set_speed, uint8_t *target_point);
#endif
