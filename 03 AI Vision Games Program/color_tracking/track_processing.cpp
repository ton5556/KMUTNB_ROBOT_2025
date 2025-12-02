#include "track_processing.h"
#include <math.h>


static PID_Parameter_t pid_parmeter = {0, 0, 0, AGGRESSIVE_KP, AGGRESSIVE_KI, AGGRESSIVE_KD,\
                                       CONSERVATIVE_KP, CONSERVATIVE_KI, CONSERVATIVE_KD};
                                
PID myPID(&pid_parmeter.Input, &pid_parmeter.Output, &pid_parmeter.Setpoint, \
                 pid_parmeter.consKp, pid_parmeter.consKi, pid_parmeter.consKd, REVERSE);


/* 平方根快速解法InvSqrt()相当于1.0/sqrt(Fast method for computing the inverse square root of a number. It is equivalent to 1.0/sqrt) */
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
/* 归一化(normalize) */
static float Normalization(float invsqrt, float numerator)
{
  return numerator * invsqrt;
}

static float Angle_Conversion(float angle, uint8_t x, uint8_t y)
{
  float new_angle;
  if((x <= IMAGE_WIDTH_SIZE / 2 && y < IMAGE_LENGTH_SIZE / 2) ||
     (x < IMAGE_WIDTH_SIZE / 2 && y >= IMAGE_LENGTH_SIZE / 2) ||
     (x >= IMAGE_WIDTH_SIZE / 2 && y > IMAGE_LENGTH_SIZE / 2))
  {
    new_angle = abs(angle - 90);
  }
  else if(x > IMAGE_WIDTH_SIZE / 2 && y <= IMAGE_LENGTH_SIZE / 2)
  {
    new_angle = abs(angle - 450);
  }
  return new_angle;
}

void Controller_Init()
{

  myPID.SetMode(AUTOMATIC);
}

/*在实际效果中，我们追求的是：(In practical application, we aim to:
      将屏幕正中心的坐标点设为运动终点，当前识别到的待追踪物体在屏幕上的坐标点为起始点，Set the coordinates of the center of the screen as the destination point. Set the coordinates of the currently detected object on the screen as the starting point.
      通过移动机体运动角度，使得之前识别到的待追踪物体的坐标移动到屏幕的中心点（也就是待追踪物体正对着机体，保持一定距离）(Adjust the angle of the robot's movement to move the previously detected object's coordinates to the center of the screen(i.e., keeping the object facing the robot and maintaining a certain distance))
      此函数循环执行，从而实现对物体的追踪(Execute this function in a loop to achieve object tracking)*/
void Position_Controller(uint8_t start_x, uint8_t start_y, uint8_t end_x, uint8_t end_y, uint8_t *new_angle, uint8_t *speed)
{
    float diff_x, diff_y, nor_diff_x, nor_diff_y, invsqrt, angle, gap, distance;
    diff_x = end_x - start_x;
    diff_y = end_y - start_y;
    invsqrt = invSqrt(diff_x * diff_x + diff_y * diff_y); //求取两个坐标点之间的直线距离(Calculate the distance between two points in a two-dimensional space)
    nor_diff_x = Normalization(invsqrt, diff_x);
    nor_diff_y = Normalization(invsqrt, diff_y);
    distance = 1 / invsqrt;
    angle = atan2(nor_diff_y, nor_diff_x) * 180 / PI;
    *new_angle = (uint8_t)Angle_Conversion(angle, start_x, start_y);
    if(start_x == 0 && start_y == 0)
    {
      diff_x = 0;
      diff_y = 0;
      distance = 0;
      angle = 0;
    }
    pid_parmeter.Setpoint = 0;
    pid_parmeter.Input = distance;
    gap = pid_parmeter.Input - pid_parmeter.Setpoint;
    if (gap < 10)
    {
      myPID.SetTunings(pid_parmeter.consKp, pid_parmeter.consKi, pid_parmeter.consKd);
    }
    else
    {
      myPID.SetTunings(pid_parmeter.aggKp, pid_parmeter.aggKi, pid_parmeter.aggKd);
    }
    myPID.Compute();
    *speed = (uint8_t)map(pid_parmeter.Output, 0, 255, 0, 100);
    // Serial.print(new_angle);Serial.print(",");Serial.println(speed);
}

