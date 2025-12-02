#include "Tracking_Calculation.h"
#include <math.h>


static PID_Parameter_t pid_parmeter = {0, 0, 0, AGGRESSIVE_KP, AGGRESSIVE_KI, AGGRESSIVE_KD,\
                                       CONSERVATIVE_KP, CONSERVATIVE_KI, CONSERVATIVE_KD};
                                
PID mypid(&pid_parmeter.Input, &pid_parmeter.Output, &pid_parmeter.Setpoint, \
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


void Calculation_Init()
{
  mypid.SetMode(AUTOMATIC);
}

err_check Position_Control(int *current_point, int8_t *set_speed, uint8_t *target_point)
{
    if(*set_speed > 100 || *set_speed <-100 || *target_point >160 || *target_point < 0)
    {
      return ERR;
    }
    int gap;
    gap = *target_point - *current_point;
    pid_parmeter.Setpoint = 0;
    pid_parmeter.Input =abs(*target_point - *current_point);
    if (pid_parmeter.Input < 10)
    {
      mypid.SetTunings(pid_parmeter.consKp, pid_parmeter.consKi, pid_parmeter.consKd);
    }
    else
    {
      mypid.SetTunings(pid_parmeter.aggKp, pid_parmeter.aggKi, pid_parmeter.aggKd);
    }
    mypid.Compute();
    if(gap >= 0)
    {
      *set_speed = map(pid_parmeter.Output, 0, 255, 0, 100);
    }
    else
    {
      *set_speed = map(pid_parmeter.Output, 0, 255, 0, -100);
    }
    return NO_ERR;
}

