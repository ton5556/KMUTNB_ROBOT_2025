#ifndef __TRACK_PROCESSING_H_
#define __TRACK_PROCESSING_H_
#include <Arduino.h>
#include <PID_v1.h>

#define IMAGE_WIDTH_SIZE 160
#define IMAGE_LENGTH_SIZE 120

#define AGGRESSIVE_KP 1.5
#define AGGRESSIVE_KI 0
#define AGGRESSIVE_KD 0.4

#define CONSERVATIVE_KP 0
#define CONSERVATIVE_KI 0
#define CONSERVATIVE_KD 0

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

void Controller_Init(void);
void Position_Controller(uint8_t start_x, uint8_t start_y, uint8_t end_x, uint8_t end_y, uint8_t *new_angle, uint8_t *speed);
#endif
