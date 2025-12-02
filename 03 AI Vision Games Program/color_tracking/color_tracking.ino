#include "MultiTimer.h"
#include "Motor.h"
#include "hw_esp32s3cam_ctl.h"
#include "track_processing.h"
Timers timer1;
Timers timer2;

HW_ESP32S3CAM hw_cam;

uint8_t set_angle,set_speed;
static uint8_t buffer[COLOR_DETECTION_REG_COUNT];

uint32_t ticksGetFunc(void)
{
  return micros();
}

/* 每20ms进行位置坐标计算(Calculate the position coordinates every 20ms) */
void timerPIDCalculateCallBack(Timers *ptimer, const void *userdata)
{
  hw_cam.Linepatrol_Data_Receive(THIRD_COLOR_INFO_REG, buffer, sizeof(buffer));
  // printf("[%08ld] Timer:%p callback-> %s.\r\n", platformTicksGetFunc(), ptimer, (char*)userdata);
  Position_Controller( buffer[5], buffer[6], IMAGE_WIDTH / 2, IMAGE_LENGTH/2, &set_angle, &set_speed);
  timerStart(ptimer, 20000, timerPIDCalculateCallBack, userdata);
}

void setup() {
  Serial.begin(115200);
  ticksFuncSet(ticksGetFunc);
  Motors_Initialize();
  hw_cam.Init();
  Controller_Init();
  timerStart(&timer1, 20, timerPIDCalculateCallBack, "20ms cycle PID calculate");
}

void loop() {
   
  timersTaskRunning();
  Velocity_Controller(set_angle, set_speed, 0, SIMULATE_PWM_CONTROL);
}


