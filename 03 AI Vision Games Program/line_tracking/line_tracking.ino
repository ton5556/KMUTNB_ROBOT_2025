#include "MultiTimer.h"
#include "Motor.h"
#include "hw_esp32s3cam_ctl.h"
#include "Tracking_Calculation.h"


Timers timer1;
Timers timer2;
Timers timer3;
Timers timer4;

HW_ESP32S3CAM hw_cam;


 /* 红色：0 黄色：1 绿色：2 蓝色：3 黑色：4(red：0; yellow：1; green：2; blue：3; black：4) */
const static uint8_t tracking_color = 0;
const static uint8_t target_point = 80;

static uint16_t set_angle = 0;
static int8_t set_rot = 0;
static int8_t set_speed = 0;

static uint8_t time_count = 0;

static uint8_t first_calibration_data;
static uint8_t second_calibration_data;
static uint8_t first_block_data[LINE_PATROL_REG_COUNT];
static uint8_t second_block_data[LINE_PATROL_REG_COUNT];

uint32_t ticksGetFunc(void)
{
  return micros();
}

/* 每20ms进行位置坐标计算(position coordinates are calculated every 20ms) */
void timerPIDCalculateCallBack(Timers *ptimer, const void *userdata)
{
  int bias_average;
  /* 寻线坐标参数获取(Obtain line following coordinate parameters) */
  hw_cam.Linepatrol_Data_Receive(LINE_PATROL_THIRD_COLOR_REG1, first_block_data, sizeof(first_block_data));
  hw_cam.Linepatrol_Data_Receive(LINE_PATROL_THIRD_COLOR_REG2, second_block_data, sizeof(second_block_data));

  
  bias_average = (first_block_data[1] + second_block_data[1]) / 2;

  if(first_block_data[0] == tracking_color && second_block_data[0] == tracking_color) 
  {
    set_speed = 50;
    Position_Control(&bias_average, &set_rot, &target_point);
    timerStart(ptimer, 60000, timerPIDCalculateCallBack, userdata);
    first_calibration_data = first_block_data[1];     /* 记录色块位置，丢线时做判断处理(the position of the block is recorded and used for error handling in case the object is lost) */
    second_calibration_data = second_block_data[1];
  }
  else if(first_block_data[0] != tracking_color || second_block_data[0] != tracking_color)
  {
    first_calibration_data = first_block_data[1];
    second_calibration_data = second_block_data[1];
    set_speed = 0;
    set_rot = 0;
    timerStop(ptimer);
    timerStart(&timer2, 100000, timerPositionCalibrationCallBack, "60ms cycle position calibration");
  }
}

/* 丢线处理定时器回调(position calibration timer callback) */
void timerPositionCalibrationCallBack(Timers *ptimer, const void *userdata)
{
  hw_cam.Linepatrol_Data_Receive(LINE_PATROL_THIRD_COLOR_REG1, first_block_data, sizeof(first_block_data));
  hw_cam.Linepatrol_Data_Receive(LINE_PATROL_THIRD_COLOR_REG2, second_block_data, sizeof(second_block_data));
  if(first_block_data[0] == tracking_color && second_block_data[0] == tracking_color)
  {
    timerStop(ptimer);
    timerStart(&timer1, 60000, timerPIDCalculateCallBack, userdata);
  }
  else
  {
    if(first_calibration_data >= 80 || second_calibration_data >= 80)
    {
      set_angle = 0;
      set_speed = 40;
      set_rot = -15;
    }
    else if(first_calibration_data < 80 || second_calibration_data < 80)
    {
      set_angle = 0;
      set_speed = 40;
      set_rot = 15;
    }
    timerStart(ptimer, 100000, timerPositionCalibrationCallBack, userdata);
  }
}

void setup() {
  Serial.begin(115200);
  ticksFuncSet(ticksGetFunc);
  Motors_Initialize();
  hw_cam.Init();
  Calculation_Init();
  timerStart(&timer1, 60000, timerPIDCalculateCallBack, "60ms cycle PID calculate");
}

void loop() {
  timersTaskRunning();
  Velocity_Controller(set_angle, set_speed, set_rot, SIMULATE_PWM_CONTROL);
}


