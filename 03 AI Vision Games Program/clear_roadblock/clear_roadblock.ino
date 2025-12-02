/**
 * @file clear_roadblock.ino
 * @author Anonymity(Anonymity@hiwonder.com)
 * @brief APP遥控玩法(APP Control)
 * @version V1.0
 * @date 2024-07-25
 *
 * @copyright Copyright (c) 2024
 *
 * @attention main函数中不可以做任何阻塞处理！(No blocking processing can be done in the main function!)
 */

#include "MultiTimer.h"
#include "Motor.h"
#include "hw_esp32s3cam_ctl.h"
#include "Tracking_Calculation.h"
#include <Servo.h>

Timers timer1;
Timers timer2;
Timers timer3;
Timers timer4;

Servo myServo;
HW_ESP32S3CAM hw_cam;

/* 引脚定义(Pin definitions) */
const static uint8_t servoPin = 5;

 /* 红色：0 黄色：1 绿色：2 蓝色：3 黑色：4(Red: 0; Yellow: 1; Green: 2; Blue: 3; Black: 4) */
const static uint8_t tracking_color = 0;
const static uint8_t roadblock_color = 2;

const static uint8_t target_point = 80;


static uint16_t set_angle = 0;
static int8_t set_rot = 0;
static int8_t set_speed = 0;

static uint8_t time_count = 0;

/* 丢线时校准参数(Calibration parameters when losing the line) */
static uint8_t first_calibration_data;
static uint8_t second_calibration_data;

/* 巡线坐标参数(Parameters for line following coordinates) */
static uint8_t first_block_data[LINE_PATROL_REG_COUNT];
static uint8_t second_block_data[LINE_PATROL_REG_COUNT];

/* 障碍物坐标参数(Parameters for obstacle coordinates) */
static uint8_t roadblock_first_block_data[LINE_PATROL_REG_COUNT];
static uint8_t roadblock_second_block_data[LINE_PATROL_REG_COUNT];

void set_servo(uint8_t angle)
{
  myServo.write(angle);
}

/* 获取基准时间(Get the reference time) */
uint32_t ticksGetFunc(void)
{
  return micros();
}

/* 每60ms进行位置坐标计算(Calculate the position coordinates every 60ms) */
void timerPIDCalculateCallBack(Timers *ptimer, const void *userdata)
{
  int bias_average;
  /* 寻线坐标参数获取(Get line following coordinate parameters) */
  hw_cam.Linepatrol_Data_Receive(LINE_PATROL_FIRST_COLOR_REG1, first_block_data, sizeof(first_block_data));
  hw_cam.Linepatrol_Data_Receive(LINE_PATROL_FIRST_COLOR_REG2, second_block_data, sizeof(second_block_data));

  /* 障碍物坐标参数获取(Get obstacle coordinate parameters) */
  hw_cam.Linepatrol_Data_Receive(LINE_PATROL_THIRD_COLOR_REG1, roadblock_first_block_data, sizeof(roadblock_first_block_data));
  hw_cam.Linepatrol_Data_Receive(LINE_PATROL_THIRD_COLOR_REG2, roadblock_second_block_data, sizeof(roadblock_second_block_data));
  
  /* 计算x轴偏移均值(Calculate the average x-axis offset) */
  bias_average = (first_block_data[1] + second_block_data[1]) / 2;

  if(first_block_data[0] == tracking_color && second_block_data[0] == tracking_color && 
     roadblock_first_block_data[0] != roadblock_color && roadblock_second_block_data[0] != roadblock_color) 
  {
    set_speed = 50;
    Position_Control(&bias_average, &set_rot, &target_point);
    timerStart(ptimer, 60000, timerPIDCalculateCallBack, userdata);
    first_calibration_data = first_block_data[1];   /* 记录色块位置，丢线时做判断处理(Record the position of the color block for judgment when losing the line) */
    second_calibration_data = second_block_data[1];
  }
  else if(roadblock_first_block_data[0] == roadblock_color)   /* 若看到障碍物就停止巡线(Stop line following when encountering an obstacle) */
  {
    timerStop(ptimer);
    timerStart(&timer2, 100000, timerMovingRoadblockCallBack, "100ms cycle moving_roadblock");
  }
  /* 若偏离程度过大，开启丢线位置矫正定时器回调，关闭PID计算定时器回调(If the deviation is too large, start the timer callback for position calibration and stop the PID calculation timer callback) */
  else if(first_block_data[0] != tracking_color || second_block_data[0] != tracking_color)  
  {
    set_speed = 0;
    set_rot = 0;
    timerStop(ptimer);
    timerStart(&timer3, 60000, timerPositionCalibrationCallBack, "60ms cycle position calibration");
  }
}

/*清除障碍物定时器回调(Timer callback for clearing obstacles)*/
void timerMovingRoadblockCallBack(Timers *ptimer, const void *userdata)
{
   hw_cam.Linepatrol_Data_Receive(LINE_PATROL_FIRST_COLOR_REG1, first_block_data, sizeof(first_block_data));
  switch(time_count) 
  {
  case 0:
    set_angle = 0;
    set_speed = 30;
    set_rot = 0;   
    break;
  case 5:
    set_servo(0);
    set_speed = 0;
    set_rot = 60;
    break;
  case 10:
    set_angle = 0;
    set_speed = 0;
    set_rot = 0;
    break;
  case 17:
    set_angle = 0;
    set_speed = 50;
    set_rot = 0;
    break;
  case 20:
    set_servo(60);
    set_angle = 0;
    set_speed = 0;
    set_rot = 0;
    break;
  case 30:
    set_angle = 180;
    set_speed = 30;
    set_rot = -30;
    break;
  }
  if(time_count >= 30)
  {
    /* 识别到线条色块就继续巡线(Continue line following if the block is detected) */
    if(first_block_data[1] != 0)
    {   
      set_angle = 0;
      set_speed = 0;
      set_rot = 0;
      time_count = 0;
      timerStop(ptimer);
      timerStart(&timer1, 60000, timerPIDCalculateCallBack, "60ms cycle PID calculate");
    }
    else
    {
      time_count++;
      timerStart(ptimer, 100000, timerMovingRoadblockCallBack, userdata);  
    }
  }
  else
  {
    time_count++;
    timerStart(ptimer, 100000, timerMovingRoadblockCallBack, userdata);  
  }
}

/* 丢线处理定时器回调(Timer callback for losing the line) */
void timerPositionCalibrationCallBack(Timers *ptimer, const void *userdata)
{
  hw_cam.Linepatrol_Data_Receive(LINE_PATROL_FIRST_COLOR_REG1, first_block_data, sizeof(first_block_data));
  hw_cam.Linepatrol_Data_Receive(LINE_PATROL_FIRST_COLOR_REG2, second_block_data, sizeof(second_block_data));
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
    timerStart(ptimer, 60000, timerPositionCalibrationCallBack, userdata);

  }
}

void setup() {
  Serial.begin(115200);
  ticksFuncSet(ticksGetFunc);
  Motors_Initialize();
  hw_cam.Init();
  Calculation_Init();
  myServo.attach(servoPin);
  set_servo(60);
  timerStart(&timer1, 60000, timerPIDCalculateCallBack, "60ms cycle PID calculate");
}

void loop() {
  /* 定时器任务运行(Run the timer task) */
  timersTaskRunning();
  /* 软件PWM驱动电机进行姿态调整(Use software PWM to adjust the posture of the motor) */
  Velocity_Controller(set_angle, set_speed, set_rot, SIMULATE_PWM_CONTROL);
}


