
#include "FastLED.h"
#include "Ultrasound.h"
#include "hw_esp32s3cam_ctl.h"

const static uint8_t ledPin = 2;
const static uint8_t buzzerPin = 3;
const static uint8_t motorpwmPin[4] = { 10, 9, 6, 11} ;
const static uint8_t motordirectionPin[4] = { 12, 8, 7, 13};

const static uint8_t pwm_min = 50;
static CRGB rgbs[1];
uint32_t previousTime = 0;
bool faceDetected = false;
bool detectionValue = false;

static uint8_t cam_buffer[FACE_DETECTION_REG_COUNT];

HW_ESP32S3CAM hw_cam;                              ///< 实例化ESP32-Cam摄像头类(Instantiate the ESP32-S3 camera class)
Ultrasound ultrasound;                           ///< 实例化超声波类(Instantiate the ultrasonic class)

void Motor_Init(void);
void Facedetect_Task(void);
bool Check_Delay(uint32_t interval);
void Rgb_Show(uint8_t rValue,uint8_t gValue,uint8_t bValue);
void Velocity_Controller(uint16_t angle, uint8_t velocity,int8_t rot);
void Motors_Set(int8_t Motor_0, int8_t Motor_1, int8_t Motor_2, int8_t Motor_3);

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(500);                        ///< 设置串行端口读取数据的超时时间(Set the timeout for reading data from the serial port)
  FastLED.addLeds<WS2812, ledPin, RGB>(rgbs, 1);
  Rgb_Show(10,10,10);
  ultrasound.Color( 10, 10, 10, 10, 10, 10);
  hw_cam.Init();                                ///< 初始化与ESP32Cam通讯接口(Initialize the communication interface with the ESP32-S3)
  Motor_Init();
  pinMode(buzzerPin,OUTPUT);
  tone(buzzerPin, 1200);                         ///< 输出音调信号的函数,频率为1200(Output a tone signal with a frequency of 1200)
  delay(100);
  noTone(buzzerPin);
}

void loop() {
  if (Check_Delay(10)) Facedetect_Task();
}

 /* 人脸识别函数(Face recognization function) */
void Facedetect_Task(void)
{

  hw_cam.Facedetection_Data_Receive(FACE_DETECTION_CENTER_REG, cam_buffer, sizeof(cam_buffer));
  if (cam_buffer[0] != 0 || cam_buffer[1] != 0)
  {
    faceDetected = true;
  }
  else
  {
    faceDetected = false;
  }
  if (faceDetected == true)
  {
    Rgb_Show(10,0,0);  
    ultrasound.Color( 0, 10, 0, 0, 10, 0);
    Velocity_Controller(0, 0, 50); 
    delay(500);
    Velocity_Controller( 0, 0, -50);   
    delay(500); 
    Velocity_Controller( 0, 0, 0);
    faceDetected = false;                       ///< 设置标志为true，表示已经检测到人脸(Set the flag to true, indicating that a face has been detected)
  }
  else
  {
    Rgb_Show(10,10,10);  
    ultrasound.Color( 10, 10, 10, 10, 10, 10);
  }
  Serial.print(cam_buffer[0]);Serial.print(",");Serial.println(cam_buffer[1]);
  // if (!detectionValue) {
  //   if (faceDetected) {                          ///< 如果之前检测到过人脸，则停止动作(If a face was detected previously, stop the action)
  //     Rgb_Show(10,10,10);  
  //     ultrasound.Color( 10, 10, 10, 10, 10, 10);
  //     faceDetected = false;                      ///< 设置标志为false，表示未检测到人脸(Set the flag to false, indicating that no face has been detected)
  //   }
  // } 
  // else {
  //   if (!faceDetected) {                         ///< 如果之前没有检测到人脸，则执行动作(If no face was detected previously, perform the action)
  //     Rgb_Show(10,0,0);  
  //     ultrasound.Color( 0, 10, 0, 0, 10, 0);
  //     Velocity_Controller(0, 0, 50); 
  //     delay(200);
  //     Velocity_Controller( 0, 0, -50);   
  //     delay(200); 
  //     Velocity_Controller( 0, 0, 0);
  //     faceDetected = true;                       ///< 设置标志为true，表示已经检测到人脸(Set the flag to true, indicating that a face has been detected)
  //   }
  // }
}

 /**
 * @brief 设置RGB灯的颜色(Set the color of the RGB LED)
 * @param rValue 取值范围[0,255](Value Range [0,255])
 * @param gValue 取值范围[0,255](Value Range [0,255])
 * @param bValue 取值范围[0,255](Value Range [0,255])
 * @retval None
 * @note (255,0,0)绿色 (0,255,0)红色 (0,0,255)蓝色 (255,255,255)白色((255,0,0) green; (0,255,0) red; (0,0,255) blue; (255,255,255) white)
 */
void Rgb_Show(uint8_t rValue,uint8_t gValue,uint8_t bValue) {
  rgbs[0].r = rValue;
  rgbs[0].g = gValue;
  rgbs[0].b = bValue;
  FastLED.show();
}

 /**
 * @brief 非阻塞延时函数(Non-blocking delay function)
 * @param interval 用于设置需要延时的时间(Used to set the time to be delayed)
 * @retval None
 * @note 
 */
bool Check_Delay(uint32_t interval) {
  uint32_t currentTime = millis();
  if (currentTime - previousTime >= interval) {
    previousTime = currentTime;
    return true;
  }
  return false;
}

 /* 电机初始化函数(Motor initialization function) */
void Motor_Init(void){
  for(uint8_t i = 0; i < 4; i++){
    pinMode(motordirectionPin[i], OUTPUT);
  }
  Velocity_Controller( 0, 0, 0);
}

/**
 * @brief 速度控制函数(Velocity control function)
 * @param angle   用于控制小车的运动方向，小车以车头为0度方向，逆时针为正方向。(Used to control the direction of the car's movement. The car's head is the 0-degree direction, and counterclockwise is the positive direction)
 *                取值范围[0,269](Range: [0,269])
 * @param velocity   用于控制小车速度，取值为0~100。(Used to control the speed of the car, with a range of 0 to 100)
 * @param rot     用于控制小车的自转速度，取值为-100~100，若大于0小车有一个逆
 *                 时针的自转速度，若小于0则有一个顺时针的自转速度。(Used to control the car's rotational speed, with a range of -100 to 100. If it is greater than 0, the car has a counterclockwise rotational speed; if it is less than 0, it has a clockwise rotational speed)
 * @retval None
 */
void Velocity_Controller(uint16_t angle, uint8_t velocity,int8_t rot) {
  int8_t velocity_0, velocity_1, velocity_2, velocity_3;
  float speed = 1;                               ///< 速度因子(Speed factor)
  angle += 90;                                   ///< 设定初始方向(Set initial direction)
  float rad = angle * PI / 180;
  if (rot == 0) speed = 1;
  else speed = 0.5; 
  velocity /= sqrt(2);
  velocity_0 = (velocity * sin(rad) - velocity * cos(rad)) * speed + rot * speed;
  velocity_1 = (velocity * sin(rad) + velocity * cos(rad)) * speed - rot * speed;
  velocity_2 = (velocity * sin(rad) - velocity * cos(rad)) * speed - rot * speed;
  velocity_3 = (velocity * sin(rad) + velocity * cos(rad)) * speed + rot * speed;
  Motors_Set(velocity_0, velocity_1, velocity_2, velocity_3);
}
 
/**
 * @brief PWM与轮子转向设置函数(PWM and wheel steering setting function)
 * @param Motor_x   作为PWM与电机转向的控制数值。根据麦克纳姆轮的运动学分析求得。(Used as the control value of PWM and motor steering. Calculated based on the kinematic analysis of Mecanum wheels)
 * @retval None
 */
void Motors_Set(int8_t Motor_0, int8_t Motor_1, int8_t Motor_2, int8_t Motor_3) {
  int8_t pwm_set[4];
  int8_t motors[4] = { Motor_0, Motor_1, Motor_2, Motor_3};
  bool direction[4] = { 1, 0, 0, 1};///< 前进 左1 右0(Forward; left 1; right 0)
  for(uint8_t i; i < 4; ++i) {
    if(motors[i] < 0) direction[i] = !direction[i];
    else direction[i] = direction[i];

    if(motors[i] == 0) pwm_set[i] = 0;
    else pwm_set[i] = map(abs(motors[i]), 0, 100, pwm_min, 255);

    digitalWrite(motordirectionPin[i], direction[i]); 
    analogWrite(motorpwmPin[i], pwm_set[i]); 
  }
}


