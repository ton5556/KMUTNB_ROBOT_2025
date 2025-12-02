#include <Arduino.h>
#include "FastLED.h"
#include "hw_esp32s3cam_ctl.h"

const static uint8_t ledPin = 2;
const static uint8_t buzzerPin = 3;
static uint16_t g_current_point = 0;
static uint16_t color_info[4] = { 0, 0, 0, 0};
static CRGB rgbs[1];
HW_ESP32S3CAM hw_cam;                              ///< 实例化ESP32-Cam摄像头类(Instantiate the ESP32-S3 camera class)

static uint8_t cam_buffer[COLOR_DETECTION_REG_COUNT];
void Rgb_Show(uint8_t rValue,uint8_t gValue,uint8_t bValue);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.setTimeout(500);                                         ///< 设置串行端口读取数据的超时时间(Set the timeout for reading data from the serial port)
  FastLED.addLeds<WS2812, ledPin, RGB>(rgbs, 1);
  Rgb_Show(255,255,255);
  hw_cam.Init(); ///<初始化与ESP32Cam通讯接口(Instantiate the interface that communicates with the ESP32-S3)
  pinMode(buzzerPin,OUTPUT);
  tone(buzzerPin, 1200);                                          ///< 输出音调信号的函数,频率为1200(Function for outputting tone signals with a frequency of 1200)
  delay(100);
  noTone(buzzerPin);
}

void loop() {
  // put your main code here, to run repeatedly:
  hw_cam.Colordetection_Data_Receive(ALL_COLOR_INDEX_REG, cam_buffer, sizeof(cam_buffer));
  delay(10);
  if(cam_buffer[0] == 0) Rgb_Show(0,10,0);    ///< red
  else if(cam_buffer[3] == 3) Rgb_Show(0,0,10); ///< blue
  else if(cam_buffer[2] == 2) Rgb_Show(10,0,0); ///< green

}

 /**
 * @brief 设置RGB灯的颜色(Set the color of the RGB LED)
 * @param rValue 取值范围[0,255](Value Range: [0,255])
 * @param gValue 取值范围[0,255](Value Range: [0,255])
 * @param bValue 取值范围[0,255](Value Range: [0,255])
 * @retval None
 * @note (255,0,0)绿色 (0,255,0)红色 (0,0,255)蓝色 (255,255,255)白色((255,0,0) green; (0,255,0) red; (0,0,255) blue; (255,255,255) white)
 */
void Rgb_Show(uint8_t rValue,uint8_t gValue,uint8_t bValue) {
  rgbs[0].r = rValue;
  rgbs[0].g = gValue;
  rgbs[0].b = bValue;
  FastLED.show();
}