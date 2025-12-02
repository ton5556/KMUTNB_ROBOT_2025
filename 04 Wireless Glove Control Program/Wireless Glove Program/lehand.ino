#include <SoftwareSerial.h> //软串口库(Software serial library)
#include "LobotServoController.h" //机器人控制信号库(Robot control signal library)
#include "MPU6050.h" //MPU6050库(MPU6050 library)
#include "Wire.h" //IIC库(I2C library)

// 蓝牙TX、RX引脚(Bluetooth TX and RX pins)
#define BTH_RX 11
#define BTH_TX 12

// 创建变位器最小值和最大值存储(Create minimum and maximum values for the potentiometers)
float min_list[5] = {0, 0, 0, 0, 0};
float max_list[5] = {255, 255, 255, 255, 255};
// 各个手指读取到的数据变量(Data variables read by each finger)
float sampling[5] = {0, 0, 0, 0, 0}; 
// 手指相关舵机变量(Variables for controlling the servos of each finger)
float data[5] = {1500, 1500, 1500, 1500, 1500};
uint16_t ServePwm[5] = {1500, 1500, 1500, 1500, 1500};
uint16_t ServoPwmSet[5] = {1500, 1500, 1500, 1500, 1500};
// 电位器校准标志位(Flag for potentiometer calibration)
bool turn_on = true;

// 蓝牙通讯串口初始化(Bluetooth communication serial port initialization)
SoftwareSerial Bth(BTH_RX, BTH_TX);
// 机器人控制对象(Robot control object)
LobotServoController lsc(Bth);

// 浮点数映射函数(Float mapping function)
float float_map(float in, float left_in, float right_in, float left_out, float right_out)
{
  return (in - left_in) * (right_out - left_out) / (right_in - left_in) + left_out;
}

// MPU6050相关变量(MPU6050 related variables)
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float ax0, ay0, az0;
float gx0, gy0, gz0;
float ax1, ay1, az1;
float gx1, gy1, gz1;

// 加速度校准变量(Acceleration calibration variables)
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //功能按键初始化(Function key initialization)
  pinMode(7, INPUT_PULLUP);
  //各手指电位器配置(Configuration of potentiometers for each finger)
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A6, INPUT);
  //LED 灯配置(LED light configuration)
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);

  //蓝牙配置(Bluetooth configuration)
  Bth.begin(9600);
  Bth.print("AT+ROLE=M");  //蓝牙配置为主模式(Configure Bluetooth as main mode)
  delay(100);
  Bth.print("AT+RESET");  //软重启蓝牙模块(Soft reset the Bluetooth module)
  delay(250);

  //MPU6050 配置(MPU6050 configuration)
  Wire.begin();
  Wire.setClock(20000);
  accelgyro.initialize();
  accelgyro.setFullScaleGyroRange(3); //设定角速度量程(Set the angular velocity range)
  accelgyro.setFullScaleAccelRange(1); //设定加速度量程(Set the acceletation range)
  delay(200);
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  //获取当前各轴数据以校准(Get current axis data for calibration)
  ax_offset = ax;  //X轴加速度校准数据(X-axis acceleration calibration data)
  ay_offset = ay;  //Y轴加速度校准数据(Y-axis acceleration calibration data)
  az_offset = az - 8192;  //Z轴加速度校准数据(Z-axis acceleration calibration data)
  gx_offset = gx; //X轴角速度校准数据(X-axis angular velocity calibration data)
  gy_offset = gy; //Y轴角速度校准数据(Y-axis angular velocity calibration data)
  gz_offset = gz; //Z轴角速度校准数据(Z-axis angular velocity calibration data)
}

//获取各个手指电位器数据(Get data from each finger's potentiometer)
void finger() {
  static uint32_t timer_sampling;
  static uint32_t timer_init;
  static uint8_t init_step = 0;
  if (timer_sampling <= millis())
  {
    for (int i = 14; i <= 18; i++)
    {
      if (i < 18)
        sampling[i - 14] += analogRead(i); //读取各个手指的数据(Read data from each finger)
      else
        sampling[i - 14] += analogRead(A6);  //读取小拇指的数据， 因为IIC 用了 A4,A5 口，所以不能从A0 开始连续读取(Read data from the little finger, because I2C uses A4 and A5 pins. It cannot read continuously from A0)
      sampling[i - 14] = sampling[i - 14] / 2.0; //取上次和本次测量值的均值(Take the average of the previous and current measurements)
      data[i - 14 ] = float_map( sampling[i - 14],min_list[i - 14], max_list[i - 14], 2500, 500); //将测量值映射到500-2500， 握紧手时为500， 张开时为2500(Map the measurement value to 500 to 2500, with 500 for gripping and 2500 for opening)
      data[i - 14] = data[i - 14] > 2500 ? 2500 : data[i - 14];  // 限制最大值为2500(Limit the maximum value to 2500)
      data[i - 14] = data[i - 14] < 500 ? 500 : data[ i - 14];   //限制最小值为500(Limit the minimum value to 500)
    }
    //timer_sampling = millis() + 10;
  }

  if (turn_on && timer_init < millis())
  {
    switch (init_step)
    {
      case 0:
        digitalWrite(2, LOW);
        digitalWrite(3, LOW);
        digitalWrite(4, LOW);
        digitalWrite(5, LOW);
        digitalWrite(6, LOW);
        timer_init = millis() + 20;
        init_step++;
        break;
      case 1:
        digitalWrite(2, HIGH);
        digitalWrite(3, HIGH);
        digitalWrite(4, HIGH);
        digitalWrite(5, HIGH);
        digitalWrite(6, HIGH);
        timer_init = millis() + 200;
        init_step++;
        break;
      case 2:
        digitalWrite(2, LOW);
        digitalWrite(3, LOW);
        digitalWrite(4, LOW);
        digitalWrite(5, LOW);
        digitalWrite(6, LOW);
        timer_init = millis() + 50;
        init_step++;
        break;
      case 3:
        digitalWrite(2, HIGH);
        digitalWrite(3, HIGH);
        digitalWrite(4, HIGH);
        digitalWrite(5, HIGH);
        digitalWrite(6, HIGH);
        timer_init = millis() + 500;
        init_step++;
        Serial.print("max_list:");
        for (int i = 14; i <= 18; i++)
        {
          max_list[i - 14] = sampling[i - 14];
          Serial.print(max_list[i - 14]);
          Serial.print("-");
        }
        Serial.println();
        break;
      case 4:
        init_step++;
        break;
      case 5:
        if ((max_list[1] - sampling[1]) > 50)
        {
          init_step++;
          digitalWrite(2, LOW);
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, LOW);
          timer_init = millis() + 2000;
        }
        break;
      case 6:
        digitalWrite(2, HIGH);
        digitalWrite(3, HIGH);
        digitalWrite(4, HIGH);
        digitalWrite(5, HIGH);
        digitalWrite(6, HIGH);
        timer_init = millis() + 200;
        init_step++;
        break;
      case 7:
        digitalWrite(2, LOW);
        digitalWrite(3, LOW);
        digitalWrite(4, LOW);
        digitalWrite(5, LOW);
        digitalWrite(6, LOW);
        timer_init = millis() + 50;
        init_step++;
        break;
      case 8:
        digitalWrite(2, HIGH);
        digitalWrite(3, HIGH);
        digitalWrite(4, HIGH);
        digitalWrite(5, HIGH);
        digitalWrite(6, HIGH);
        timer_init = millis() + 500;
        init_step++;
        Serial.print("min_list:");
        for (int i = 14; i <= 18; i++)
        {
          min_list[i - 14] = sampling[i - 14];
          Serial.print(min_list[i - 14]);
          Serial.print("-");
        }
        Serial.println();
        lsc.runActionGroup(0, 1);
        turn_on = false;
        break;

      default:
        break;
    }
  }
}


float radianX;
float radianY;
float radianZ;
float radianX_last; //最终获得的X轴倾角(Final X-axis inclination angle obtained)
float radianY_last; //最终获得的Y轴倾角(Final Y-axis inclination angle obtained)


//更新倾角传感器数据(Update data from the inclination sensor)
void update_mpu6050()
{
  static uint32_t timer_u;
  if (timer_u < millis())
  {
    // put your main code here, to run repeatedly:
    timer_u = millis() + 20;
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    ax0 = ((float)(ax)) * 0.3 + ax0 * 0.7;  //对读取到的值进行滤波(Filter the read values)
    ay0 = ((float)(ay)) * 0.3 + ay0 * 0.7;
    az0 = ((float)(az)) * 0.3 + az0 * 0.7;
    ax1 = (ax0 - ax_offset) /  8192.0;  // 校正，并转为重力加速度的倍数(Correct and convert to multiples of gravity acceleration)
    ay1 = (ay0 - ay_offset) /  8192.0;
    az1 = (az0 - az_offset) /  8192.0;

    gx0 = ((float)(gx)) * 0.3 + gx0 * 0.7;  //对读取到的角速度的值进行滤波(Filter the read angular velocity values)
    gy0 = ((float)(gy)) * 0.3 + gy0 * 0.7;
    gz0 = ((float)(gz)) * 0.3 + gz0 * 0.7;
    gx1 = (gx0 - gx_offset);  //校正角速度(Correct the angular velocity)
    gy1 = (gy0 - gy_offset);
    gz1 = (gz0 - gz_offset);


    //互补计算x轴倾角(Complementary calculation of X-axis inclination angle)
    radianX = atan2(ay1, az1);
    radianX = radianX * 180.0 / 3.1415926;
    float radian_temp = (float)(gx1) / 16.4 * 0.02;
    radianX_last = 0.8 * (radianX_last + radian_temp) + (-radianX) * 0.2;

    //互补计算y轴倾角(Complementary calculation of Y-axis inclination angle)
    radianY = atan2(ax1, az1);
    radianY = radianY * 180.0 / 3.1415926;
    radian_temp = (float)(gy1) / 16.4 * 0.01;
    radianY_last = 0.8 * (radianY_last + radian_temp) + (-radianY) * 0.2;
  }
}

//打印数据(Print data)
void print_data()
{
  static uint32_t timer_p;
  static uint32_t timer_printlog;
  if ( timer_p < millis())
  {
    Serial.print("ax:"); Serial.print(ax1);
    Serial.print(", ay:"); Serial.print(ay1);
    Serial.print(", az:"); Serial.print(az1);
    Serial.print(", gx:"); Serial.print(gx1);
    Serial.print(", gy:"); Serial.print(gy1);
    Serial.print(", gz:"); Serial.print(gz1);
    Serial.print(", GX:"); Serial.print(radianX_last);
    Serial.print(", GY:"); Serial.println(radianY_last);
    timer_p = millis() + 300;
  }
/**
  if (timer_printlog <= millis())  //要输出数据 将 0 && 去掉(The data to be output, clearing 0)
  {
    for (int i = 14; i <= 18; i++)
    {
      Serial.print(data[i - 14]);
      Serial.print("  ");
      // Serial.print(float_map(min_list[i-14], max_list[i-14], 500,2500,sampling[i-14]));
      Serial.print(" ");
      // Serial.print();
    }
    timer_printlog = millis() + 1000;
    Serial.println();
  }
**/
}

#define STOP       0
#define GO_FORWARD 1
#define GO_BACK    2
#define TURN_LEFT  3
#define TURN_RIGHT 4

//run,控制六足(run() controls the hexapod robot)
void run()
{
  static uint32_t timer;
  static uint32_t step;
  static int act;
  static int last_act;
  static uint8_t count = 0;
  if (timer > millis())
    return;
  timer = millis() + 80;
  if (radianY_last < -35 && radianY_last > -90 && data[3] < 1200  && data[2] > 2000) // 手掌右倾角度大于35度且小于90度， 中指伸出无名指弯曲(Right tilt angle of the palm is greater than 35 degrees and less than 90 degrees, with middle finger is extended and ring finger is bent)
  {
    act = TURN_RIGHT; //右转(Turn right)
  }
  if (radianY_last < 90 && radianY_last > 35 && data[3] < 1200 && data[2] > 2000)    // 手掌左倾角度大于35度且小于90度， 中指伸出无名指弯曲(Left tilt angle of the palm is greater than 35 degrees and less than 90 degrees, with middle finger is extended and ring finger is bent)
  {
    act = TURN_LEFT; //左转(Turn left)
  }
  if ((radianY_last < 15 && radianY_last > -15) && data[2] < 600)  //手心朝下，握拳（中指弯曲），停止(If you make a fist with the palm facing dowm and the middle finger bending, the robot stops)
  {
    act = STOP;
  }
  if ((radianY_last < 15 &&  radianY_last > -15 ) && data[2] > 2100 && data[3] > 2100)  //手心朝下，张开手（中指伸直），前进(If you open your hand with the palm facing dowm and the middle finger extending, the robot moves forward)
  {
    act = GO_FORWARD;
  }
  if ((radianY_last < -130 ||  radianY_last > 130 ) && data[2] < 1200 && data[4] > 2000)  //手心朝上， 中指弯曲，小拇指伸直（蜘蛛侠动作）， 后退(If you make a Spiderman gesture with the palm facing up, the robot moves backward)
  {
    act = GO_BACK;
  }
  if ((radianY_last < -130 ||  radianY_last > 130 ) && data[2] > 2000) //手心朝上，张开手，停止(If you open your hand with the palm facing up, the robot stops)
  {
    act = STOP;
  }
  if (act != last_act)
  {
    last_act = act;
    if (act == STOP)
    {
    if (count != 1) {
      count = 1;
      lsc.stopActionGroup();  //停止当前动作组(Stop the current action group)
      lsc.runActionGroup(0, 1);  //运行指定动作组(Run the specified action group)
      return;
    }
    }
    if (act == GO_FORWARD)
    {
    if (count != 2) {
      count = 2;
      lsc.stopActionGroup();
      lsc.runActionGroup(1, 0);
      return;
    }
    }
    if (act == GO_BACK)
    {
    if (count != 3) {
      count = 3;
      lsc.stopActionGroup();
      lsc.runActionGroup(2, 0);
      return;
    }
    }
    if (act == TURN_LEFT)
    {
    if (count != 4) {
      count = 4;
      lsc.stopActionGroup();
      lsc.runActionGroup(3, 0);
      return;
    }
    }
    if (act == TURN_RIGHT)
    {
    if (count != 5) {
      count = 5;
      lsc.stopActionGroup();
      lsc.runActionGroup(4, 0);
      return;
    }
    }
  }
}

//run1, 控制手掌。(run1 controls the hand)
void run1(int mode)
{
  static float RadianY_Las = 0;
  // 为每个手指的舵机赋值(Assign values to each finger's servo)
  for (int i = 0; i < 5; i++)
  {
    ServoPwmSet[i] = data[i]; 
    ServoPwmSet[i] = float_map(ServoPwmSet[i], 500, 2500, 1100, 1950);
  }
  int pos = 0;
  if(mode == 4) //当为模式4时，为左手掌(When it is mode 4, control the left hand)
    pos = ServoPwmSet[4];
  else //为模式1，右手掌(When it is mode 1, control the right hand)
    pos = 2750 - ServoPwmSet[4];
  // 若有Y轴的旋转，则添加云台舵机的位置(If there is Y-axis rotation, add the position of the pan-tilt servo)
  if (radianY_last < 90 && radianY_last > -90)
  {
    if ( abs(radianY_last - RadianY_Las) > 1)  {
      uint16_t se = 1500 + radianY_last*10;
      lsc.moveServos(6, 30, 1, 3050 - ServoPwmSet[0], 2, ServoPwmSet[1], 3, ServoPwmSet[2], 4, ServoPwmSet[3], 5, pos , 6 , se);//控制每个手指(control every finger)
      return;
    }
  }
  // 若无Y轴选择，则只发五个手指的舵机位置(If there is no Y-axis rotation, only send the servo positions of the five fingers)
  lsc.moveServos(5, 30, 1, 3050 - ServoPwmSet[0], 2, ServoPwmSet[1], 3, ServoPwmSet[2], 4, ServoPwmSet[3], 5, pos);//控制每个手指(control every finger)
}

//具体向小车发送数据(Specific data sent to the car)
void car_control(byte motor1, byte motor2)
{
  byte buf[6];
  buf[0] = buf[1] = 0x55;
  buf[2] = 0x04;
  buf[3] = 0x32;
  buf[4] = (byte)motor1;
  buf[5] = (byte)motor2;
  Bth.write(buf, 6);
}

//run2,控制小车(run2 controls the car)
void run2()
{
  static uint32_t timer;
  static uint32_t step;
  static uint8_t count = 0;
  int act = 0;
  static int last_act;
  if (timer > millis())
    return;
  timer = millis() + 100;
  if (data[2] < 600 && (radianY_last < -30 && radianY_last > -90))
  {
    car_control(100, -100);
  }
  else if (data[2] < 600  && (radianY_last > 30 && radianY_last < 90))
  {
   car_control(-100, 100); 
  }
  else if (data[2] < 600 && abs(radianY_last) < 30 )
  {
    car_control(100, 100);
  }
  else if (data[2] < 600 && (radianY_last < -130 ||  radianY_last > 130 ))
  {
   car_control(-100, -100); 
  }
  else
    car_control(0, 0); 
}

//run3, 操控机械臂(run3 controls the robotic arm)
void run3()
{
  static uint32_t timer;
  static uint32_t step;
  int act = 0;
  static int last_act;
  static uint8_t mode = 0;
  static uint8_t mode1 = 0;
  static uint8_t count = 0;
  if (timer > millis())
    return;
  timer = millis() + 50;

  static float RadianY_Las = 0;
  if (data[1] < 1200 && data[2] < 1000 && data[3] < 1000)  //握拳旋转控制6号舵机转动(Make a fist and tilt to control servo 6)
  {
      if (radianY_last < 90 && radianY_last > -90)
      {
          lsc.moveServo(6, 1500 + radianY_last*10, 50);
          delay(50);
      }
  } 
  else if ( data[0] > 1400 && data[1] > 1400 && data[2] > 1400 && data[3] > 1400) //五指张开旋转控制5号舵机转动(Open your hand to control servo 5)
  {
      if (radianY_last < 90 && radianY_last > -90)
      {
          lsc.moveServo(5, 1500 + radianY_last*10, 50);
          delay(50);
      }
  }
  else if (data[1] > 1400 && data[2] < 1000 && data[3] < 1000 ) //食指伸直旋转控制1号舵机(Extend your index finger and tilt to control servo 1)
  {
      if (radianY_last < 90 && radianY_last > -90)
      {
          lsc.moveServo(1, 1500 + radianY_last*10, 50);
          delay(50);
      }
  }
  else if (data[1] > 1400 && data[2] > 1400 && data[3] < 1000 ) //食指和中指伸直控制2号舵机(Extend your index finger and middle finger to control servo 2)
  {
      if (radianY_last < 90 && radianY_last > -90)
      {
          lsc.moveServo(2, 1500 + radianY_last*10, 50);
          delay(50);
      }
  }
  else if (data[1] < 1400 && data[2] > 1200 && data[3] > 1000 ) //中指无名指小指伸直控制3号舵机(Extend your middle, ring, and little fingers to control servo 3)
  {
      if (radianY_last < 90 && radianY_last > -90)
      {
          lsc.moveServo(3, 1500 + radianY_last*10, 50);
          delay(50);
      }
  }
  else if (data[0] < 1400 && data[1] > 1400 && data[2] > 1400 && data[3] > 1400) //食指中指无名指小指伸直控制4号舵机(Open your hand with your thumb bending to control servo 4)
  {
      if (radianY_last < 90 && radianY_last > -90)
      {
          lsc.moveServo(4, 1500 + radianY_last*10, 50);
          delay(50);
      }
  }
}

int mode = 0;
bool key_state = false;
void loop() {
  finger();  //更新手指电位器数据(Update finger potentiometers data)
  update_mpu6050();  //更新倾角传感器数据(Update inclination sensor data)

  if (turn_on == false) //启动后电位器校正完毕(After startup, the potentiometer calibration is completed)
  {
    // 若K3按键按下(If K3 button is pressed)
    if(key_state == true && digitalRead(7) == true)
    {
      delay(30);
      if(digitalRead(7) == true)
        key_state = false;
    }
    if (digitalRead(7) == false && key_state == false)
    {
      delay(30);
      // 若K3被按下,则更换模式,并显示对应的LED灯数(If K3 is pressed, change the mode and display the corresponding LED number)
      if (digitalRead(7) == false)
      {
        key_state = true;
        if (mode == 5)
        {
          mode = 0;
        }
        else
          mode++;
        if (mode == 0)
        {
          digitalWrite(2, HIGH);
          digitalWrite(3, HIGH);
          digitalWrite(4, HIGH);
          digitalWrite(5, HIGH);
          digitalWrite(6, HIGH);
        }
        if (mode == 1)
        {
          digitalWrite(2, LOW);
          digitalWrite(3, HIGH);
          digitalWrite(4, HIGH);
          digitalWrite(5, HIGH);
          digitalWrite(6, HIGH);
        }
        if (mode == 2)
        {
          digitalWrite(2, LOW);
          digitalWrite(3, LOW);
          digitalWrite(4, HIGH);
          digitalWrite(5, HIGH);
          digitalWrite(6, HIGH);
        }
        if (mode == 3)
        {
          digitalWrite(2, LOW);
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, HIGH);
          digitalWrite(6, HIGH);
        }

        if (mode == 4)
        {
          digitalWrite(2, LOW);
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, HIGH);
        }
      }
    }

    if (mode == 0)
      run();  // 蜘蛛(Spider)
    if (mode == 1 || mode == 4)  { //模式1为左手掌,模式2为右手掌(Mode 1 is the left hand, mode 2 is the right hand)
      run1(mode); // 手掌(Hand)
    }
    if (mode == 2)
      run2(); //小车(Car)
    if (mode == 3)
      run3();  //机械臂(Robotic arm)
  }
  // print_data();  //打印传感器数据便于调试(Print sensor data for debugging)
}
