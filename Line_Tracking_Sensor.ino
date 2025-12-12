/********四路巡线传感器测试程序*******
 * Arduino型号：Arduino UNO
 **************************/
#include <Wire.h>

#define LINE_FOLLOWER_I2C_ADDR    0x78  // iic地址

u8 data;
bool WireWriteByte(uint8_t val)
{
    Wire.beginTransmission(LINE_FOLLOWER_I2C_ADDR);
    Wire.write(val);
    if( Wire.endTransmission() != 0 ) {
        Serial.println("false");
        return false;
    }
    Serial.println("true");
    return true;
}

bool WireReadDataByte(uint8_t reg, uint8_t &val)
{
    if (!WireWriteByte(reg)) {
        return false;
    }   
    Wire.requestFrom(LINE_FOLLOWER_I2C_ADDR, 1);
    while (Wire.available()) {
        val = Wire.read();
    }   
    return true;
}
int WireReadDataArray(uint8_t reg, uint8_t *val, unsigned int len)
{
    unsigned char i = 0;
    
    /* Indicate which register we want to read from */
    if (!WireWriteByte(reg)) {
        return -1;
    }
    
    /* Read block data */
    Wire.requestFrom(LINE_FOLLOWER_I2C_ADDR, len);
    while (Wire.available()) {
        if (i >= len) {
            return -1;
        }
        val[i] = Wire.read();
        i++;
    }
    
    return i;
}

void setup()
{
  Wire.begin();
  Serial.begin(9600);
}

void loop()
{
  WireReadDataByte(1,data);  //获取传感器数值
  Serial.print("Sensor1: ");Serial.print(data & 0x01);
  Serial.print("      Sensor2: ");Serial.print((data>>1) & 0x01);
  Serial.print("      Sensor3: ");Serial.print((data>>2) & 0x01);
  Serial.print("      Sensor4: ");Serial.println((data>>3) & 0x01);
  delay(500);
}
