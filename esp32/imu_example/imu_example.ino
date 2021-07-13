
//#include <WSWire.h>
#include <Wire.h>

// internal device registers
const uint8_t IMU_DREG_ADDR_X0 = 0x43; 
const uint8_t IMU_PREG = 0x6B;

// I2C address
uint8_t IMU_I2C_ADDR = 0x68;
// I2C read or write operation bit
const uint8_t I2C_SLV2_RW = 8;

const uint32_t BAUD_RATE = 115200;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUD_RATE);
  Wire.begin();
  delay(100);
  // Begin communication with MPU6050
  Wire.beginTransmission(IMU_I2C_ADDR);
  // set power 
  Wire.write(IMU_PREG);
  Wire.write((uint8_t) 0);
//  Wire.write(I2C_SLV2_RW);
  Wire.endTransmission();
  Serial.println("Starting IMU");
}

void loop() {
//    Serial.print("hi loop");
//  // put your main code here, to run repeatedly:
  Wire.beginTransmission(IMU_I2C_ADDR);
  // request for data
  Wire.write(IMU_DREG_ADDR_X0);
//  Wire.write(IMU_DREG_ADDR_X1);
  Wire.endTransmission();

  // get requested data
  Wire.requestFrom(IMU_I2C_ADDR, 2);

  int x0, x1, xout;
  if (Wire.available() <= 2) {
    x0 = Wire.read();
    x1 = Wire.read();
    Serial.print("x0=");
    Serial.print(x0);
    Serial.print(", x1=");
    Serial.print(x1);
    Serial.print("\n");
  }

}
