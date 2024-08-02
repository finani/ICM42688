#include <Wire.h>
#include "ICM42688.h"

#define I2C_SDA 8
#define I2C_SCL 9

// create TwoWire instance
TwoWire I2C_0 = TwoWire(0);

ICM42688 IMU;

void setup() {
  I2C_0.begin(I2C_SDA, I2C_SCL);
  // serial to display data
  Serial.begin(115200);
  while(!Serial) {}

  // start communication with IMU
  int status = IMU.begin_I2C(0x68, I2C_0);
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
  Serial.println("ax,ay,az,gx,gy,gz,temp_C");
}

void loop() {
  // read the sensor
  IMU.getAGT();
  // display the data
  Serial.print(IMU.accX(),6);
  Serial.print("\t");
  Serial.print(IMU.accY(),6);
  Serial.print("\t");
  Serial.print(IMU.accZ(),6);
  Serial.print("\t");
  Serial.print(IMU.gyrX(),6);
  Serial.print("\t");
  Serial.print(IMU.gyrY(),6);
  Serial.print("\t");
  Serial.print(IMU.gyrZ(),6);
  Serial.print("\t");
  Serial.println(IMU.temp(),6);
  delay(100);
}
