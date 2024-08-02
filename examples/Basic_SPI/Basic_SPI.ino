#include "ICM42688.h"

#define SPI_SCK 8
#define SPI_MOSI 9
#define SPI_MISO 10
#define SPI_CS 11

ICM42688 IMU;

void setup() {
  // serial to display data
  Serial.begin(115200);
  while(!Serial) {}

  // start communication with IMU
  //int status = IMU.begin_SPI(SPI_CS);
  int status = IMU.begin_SPI(SPI_CS, SPI_SCK, SPI_MISO, SPI_MOSI);
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
