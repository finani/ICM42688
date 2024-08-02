#include "ICM42688.h"

#define SPI_SCK 8
#define SPI_MOSI 9
#define SPI_MISO 10
#define SPI_CS 11

ICM42688 IMU;

volatile bool dataReady = false;

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

  // attaching the interrupt to microcontroller pin 1
  pinMode(22, INPUT);
  attachInterrupt(22, setImuFlag, RISING);

  // set output data rate to 12.5 Hz
  imu.setAccelODR(ICM42688::odr12_5);
  imu.setGyroODR(ICM42688::odr12_5);

  // enabling the data ready interrupt
  imu.enableDataReadyInterrupt(); 

  Serial.println("ax,ay,az,gx,gy,gz,temp_C");
}

void loop() {
  if (!dataReady) return;

  dataReady = false;

  // read the sensor
  imu.getAGT();
  
  // display the data
  Serial.print(imu.accX(),6);
  Serial.print("\t");
  Serial.print(imu.accY(),6);
  Serial.print("\t");
  Serial.print(imu.accZ(),6);
  Serial.print("\t");
  Serial.print(imu.gyrX(),6);
  Serial.print("\t");
  Serial.print(imu.gyrY(),6);
  Serial.print("\t");
  Serial.print(imu.gyrZ(),6);
  Serial.print("\t");
  Serial.println(imu.temp(),6);
}

void setImuFlag() {
  dataReady = true;
}