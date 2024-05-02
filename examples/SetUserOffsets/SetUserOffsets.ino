#include <Arduino.h>
#include "ICM42688.h"


ICM42688 imu(SPI, 5);
volatile bool dataReady = false;


// prototypes:
void setImuFlag();


void setup() {
  Serial.begin(115200);
  while(!Serial) {}
  Serial.println("program starts...");

  int status = imu.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }

  // attaching the interrupt to microcontroller pin 
  pinMode(22, INPUT);
  attachInterrupt(22, setImuFlag, RISING);
  // configure ICM42688
    // set output data rate to 200 Hz
    imu.setAccelODR(ICM42688::odr50);
    imu.setGyroODR(ICM42688::odr50);
    // set Scale
    imu.setAccelFS(ICM42688::AccelFS::gpm16);
    imu.setGyroFS(ICM42688::GyroFS::dps2000);
    //disable filters
    imu.setFilters(false, false);
    // enabling the data ready interrupt
    imu.enableDataReadyInterrupt(); 

  
  // Place the IMU in its calibration position, do not induce any motion/vibration
  imu.computeOffsets();  //note, Full scale used are dps2000 for gyro and gm2 for accelerometers. user full scale are restored after offsets
  imu.setAllOffsets();
  Serial.println("raw Bias:");
  Serial.print(imu.rawBiasAccX());Serial.print("\t");
  Serial.print(imu.rawBiasAccY());Serial.print("\t");
  Serial.print(imu.rawBiasAccZ());Serial.print("\t");
  Serial.print(imu.rawBiasGyrX());Serial.print("\t");
  Serial.print(imu.rawBiasGyrY());Serial.print("\t");
  Serial.print(imu.rawBiasGyrZ());Serial.print("\t");
  Serial.println("");
  delay(1000);
  Serial.println("End of Setup");

}

void loop() {
  if (!dataReady) return;
    dataReady = false;
    // read the sensor
    imu.getRawAGT();
    //read raw data
    int16_t AccXraw = imu.rawAccX();
    int16_t AccYraw = imu.rawAccY();
    int16_t AccZraw = imu.rawAccZ();
    int16_t GyrXraw = imu.rawGyrX();
    int16_t GyrYraw = imu.rawGyrY();
    int16_t GyrZraw = imu.rawGyrZ();
    int16_t Tempraw = imu.rawTemp();

    Serial.print(AccXraw);Serial.print("\t");
    Serial.print(AccYraw);Serial.print("\t");
    Serial.print(AccZraw);Serial.print("\t");
    Serial.print(GyrXraw);Serial.print("\t");
    Serial.print(GyrYraw);Serial.print("\t");
    Serial.print(GyrZraw);Serial.print("\t");
    Serial.print(Tempraw);Serial.print("\t");
    Serial.println("");
     delay(50);
}


// Interrupt function
void IRAM_ATTR setImuFlag(){dataReady = true;}