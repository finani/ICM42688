#include "ICM20689.h"

// an ICM20689 object with the ICM20689 sensor on SPI bus 0 and chip select pin 10
ICM20689 IMU(SPI,10);
int status;

void setup() {
  // serial to display data
  Serial.begin(115200);
  while(!Serial) {}

  // start communication with IMU
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(ICM20689::DLPF_BANDWIDTH_21HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(19);
  // enabling the data ready interrupt
  IMU.enableDataReadyInterrupt();
  // attaching the interrupt to microcontroller pin 1
  pinMode(1,INPUT);
  attachInterrupt(1,getIMU,RISING);
  Serial.println("ax,ay,az,gx,gy,gz,temp_C");
}

void loop() {}

void getIMU(){
  // read the sensor
  IMU.readSensor();
  // display the data
  Serial.print(IMU.getAccelX_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getAccelY_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getAccelZ_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroX_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroY_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroZ_rads(),6);
  Serial.print("\t");
  Serial.println(IMU.getTemperature_C(),6);
}
