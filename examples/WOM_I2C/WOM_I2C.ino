#include "ICM20689.h"

// an ICM20689 object with the ICM20689 sensor on I2C bus 0 with address 0x68
ICM20689 IMU(Wire,0x68);
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
  // enabling wake on motion low power mode with a threshold of 400 mg and
  // an accelerometer data rate of 15.63 Hz.
  IMU.enableWakeOnMotion(400,ICM20689::LP_ACCEL_ODR_15_63HZ);
  // attaching the interrupt to microcontroller pin 1
  pinMode(1,INPUT);
  attachInterrupt(1,wakeUp,RISING);
}

void loop() {}

void wakeUp() {
  Serial.println("Awake!");
}
