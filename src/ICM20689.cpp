#include "Arduino.h"
#include "ICM20689.h"

/* ICM20689 object, input the I2C bus and address */
ICM20689::ICM20689(TwoWire &bus,uint8_t address){
  _i2c = &bus; // I2C bus
  _address = address; // I2C address
  _useSPI = false; // set to use I2C
}

/* ICM20689 object, input the SPI bus and chip select pin */
ICM20689::ICM20689(SPIClass &bus,uint8_t csPin){
  _spi = &bus; // SPI bus
  _csPin = csPin; // chip select pin
  _useSPI = true; // set to use SPI
}

/* starts communication with the ICM20689 */
int ICM20689::begin(){
  if( _useSPI ) { // using SPI for communication
    // use low speed SPI for register setting
    _useSPIHS = false;
    // setting CS pin to output
    pinMode(_csPin,OUTPUT);
    // setting CS pin high
    digitalWrite(_csPin,HIGH);
    // begin SPI communication
    _spi->begin();
  } else { // using I2C for communication
    // starting the I2C bus
    _i2c->begin();
    // setting the I2C clock
    _i2c->setClock(_i2cRate);
  }
  // select clock source to gyro
  if(writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL) < 0) {
    return -1;
  }
  // reset the ICM20689
  writeRegister(PWR_MGMNT_1,PWR_RESET);
  // wait for ICM20689 to come back up
  delay(1);
  // select clock source to gyro
  if(writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL) < 0) {
    return -2;
  }
  // check the WHO AM I byte, expected value is 0x98 (decimal 152)
  if(whoAmI() != 152) {
    return -3;
  }
  // enable accelerometer and gyro
  if(writeRegister(PWR_MGMNT_2,SEN_ENABLE) < 0) {
    return -4;
  }
  // setting accel range to 16G as default
  if(writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_16G) < 0) {
    return -5;
  }
  _accelScale = G * 16.0f/32767.5f; // setting the accel scale to 16G
  _accelRange = ACCEL_RANGE_16G;
  // setting the gyro range to 2000DPS as default
  if(writeRegister(GYRO_CONFIG,GYRO_FS_SEL_2000DPS) < 0) {
    return -6;
  }
  _gyroScale = 2000.0f/32767.5f * _d2r; // setting the gyro scale to 2000DPS
  _gyroRange = GYRO_RANGE_2000DPS;
  // setting bandwidth to 218Hz as default
  if(writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_218HZ) < 0) {
    return -7;
  }
  if(writeRegister(CONFIG,GYRO_DLPF_250HZ) < 0) { // setting gyro bandwidth to 184Hz
    return -8;
  }
  _bandwidth = DLPF_BANDWIDTH_218HZ;
  // setting the sample rate divider to 0 as default
  if(writeRegister(SMPLRT_DIV,0x00) < 0) {
    return -9;
  }
  // estimate gyro bias
  if (calibrateGyro() < 0) {
    return -10;
  }
  // successful init, return 1
  return 1;
}

/* sets the accelerometer full scale range to values other than default */
int ICM20689::setAccelRange(AccelRange range) {
  // use low speed SPI for register setting
  _useSPIHS = false;
  switch(range) {
    case ACCEL_RANGE_2G: {
      // setting the accel range to 2G
      if(writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_2G) < 0){
        return -1;
      }
      _accelScale = G * 2.0f/32767.5f; // setting the accel scale to 2G
      break;
    }
    case ACCEL_RANGE_4G: {
      // setting the accel range to 4G
      if(writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_4G) < 0){
        return -1;
      }
      _accelScale = G * 4.0f/32767.5f; // setting the accel scale to 4G
      break;
    }
    case ACCEL_RANGE_8G: {
      // setting the accel range to 8G
      if(writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_8G) < 0){
        return -1;
      }
      _accelScale = G * 8.0f/32767.5f; // setting the accel scale to 8G
      break;
    }
    case ACCEL_RANGE_16G: {
      // setting the accel range to 16G
      if(writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_16G) < 0){
        return -1;
      }
      _accelScale = G * 16.0f/32767.5f; // setting the accel scale to 16G
      break;
    }
  }
  _accelRange = range;
  return 1;
}

/* sets the gyro full scale range to values other than default */
int ICM20689::setGyroRange(GyroRange range) {
  // use low speed SPI for register setting
  _useSPIHS = false;
  switch(range) {
    case GYRO_RANGE_250DPS: {
      // setting the gyro range to 250DPS
      if(writeRegister(GYRO_CONFIG,GYRO_FS_SEL_250DPS) < 0){
        return -1;
      }
      _gyroScale = 250.0f/32767.5f * _d2r; // setting the gyro scale to 250DPS
      break;
    }
    case GYRO_RANGE_500DPS: {
      // setting the gyro range to 500DPS
      if(writeRegister(GYRO_CONFIG,GYRO_FS_SEL_500DPS) < 0){
        return -1;
      }
      _gyroScale = 500.0f/32767.5f * _d2r; // setting the gyro scale to 500DPS
      break;
    }
    case GYRO_RANGE_1000DPS: {
      // setting the gyro range to 1000DPS
      if(writeRegister(GYRO_CONFIG,GYRO_FS_SEL_1000DPS) < 0){
        return -1;
      }
      _gyroScale = 1000.0f/32767.5f * _d2r; // setting the gyro scale to 1000DPS
      break;
    }
    case GYRO_RANGE_2000DPS: {
      // setting the gyro range to 2000DPS
      if(writeRegister(GYRO_CONFIG,GYRO_FS_SEL_2000DPS) < 0){
        return -1;
      }
      _gyroScale = 2000.0f/32767.5f * _d2r; // setting the gyro scale to 2000DPS
      break;
    }
  }
  _gyroRange = range;
  return 1;
}

/* sets the DLPF bandwidth to values other than default */
int ICM20689::setDlpfBandwidth(DlpfBandwidth bandwidth) {
  // use low speed SPI for register setting
  _useSPIHS = false;
  switch(bandwidth) {
    case DLPF_BANDWIDTH_MAX: {
      if(writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_1046HZ) < 0){ // setting accel bandwidth to 218Hz
        return -1;
      }
      readRegisters(GYRO_CONFIG,1,_buffer);
      if(writeRegister(GYRO_CONFIG,((_buffer[0] & 0xFC) | GYRO_FCHOICE_B_8173HZ)) < 0){ // setting gyro bandwidth to 250Hz
        return -2;
      }
      break;
    }
    case DLPF_BANDWIDTH_218HZ: {
      if(writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_218HZ) < 0){ // setting accel bandwidth to 218Hz
        return -1;
      }
      if(writeRegister(CONFIG,GYRO_DLPF_250HZ) < 0){ // setting gyro bandwidth to 250Hz
        return -2;
      }
      break;
    }
    case DLPF_BANDWIDTH_99HZ: {
      if(writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_99HZ) < 0){ // setting accel bandwidth to 99Hz
        return -1;
      }
      if(writeRegister(CONFIG,GYRO_DLPF_92HZ) < 0){ // setting gyro bandwidth to 92Hz
        return -2;
      }
      break;
    }
    case DLPF_BANDWIDTH_45HZ: {
      if(writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_45HZ) < 0){ // setting accel bandwidth to 45Hz
        return -1;
      }
      if(writeRegister(CONFIG,GYRO_DLPF_41HZ) < 0){ // setting gyro bandwidth to 41Hz
        return -2;
      }
      break;
    }
    case DLPF_BANDWIDTH_21HZ: {
      if(writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_21HZ) < 0){ // setting accel bandwidth to 21Hz
        return -1;
      }
      if(writeRegister(CONFIG,GYRO_DLPF_20HZ) < 0){ // setting gyro bandwidth to 20Hz
        return -2;
      }
      break;
    }
    case DLPF_BANDWIDTH_10HZ: {
      if(writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_10HZ) < 0){ // setting accel bandwidth to 10Hz
        return -1;
      }
      if(writeRegister(CONFIG,GYRO_DLPF_10HZ) < 0){ // setting gyro bandwidth to 10Hz
        return -2;
      }
      break;
    }
    case DLPF_BANDWIDTH_5HZ: {
      if(writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_5HZ) < 0){ // setting accel bandwidth to 5Hz
        return -1;
      }
      if(writeRegister(CONFIG,GYRO_DLPF_5HZ) < 0){ // setting gyro bandwidth to 5Hz
        return -2;
      }
      break;
    }
  }
  _bandwidth = bandwidth;
  return 1;
}

/* sets the sample rate divider to values other than default */
int ICM20689::setSrd(uint8_t srd) {
  // use low speed SPI for register setting
  _useSPIHS = false;
  /* setting the sample rate divider to 19 */
  if(writeRegister(SMPLRT_DIV,19) < 0){ // setting the sample rate divider
    return -1;
  }
  /* setting the sample rate divider */
  if(writeRegister(SMPLRT_DIV,srd) < 0){ // setting the sample rate divider
    return -4;
  }
  _srd = srd;
  return 1;
}

/* enables the data ready interrupt */
int ICM20689::enableDataReadyInterrupt() {
  // use low speed SPI for register setting
  _useSPIHS = false;
  /* setting the interrupt */
  if (writeRegister(INT_PIN_CFG,INT_PULSE_50US) < 0){ // setup interrupt, 50 us pulse
  // if (writeRegister(INT_PIN_CFG,INT_HOLD_ANY) < 0){ // setup interrupt, hold, any read operation
    return -1;
  }
  if (writeRegister(INT_ENABLE,INT_RAW_RDY_EN) < 0){ // set to data ready
    return -2;
  }
  return 1;
}

/* disables the data ready interrupt */
int ICM20689::disableDataReadyInterrupt() {
  // use low speed SPI for register setting
  _useSPIHS = false;
  if(writeRegister(INT_ENABLE,INT_DISABLE) < 0){ // disable interrupt
    return -1;
  }
  return 1;
}

/* disables the data ready interrupt */
uint8_t ICM20689::isInterrupted() {
  _useSPIHS = false; // use the high speed SPI for data readout
  readRegisters(INT_STATUS, 1, &_isInterrupted);
  return _isInterrupted & 0x01;
}

/* set SPI mode */
int ICM20689::setUseSPIHS(bool useSPIHS) {
  _useSPIHS = useSPIHS;
  return 1;
}

/* reads the most current data from ICM20689 and stores in buffer */
int ICM20689::readSensor() {
  _useSPIHS = true; // use the high speed SPI for data readout
  // grab the data from the ICM20689
  if (readRegisters(ACCEL_OUT, 15, _buffer) < 0) {
    return -1;
  }
  // combine into 16 bit values
  _accCounts[0] = (((int16_t)_buffer[0]) << 8) | _buffer[1];
  _accCounts[1] = (((int16_t)_buffer[2]) << 8) | _buffer[3];
  _accCounts[2] = (((int16_t)_buffer[4]) << 8) | _buffer[5];
  _tcounts = (((int16_t)_buffer[6]) << 8) | _buffer[7];
  _gyroCounts[0] = (((int16_t)_buffer[8]) << 8) | _buffer[9];
  _gyroCounts[1] = (((int16_t)_buffer[10]) << 8) | _buffer[11];
  _gyroCounts[2] = (((int16_t)_buffer[12]) << 8) | _buffer[13];
  _acc[0] = (((double)(tX[0]*_accCounts[0] + tX[1]*_accCounts[1] + tX[2]*_accCounts[2]) * _accelScale) - _accB[0])*_accS[0];
  _acc[1] = (((double)(tY[0]*_accCounts[0] + tY[1]*_accCounts[1] + tY[2]*_accCounts[2]) * _accelScale) - _accB[1])*_accS[1];
  _acc[2] = (((double)(tZ[0]*_accCounts[0] + tZ[1]*_accCounts[1] + tZ[2]*_accCounts[2]) * _accelScale) - _accB[2])*_accS[2];
  _t = ((((double) _tcounts) - _tempOffset)/_tempScale) + _tempOffset;
  _gyro[0] = ((double)(tX[0]*_gyroCounts[0] + tX[1]*_gyroCounts[1] + tX[2]*_gyroCounts[2]) * _gyroScale) - _gyroB[0];
  _gyro[1] = ((double)(tY[0]*_gyroCounts[0] + tY[1]*_gyroCounts[1] + tY[2]*_gyroCounts[2]) * _gyroScale) - _gyroB[1];
  _gyro[2] = ((double)(tZ[0]*_gyroCounts[0] + tZ[1]*_gyroCounts[1] + tZ[2]*_gyroCounts[2]) * _gyroScale) - _gyroB[2];
  return 1;
}

/* reads the most current acc data from ICM20689 */
int ICM20689::readAcc(double* acc) {
  _useSPIHS = true; // use the high speed SPI for data readout
  // grab the data from the ICM20689
  if (readRegisters(ACCEL_OUT, 6, _buffer) < 0) {
    return -1;
  }
  // combine into 16 bit values
  _accCounts[0] = (((int16_t)_buffer[0]) << 8) | _buffer[1];
  _accCounts[1] = (((int16_t)_buffer[2]) << 8) | _buffer[3];
  _accCounts[2] = (((int16_t)_buffer[4]) << 8) | _buffer[5];
  _acc[0] = (((double)(tX[0]*_accCounts[0] + tX[1]*_accCounts[1] + tX[2]*_accCounts[2]) * _accelScale) - _accB[0])*_accS[0];
  _acc[1] = (((double)(tY[0]*_accCounts[0] + tY[1]*_accCounts[1] + tY[2]*_accCounts[2]) * _accelScale) - _accB[1])*_accS[1];
  _acc[2] = (((double)(tZ[0]*_accCounts[0] + tZ[1]*_accCounts[1] + tZ[2]*_accCounts[2]) * _accelScale) - _accB[2])*_accS[2];
  memcpy(acc, _acc, 3*sizeof(double));
  return 1;
}

/* reads the most current gyro data from ICM20689 */
int ICM20689::readGyro(double* gyro) {
  _useSPIHS = true; // use the high speed SPI for data readout
  // grab the data from the ICM20689
  if (readRegisters(GYRO_OUT, 6, _buffer) < 0) {
    return -1;
  }
  // combine into 16 bit values
  _gyroCounts[0] = (((int16_t)_buffer[0]) << 8) | _buffer[1];
  _gyroCounts[1] = (((int16_t)_buffer[2]) << 8) | _buffer[3];
  _gyroCounts[2] = (((int16_t)_buffer[4]) << 8) | _buffer[5];
  _gyro[0] = ((double)(tX[0]*_gyroCounts[0] + tX[1]*_gyroCounts[1] + tX[2]*_gyroCounts[2]) * _gyroScale) - _gyroB[0];
  _gyro[1] = ((double)(tY[0]*_gyroCounts[0] + tY[1]*_gyroCounts[1] + tY[2]*_gyroCounts[2]) * _gyroScale) - _gyroB[1];
  _gyro[2] = ((double)(tZ[0]*_gyroCounts[0] + tZ[1]*_gyroCounts[1] + tZ[2]*_gyroCounts[2]) * _gyroScale) - _gyroB[2];
  memcpy(gyro, _gyro, 3*sizeof(double));
  return 1;
}

/* reads the most current accGyro data from ICM20689 */
int ICM20689::readAccGyro(double* accGyro) {
  _useSPIHS = true; // use the high speed SPI for data readout
  // grab the data from the ICM20689
  if (readRegisters(ACCEL_OUT, 15, _buffer) < 0) {
    return -1;
  }
  // combine into 16 bit values
  _accCounts[0] = (((int16_t)_buffer[0]) << 8) | _buffer[1];
  _accCounts[1] = (((int16_t)_buffer[2]) << 8) | _buffer[3];
  _accCounts[2] = (((int16_t)_buffer[4]) << 8) | _buffer[5];
  _gyroCounts[0] = (((int16_t)_buffer[8]) << 8) | _buffer[9];
  _gyroCounts[1] = (((int16_t)_buffer[10]) << 8) | _buffer[11];
  _gyroCounts[2] = (((int16_t)_buffer[12]) << 8) | _buffer[13];
  _acc[0] = (((double)(tX[0]*_accCounts[0] + tX[1]*_accCounts[1] + tX[2]*_accCounts[2]) * _accelScale) - _accB[0])*_accS[0];
  _acc[1] = (((double)(tY[0]*_accCounts[0] + tY[1]*_accCounts[1] + tY[2]*_accCounts[2]) * _accelScale) - _accB[1])*_accS[1];
  _acc[2] = (((double)(tZ[0]*_accCounts[0] + tZ[1]*_accCounts[1] + tZ[2]*_accCounts[2]) * _accelScale) - _accB[2])*_accS[2];
  _gyro[0] = ((double)(tX[0]*_gyroCounts[0] + tX[1]*_gyroCounts[1] + tX[2]*_gyroCounts[2]) * _gyroScale) - _gyroB[0];
  _gyro[1] = ((double)(tY[0]*_gyroCounts[0] + tY[1]*_gyroCounts[1] + tY[2]*_gyroCounts[2]) * _gyroScale) - _gyroB[1];
  _gyro[2] = ((double)(tZ[0]*_gyroCounts[0] + tZ[1]*_gyroCounts[1] + tZ[2]*_gyroCounts[2]) * _gyroScale) - _gyroB[2];
  memcpy(&accGyro[0], _acc, 3*sizeof(double));
  memcpy(&accGyro[3], _gyro, 3*sizeof(double));
  return 1;
}

/* returns the accelerometer measurement in the x direction, m/s/s */
double ICM20689::getAccelX_mss() {
  return _acc[0];
}

/* returns the accelerometer measurement in the y direction, m/s/s */
double ICM20689::getAccelY_mss() {
  return _acc[1];
}

/* returns the accelerometer measurement in the z direction, m/s/s */
double ICM20689::getAccelZ_mss() {
  return _acc[2];
}

/* returns the gyroscope measurement in the x direction, rad/s */
double ICM20689::getGyroX_rads() {
  return _gyro[0];
}

/* returns the gyroscope measurement in the y direction, rad/s */
double ICM20689::getGyroY_rads() {
  return _gyro[1];
}

/* returns the gyroscope measurement in the z direction, rad/s */
double ICM20689::getGyroZ_rads() {
  return _gyro[2];
}

/* returns the gyroscope measurement in the x direction, rad/s */
double ICM20689::getGyroX_dps() {
  return _gyro[0]*_r2d;
}

/* returns the gyroscope measurement in the y direction, rad/s */
double ICM20689::getGyroY_dps() {
  return _gyro[1]*_r2d;
}

/* returns the gyroscope measurement in the z direction, rad/s */
double ICM20689::getGyroZ_dps() {
  return _gyro[2]*_r2d;
}

/* returns the die temperature, C */
double ICM20689::getTemperature_C() {
  return _t;
}

/* configures and enables the FIFO buffer  */
int ICM20689_FIFO::enableFifo(bool accel,bool gyro,bool temp) {
  // use low speed SPI for register setting
  _useSPIHS = false;
  if(writeRegister(FIFO_EN,(accel*FIFO_ACCEL)|(gyro*FIFO_GYRO)|(temp*FIFO_TEMP)) < 0){
    return -2;
  }
  _enFifoAccel = accel;
  _enFifoGyro = gyro;
  _enFifoTemp = temp;
  _fifoFrameSize = accel*6 + gyro*6 + temp*2;
  return 1;
}

/* reads data from the ICM20689 FIFO and stores in buffer */
int ICM20689_FIFO::readFifo() {
  _useSPIHS = true; // use the high speed SPI for data readout
  // get the fifo size
  readRegisters(FIFO_COUNT, 2, _buffer);
  _fifoSize = (((uint16_t) (_buffer[0]&0x0F)) <<8) + (((uint16_t) _buffer[1]));
  // read and parse the buffer
  for (size_t i=0; i < _fifoSize/_fifoFrameSize; i++) {
    // grab the data from the ICM20689
    if (readRegisters(FIFO_READ,_fifoFrameSize,_buffer) < 0) {
      return -1;
    }
    if (_enFifoAccel) {
      // combine into 16 bit values
      _accCounts[0] = (((int16_t)_buffer[0]) << 8) | _buffer[1];
      _accCounts[1] = (((int16_t)_buffer[2]) << 8) | _buffer[3];
      _accCounts[2] = (((int16_t)_buffer[4]) << 8) | _buffer[5];
      // transform and convert to double values
      _axFifo[i] = (((double)(tX[0]*_accCounts[0] + tX[1]*_accCounts[1] + tX[2]*_accCounts[2]) * _accelScale)-_accB[0])*_accS[0];
      _ayFifo[i] = (((double)(tY[0]*_accCounts[0] + tY[1]*_accCounts[1] + tY[2]*_accCounts[2]) * _accelScale)-_accB[1])*_accS[1];
      _azFifo[i] = (((double)(tZ[0]*_accCounts[0] + tZ[1]*_accCounts[1] + tZ[2]*_accCounts[2]) * _accelScale)-_accB[2])*_accS[2];
      _aSize = _fifoSize/_fifoFrameSize;
    }
    if (_enFifoTemp) {
      // combine into 16 bit values
      _tcounts = (((int16_t)_buffer[0 + _enFifoAccel*6]) << 8) | _buffer[1 + _enFifoAccel*6];
      // transform and convert to double values
      _tFifo[i] = ((((double) _tcounts) - _tempOffset)/_tempScale) + _tempOffset;
      _tSize = _fifoSize/_fifoFrameSize;
    }
    if (_enFifoGyro) {
      // combine into 16 bit values
      _gyroCounts[0] = (((int16_t)_buffer[0 + _enFifoAccel*6 + _enFifoTemp*2]) << 8) | _buffer[1 + _enFifoAccel*6 + _enFifoTemp*2];
      _gyroCounts[1] = (((int16_t)_buffer[2 + _enFifoAccel*6 + _enFifoTemp*2]) << 8) | _buffer[3 + _enFifoAccel*6 + _enFifoTemp*2];
      _gyroCounts[2] = (((int16_t)_buffer[4 + _enFifoAccel*6 + _enFifoTemp*2]) << 8) | _buffer[5 + _enFifoAccel*6 + _enFifoTemp*2];
      // transform and convert to double values
      _gxFifo[i] = ((double)(tX[0]*_gyroCounts[0] + tX[1]*_gyroCounts[1] + tX[2]*_gyroCounts[2]) * _gyroScale) - _gyroB[0];
      _gyFifo[i] = ((double)(tY[0]*_gyroCounts[0] + tY[1]*_gyroCounts[1] + tY[2]*_gyroCounts[2]) * _gyroScale) - _gyroB[1];
      _gzFifo[i] = ((double)(tZ[0]*_gyroCounts[0] + tZ[1]*_gyroCounts[1] + tZ[2]*_gyroCounts[2]) * _gyroScale) - _gyroB[2];
      _gSize = _fifoSize/_fifoFrameSize;
    }
  }
  return 1;
}

/* returns the accelerometer FIFO size and data in the x direction, m/s/s */
void ICM20689_FIFO::getFifoAccelX_mss(size_t *size,double* data) {
  *size = _aSize;
  memcpy(data,_axFifo,_aSize*sizeof(double));
}

/* returns the accelerometer FIFO size and data in the y direction, m/s/s */
void ICM20689_FIFO::getFifoAccelY_mss(size_t *size,double* data) {
  *size = _aSize;
  memcpy(data,_ayFifo,_aSize*sizeof(double));
}

/* returns the accelerometer FIFO size and data in the z direction, m/s/s */
void ICM20689_FIFO::getFifoAccelZ_mss(size_t *size,double* data) {
  *size = _aSize;
  memcpy(data,_azFifo,_aSize*sizeof(double));
}

/* returns the gyroscope FIFO size and data in the x direction, rad/s */
void ICM20689_FIFO::getFifoGyroX_rads(size_t *size,double* data) {
  *size = _gSize;
  memcpy(data,_gxFifo,_gSize*sizeof(double));
}

/* returns the gyroscope FIFO size and data in the y direction, rad/s */
void ICM20689_FIFO::getFifoGyroY_rads(size_t *size,double* data) {
  *size = _gSize;
  memcpy(data,_gyFifo,_gSize*sizeof(double));
}

/* returns the gyroscope FIFO size and data in the z direction, rad/s */
void ICM20689_FIFO::getFifoGyroZ_rads(size_t *size,double* data) {
  *size = _gSize;
  memcpy(data,_gzFifo,_gSize*sizeof(double));
}

/* returns the die temperature FIFO size and data, C */
void ICM20689_FIFO::getFifoTemperature_C(size_t *size,double* data) {
  *size = _tSize;
  memcpy(data,_tFifo,_tSize*sizeof(double));
}

/* estimates the gyro biases */
int ICM20689::calibrateGyro() {
  // set the range, bandwidth, and srd
  if (setGyroRange(GYRO_RANGE_250DPS) < 0) {
    return -1;
  }
  if (setDlpfBandwidth(DLPF_BANDWIDTH_21HZ) < 0) {
    return -2;
  }
  if (setSrd(19) < 0) {
    return -3;
  }

  // take samples and find bias
  _gyroBD[0] = 0;
  _gyroBD[1] = 0;
  _gyroBD[2] = 0;
  for (size_t i=0; i < _numSamples; i++) {
    readSensor();
    _gyroBD[0] += (getGyroX_rads() + _gyroB[0])/((double)_numSamples);
    _gyroBD[1] += (getGyroY_rads() + _gyroB[1])/((double)_numSamples);
    _gyroBD[2] += (getGyroZ_rads() + _gyroB[2])/((double)_numSamples);
    delay(20);
  }
  _gyroB[0] = (double)_gyroBD[0];
  _gyroB[1] = (double)_gyroBD[1];
  _gyroB[2] = (double)_gyroBD[2];

  // set the range, bandwidth, and srd back to what they were
  if (setGyroRange(_gyroRange) < 0) {
    return -4;
  }
  if (setDlpfBandwidth(_bandwidth) < 0) {
    return -5;
  }
  if (setSrd(_srd) < 0) {
    return -6;
  }
  return 1;
}

/* returns the gyro bias in the X direction, rad/s */
double ICM20689::getGyroBiasX_rads() {
  return _gyroB[0];
}

/* returns the gyro bias in the Y direction, rad/s */
double ICM20689::getGyroBiasY_rads() {
  return _gyroB[1];
}

/* returns the gyro bias in the Z direction, rad/s */
double ICM20689::getGyroBiasZ_rads() {
  return _gyroB[2];
}

/* sets the gyro bias in the X direction to bias, rad/s */
void ICM20689::setGyroBiasX_rads(double bias) {
  _gyroB[0] = bias;
}

/* sets the gyro bias in the Y direction to bias, rad/s */
void ICM20689::setGyroBiasY_rads(double bias) {
  _gyroB[1] = bias;
}

/* sets the gyro bias in the Z direction to bias, rad/s */
void ICM20689::setGyroBiasZ_rads(double bias) {
  _gyroB[2] = bias;
}

/* finds bias and scale factor calibration for the accelerometer,
this should be run for each axis in each direction (6 total) to find
the min and max values along each */
int ICM20689::calibrateAccel() {
  // set the range, bandwidth, and srd
  if (setAccelRange(ACCEL_RANGE_2G) < 0) {
    return -1;
  }
  if (setDlpfBandwidth(DLPF_BANDWIDTH_21HZ) < 0) {
    return -2;
  }
  if (setSrd(19) < 0) {
    return -3;
  }

  // take samples and find min / max
  _accBD[0] = 0;
  _accBD[1] = 0;
  _accBD[2] = 0;
  for (size_t i=0; i < _numSamples; i++) {
    readSensor();
    _accBD[0] += (getAccelX_mss()/_accS[0] + _accB[0])/((double)_numSamples);
    _accBD[1] += (getAccelY_mss()/_accS[1] + _accB[1])/((double)_numSamples);
    _accBD[2] += (getAccelZ_mss()/_accS[2] + _accB[2])/((double)_numSamples);
    delay(20);
  }
  if (_accBD[0] > 9.0f) {
    _accMax[0] = (double)_accBD[0];
  }
  if (_accBD[1] > 9.0f) {
    _accMax[1] = (double)_accBD[1];
  }
  if (_accBD[2] > 9.0f) {
    _accMax[2] = (double)_accBD[2];
  }
  if (_accBD[0] < -9.0f) {
    _accMin[0] = (double)_accBD[0];
  }
  if (_accBD[1] < -9.0f) {
    _accMin[1] = (double)_accBD[1];
  }
  if (_accBD[2] < -9.0f) {
    _accMin[2] = (double)_accBD[2];
  }

  // find bias and scale factor
  if ((abs(_accMin[0]) > 9.0f) && (abs(_accMax[0]) > 9.0f)) {
    _accB[0] = (_accMin[0] + _accMax[0]) / 2.0f;
    _accS[0] = G/((abs(_accMin[0]) + abs(_accMax[0])) / 2.0f);
  }
  if ((abs(_accMin[1]) > 9.0f) && (abs(_accMax[1]) > 9.0f)) {
    _accB[1] = (_accMin[1] + _accMax[1]) / 2.0f;
    _accS[1] = G/((abs(_accMin[1]) + abs(_accMax[1])) / 2.0f);
  }
  if ((abs(_accMin[2]) > 9.0f) && (abs(_accMax[2]) > 9.0f)) {
    _accB[2] = (_accMin[2] + _accMax[2]) / 2.0f;
    _accS[2] = G/((abs(_accMin[2]) + abs(_accMax[2])) / 2.0f);
  }

  // set the range, bandwidth, and srd back to what they were
  if (setAccelRange(_accelRange) < 0) {
    return -4;
  }
  if (setDlpfBandwidth(_bandwidth) < 0) {
    return -5;
  }
  if (setSrd(_srd) < 0) {
    return -6;
  }
  return 1;
}

/* returns the accelerometer bias in the X direction, m/s/s */
double ICM20689::getAccelBiasX_mss() {
  return _accB[0];
}

/* returns the accelerometer scale factor in the X direction */
double ICM20689::getAccelScaleFactorX() {
  return _accS[0];
}

/* returns the accelerometer bias in the Y direction, m/s/s */
double ICM20689::getAccelBiasY_mss() {
  return _accB[1];
}

/* returns the accelerometer scale factor in the Y direction */
double ICM20689::getAccelScaleFactorY() {
  return _accS[1];
}

/* returns the accelerometer bias in the Z direction, m/s/s */
double ICM20689::getAccelBiasZ_mss() {
  return _accB[2];
}

/* returns the accelerometer scale factor in the Z direction */
double ICM20689::getAccelScaleFactorZ() {
  return _accS[2];
}

/* sets the accelerometer bias (m/s/s) and scale factor in the X direction */
void ICM20689::setAccelCalX(double bias,double scaleFactor) {
  _accB[0] = bias;
  _accS[0] = scaleFactor;
}

/* sets the accelerometer bias (m/s/s) and scale factor in the Y direction */
void ICM20689::setAccelCalY(double bias,double scaleFactor) {
  _accB[1] = bias;
  _accS[1] = scaleFactor;
}

/* sets the accelerometer bias (m/s/s) and scale factor in the Z direction */
void ICM20689::setAccelCalZ(double bias,double scaleFactor) {
  _accB[2] = bias;
  _accS[2] = scaleFactor;
}

/* writes a byte to ICM20689 register given a register address and data */
int ICM20689::writeRegister(uint8_t subAddress, uint8_t data){
  /* write data to device */
  if( _useSPI ){
    _spi->beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3)); // begin the transaction
    digitalWrite(_csPin,LOW); // select the ICM20689 chip
    _spi->transfer(subAddress); // write the register address
    _spi->transfer(data); // write the data
    digitalWrite(_csPin,HIGH); // deselect the ICM20689 chip
    _spi->endTransaction(); // end the transaction
  }
  else{
    _i2c->beginTransmission(_address); // open the device
    _i2c->write(subAddress); // write the register address
    _i2c->write(data); // write the data
    _i2c->endTransmission();
  }

  delay(10);

  /* read back the register */
  readRegisters(subAddress,1,_buffer);
  /* check the read back register against the written register */
  if(_buffer[0] == data) {
    return 1;
  }
  else{
    return -1;
  }
}

/* reads registers from ICM20689 given a starting register address, number of bytes, and a pointer to store data */
int ICM20689::readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest){
  if( _useSPI ){
    // begin the transaction
    if(_useSPIHS){
      _spi->beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE3));
    }
    else{
      _spi->beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
    }
    digitalWrite(_csPin,LOW); // select the ICM20689 chip
    _spi->transfer(subAddress | SPI_READ); // specify the starting register address
    for(uint8_t i = 0; i < count; i++){
      dest[i] = _spi->transfer(0x00); // read the data
    }
    digitalWrite(_csPin,HIGH); // deselect the ICM20689 chip
    _spi->endTransaction(); // end the transaction
    return 1;
  }
  else{
    _i2c->beginTransmission(_address); // open the device
    _i2c->write(subAddress); // specify the starting register address
    _i2c->endTransmission(false);
    _numBytes = _i2c->requestFrom(_address, count); // specify the number of bytes to receive
    if (_numBytes == count) {
      for(uint8_t i = 0; i < count; i++){
        dest[i] = _i2c->read();
      }
      return 1;
    } else {
      return -1;
    }
  }
}

/* gets the ICM20689 WHO_AM_I register value, expected to be 0x98 */
int ICM20689::whoAmI(){
  // read the WHO AM I register
  if (readRegisters(WHO_AM_I,1,_buffer) < 0) {
    return -1;
  }
  // return the register value
  return _buffer[0];
}
