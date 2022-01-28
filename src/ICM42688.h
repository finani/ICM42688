#ifndef ICM42688_H
#define ICM42688_H

#include "Arduino.h"
#include "Wire.h"    // I2C library
#include "SPI.h"     // SPI library

class ICM42688{
  public:
    enum GyroRange
    {
      GYRO_RANGE_15_625DPS,
      GYRO_RANGE_31_25DPS,
      GYRO_RANGE_62_5DPS,
      GYRO_RANGE_125DPS,
      GYRO_RANGE_250DPS,
      GYRO_RANGE_500DPS,
      GYRO_RANGE_1000DPS,
      GYRO_RANGE_2000DPS
    };
    enum AccelRange
    {
      ACCEL_RANGE_2G,
      ACCEL_RANGE_4G,
      ACCEL_RANGE_8G,
      ACCEL_RANGE_16G
    };
    ICM42688(TwoWire &bus,uint8_t address);
    ICM42688(SPIClass &bus,uint8_t csPin);
    int begin();
    int setAccelRange(AccelRange range);
    int setGyroRange(GyroRange range);
    int setFilters(bool gyroFilters, bool accFilters);
    int enableDataReadyInterrupt();
    int disableDataReadyInterrupt();
    uint8_t isInterrupted();
    int setUseSPIHS(bool useSPIHS);
    int readSensor();
    int readAcc(double* acc);
    int readGyro(double* gyro);
    int readAccGyro(double* accGyro);
    double getAccelX_mss();
    double getAccelY_mss();
    double getAccelZ_mss();
    double getGyroX_rads();
    double getGyroY_rads();
    double getGyroZ_rads();
    double getGyroX_dps();
    double getGyroY_dps();
    double getGyroZ_dps();
    double getTemperature_C();

    int calibrateGyro();
    double getGyroBiasX_rads();
    double getGyroBiasY_rads();
    double getGyroBiasZ_rads();
    void setGyroBiasX_rads(double bias);
    void setGyroBiasY_rads(double bias);
    void setGyroBiasZ_rads(double bias);
    int calibrateAccel();
    double getAccelBiasX_mss();
    double getAccelScaleFactorX();
    double getAccelBiasY_mss();
    double getAccelScaleFactorY();
    double getAccelBiasZ_mss();
    double getAccelScaleFactorZ();
    void setAccelCalX(double bias,double scaleFactor);
    void setAccelCalY(double bias,double scaleFactor);
    void setAccelCalZ(double bias,double scaleFactor);
  protected:
    // i2c
    uint8_t _address = 0;
    TwoWire *_i2c = {};
    const uint32_t _i2cRate = 400000; // 400 kHz
    size_t _numBytes = 0; // number of bytes received from I2C
    // spi
    SPIClass *_spi = {};
    uint8_t _csPin = 0;
    bool _useSPI = false;
    bool _useSPIHS = false;
    const uint8_t SPI_READ = 0x80;
    const uint32_t SPI_LS_CLOCK = 1000000; // 1 MHz
    const uint32_t SPI_HS_CLOCK = 8000000; // 8 MHz
    // buffer for reading from sensor
    uint8_t _buffer[15] = {};
    // data counts
    int16_t _accCounts[3] = {};
    int16_t _gyroCounts[3] = {};
    int16_t _tcounts = 0;
    // data buffer
    double _acc[3] = {};
    double _gyro[3] = {};
    double _t = 0.0;
    uint8_t _isInterrupted = 0;
    // scale factors
    double _accelScale = 0.0;
    double _gyroScale = 0.0;
    const double _tempScale = 333.87f;
    const double _tempOffset = 21.0f;
    // configuration
    AccelRange _accelRange;
    GyroRange _gyroRange;
    // gyro bias estimation
    size_t _numSamples = 100;
    double _gyroBD[3] = {};
    double _gyroB[3] = {};
    // accel bias and scale factor estimation
    double _accBD[3] = {};
    double _accB[3] = {};
    double _accS[3] = {1.0, 1.0, 1.0};
    double _accMax[3] = {};
    double _accMin[3] = {};
    // transformation matrix
    const int16_t tX[3] = {0,  1,  0};
    const int16_t tY[3] = {1,  0,  0};
    const int16_t tZ[3] = {0,  0, -1};
    // constants
    const double G = 9.807f;
    const double _d2r = 3.14159265359f/180.0f;
    const double _r2d = 180.0f/3.14159265359f;
    // ICM42688 registers
    // BANK 0
    const uint8_t ACCEL_OUT = 0x1F;
    const uint8_t GYRO_OUT = 0x25;
    const uint8_t TEMP_OUT = 0x1D;

    const uint8_t ACCEL_CONFIG0 = 0x50;
    const uint8_t ACCEL_FS_SEL_2G = 0x80; // TODO: 0x60 in datasheet
    const uint8_t ACCEL_FS_SEL_4G = 0x60; // TODO: 0x40 in datasheet
    const uint8_t ACCEL_FS_SEL_8G = 0x40; // TODO: 0x20 in datasheet
    const uint8_t ACCEL_FS_SEL_16G = 0x20; // TODO: 0x00 in datasheet
    const uint8_t ACCEL_ODR_32KHZ = 0x01;
    const uint8_t ACCEL_ODR_16KHZ = 0x02;
    const uint8_t ACCEL_ODR_8KHZ = 0x03;
    const uint8_t ACCEL_ODR_4KHZ = 0x04;
    const uint8_t ACCEL_ODR_2KHZ = 0x05;
    const uint8_t ACCEL_ODR_1KHZ = 0x06;
    const uint8_t ACCEL_ODR_200HZ = 0x07;
    const uint8_t ACCEL_ODR_100HZ = 0x08;
    const uint8_t ACCEL_ODR_50HZ = 0x09;
    const uint8_t ACCEL_ODR_25HZ = 0x0A;
    const uint8_t ACCEL_ODR_12_5HZ = 0x0B;
    const uint8_t ACCEL_ODR_6_25HZ = 0x0C;
    const uint8_t ACCEL_ODR_3_125HZ = 0x0D;
    const uint8_t ACCEL_ODR_1_5625HZ = 0x0E;
    const uint8_t ACCEL_ODR_500HZ = 0x0F;

    const uint8_t GYRO_CONFIG0 = 0x4F;
    const uint8_t GYRO_FS_SEL_15_625DPS = 0xE0;
    const uint8_t GYRO_FS_SEL_31_25DPS = 0xC0;
    const uint8_t GYRO_FS_SEL_62_5DPS = 0xA0;
    const uint8_t GYRO_FS_SEL_125DPS = 0x80;
    const uint8_t GYRO_FS_SEL_250DPS = 0x60;
    const uint8_t GYRO_FS_SEL_500DPS = 0x40;
    const uint8_t GYRO_FS_SEL_1000DPS = 0x20;
    const uint8_t GYRO_FS_SEL_2000DPS = 0x00;
    const uint8_t GYRO_ODR_32KHZ = 0x01;
    const uint8_t GYRO_ODR_16KHZ = 0x02;
    const uint8_t GYRO_ODR_8KHZ = 0x03;
    const uint8_t GYRO_ODR_4KHZ = 0x04;
    const uint8_t GYRO_ODR_2KHZ = 0x05;
    const uint8_t GYRO_ODR_1KHZ = 0x06;
    const uint8_t GYRO_ODR_200HZ = 0x07;
    const uint8_t GYRO_ODR_100HZ = 0x08;
    const uint8_t GYRO_ODR_50HZ = 0x09;
    const uint8_t GYRO_ODR_25HZ = 0x0A;
    const uint8_t GYRO_ODR_12_5HZ = 0x0B;
    const uint8_t GYRO_ODR_500HZ = 0x0F;

    const uint8_t INT_CONFIG = 0x14;
    const uint8_t INT_HOLD_ANY = 0x08;
    const uint8_t INT_PULSE_100us = 0x03;
    const uint8_t INT_SOURCE0 = 0x65;
    const uint8_t RESET_DONE_INT1_EN = 0x10;
    const uint8_t UI_DRDY_INT1_EN = 0x10;
    const uint8_t INT_STATUS = 0x2D;

    const uint8_t DEVICE_CONFIG = 0x11;
    const uint8_t PWR_RESET = 0x80;
    const uint8_t INTF_CONFIG1 = 0x4D;
    const uint8_t CLOCK_SEL_PLL = 0x01;
    const uint8_t PWR_MGMT0 = 0x4E;
    const uint8_t SEN_ENABLE = 0x0F;

    const uint8_t WHO_AM_I = 0x75;
    const uint8_t FIFO_EN = 0x23;
    const uint8_t FIFO_TEMP_EN = 0x04;
    const uint8_t FIFO_GYRO = 0x02;
    const uint8_t FIFO_ACCEL = 0x01;
    const uint8_t FIFO_COUNT = 0x2E;
    const uint8_t FIFO_DATA = 0x30;

    const uint8_t BANK_SEL = 0x76;
    const uint8_t BANK0 = 0x00;
    const uint8_t BANK1 = 0x01;
    const uint8_t BANK2 = 0x02;
    const uint8_t BANK3 = 0x03;
    const uint8_t BANK4 = 0x04;

    // BANK 1
    const uint8_t GYRO_CONFIG_STATIC2 = 0x0B;
    const uint8_t GYRO_NF_ENABLE = 0x00;
    const uint8_t GYRO_NF_DISABLE = 0x01;
    const uint8_t GYRO_AAF_ENABLE = 0x00;
    const uint8_t GYRO_AAF_DISABLE = 0x02;

    // BANK 2
    const uint8_t ACCEL_CONFIG_STATIC2 = 0x03;
    const uint8_t ACCEL_AAF_ENABLE = 0x00;
    const uint8_t ACCEL_AAF_DISABLE = 0x01;

    // private functions
    int writeRegister(uint8_t subAddress, uint8_t data);
    int readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
    int whoAmI();
};

class ICM42688_FIFO: public ICM42688 {
  public:
    using ICM42688::ICM42688;
    int enableFifo(bool accel,bool gyro,bool temp);
    int readFifo();
    void getFifoAccelX_mss(size_t *size,double* data);
    void getFifoAccelY_mss(size_t *size,double* data);
    void getFifoAccelZ_mss(size_t *size,double* data);
    void getFifoGyroX_rads(size_t *size,double* data);
    void getFifoGyroY_rads(size_t *size,double* data);
    void getFifoGyroZ_rads(size_t *size,double* data);
    void getFifoTemperature_C(size_t *size,double* data);
  protected:
    // fifo
    bool _enFifoAccel = false;
    bool _enFifoGyro = false;
    bool _enFifoTemp = false;
    size_t _fifoSize = 0;
    size_t _fifoFrameSize = 0;
    double _axFifo[85] = {};
    double _ayFifo[85] = {};
    double _azFifo[85] = {};
    size_t _aSize = 0;
    double _gxFifo[85] = {};
    double _gyFifo[85] = {};
    double _gzFifo[85] = {};
    size_t _gSize = 0;
    double _tFifo[256] = {};
    size_t _tSize = 0;
};

#endif // ICM42688_H
