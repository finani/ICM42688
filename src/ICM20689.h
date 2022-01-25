#ifndef ICM20689_H
#define ICM20689_H

#include "Arduino.h"
#include "Wire.h"    // I2C library
#include "SPI.h"     // SPI library

class ICM20689{
  public:
    enum GyroRange
    {
      GYRO_RANGE_250DPS, // FS_SEL[1:0] 00
      GYRO_RANGE_500DPS, // FS_SEL[1:0] 01
      GYRO_RANGE_1000DPS, // FS_SEL[1:0] 10
      GYRO_RANGE_2000DPS // FS_SEL[1:0] 11
    };
    enum AccelRange
    {
      ACCEL_RANGE_2G, // ACCEL_FS_SEL[1:0] 00
      ACCEL_RANGE_4G, // ACCEL_FS_SEL[1:0] 01
      ACCEL_RANGE_8G, // ACCEL_FS_SEL[1:0] 10
      ACCEL_RANGE_16G // ACCEL_FS_SEL[1:0] 11
    };
    enum DlpfBandwidth
    {
      DLPF_BANDWIDTH_MAX,
      DLPF_BANDWIDTH_218HZ, // A_DLPF_CFG 0, 1
      DLPF_BANDWIDTH_99HZ, // A_DLPF_CFG 2
      DLPF_BANDWIDTH_45HZ, // A_DLPF_CFG 3
      DLPF_BANDWIDTH_21HZ, // A_DLPF_CFG 4
      DLPF_BANDWIDTH_10HZ, // A_DLPF_CFG 5
      DLPF_BANDWIDTH_5HZ // A_DLPF_CFG 6
    };
    ICM20689(TwoWire &bus,uint8_t address);
    ICM20689(SPIClass &bus,uint8_t csPin);
    int begin();
    int setAccelRange(AccelRange range);
    int setGyroRange(GyroRange range);
    int setDlpfBandwidth(DlpfBandwidth bandwidth);
    int setSrd(uint8_t srd);
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
    DlpfBandwidth _bandwidth;
    uint8_t _srd = 0;
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
    // ICM20689 registers
    const uint8_t ACCEL_OUT = 0x3B;
    const uint8_t GYRO_OUT = 0x43;
    const uint8_t TEMP_OUT = 0x41;
    const uint8_t ACCEL_CONFIG = 0x1C;
    const uint8_t ACCEL_FS_SEL_2G = 0x00;
    const uint8_t ACCEL_FS_SEL_4G = 0x08;
    const uint8_t ACCEL_FS_SEL_8G = 0x10;
    const uint8_t ACCEL_FS_SEL_16G = 0x18;
    const uint8_t GYRO_CONFIG = 0x1B;
    const uint8_t GYRO_FS_SEL_250DPS = 0x00;
    const uint8_t GYRO_FS_SEL_500DPS = 0x08;
    const uint8_t GYRO_FS_SEL_1000DPS = 0x10;
    const uint8_t GYRO_FS_SEL_2000DPS = 0x18;
    const uint8_t GYRO_FCHOICE_B_8173HZ = 0x01;
    const uint8_t GYRO_FCHOICE_B_3281HZ = 0x10;
    const uint8_t ACCEL_CONFIG2 = 0x1D;
    const uint8_t ACCEL_DLPF_218HZ = 0x01;
    const uint8_t ACCEL_DLPF_99HZ = 0x02;
    const uint8_t ACCEL_DLPF_45HZ = 0x03;
    const uint8_t ACCEL_DLPF_21HZ = 0x04;
    const uint8_t ACCEL_DLPF_10HZ = 0x05;
    const uint8_t ACCEL_DLPF_5HZ = 0x06;
    const uint8_t ACCEL_DLPF_420HZ = 0x07;
    const uint8_t ACCEL_DLPF_1046HZ = 0x08;
    const uint8_t CONFIG = 0x1A;
    const uint8_t GYRO_DLPF_250HZ = 0x00;
    const uint8_t GYRO_DLPF_176HZ = 0x01;
    const uint8_t GYRO_DLPF_92HZ = 0x02;
    const uint8_t GYRO_DLPF_41HZ = 0x03;
    const uint8_t GYRO_DLPF_20HZ = 0x04;
    const uint8_t GYRO_DLPF_10HZ = 0x05;
    const uint8_t GYRO_DLPF_5HZ = 0x06;
    const uint8_t SMPLRT_DIV = 0x19;
    const uint8_t INT_PIN_CFG = 0x37;
    const uint8_t INT_ENABLE = 0x38;
    const uint8_t INT_DISABLE = 0x00;
    const uint8_t INT_HOLD_ANY = 0x30;
    const uint8_t INT_PULSE_50US = 0x00;
    const uint8_t INT_WOM_EN = 0x40;
    const uint8_t INT_RAW_RDY_EN = 0x01;
    const uint8_t INT_STATUS = 0x3A;
    const uint8_t PWR_MGMNT_1 = 0x6B;
    const uint8_t PWR_CYCLE = 0x20;
    const uint8_t PWR_RESET = 0x80;
    const uint8_t CLOCK_SEL_PLL = 0x01;
    const uint8_t PWR_MGMNT_2 = 0x6C;
    const uint8_t SEN_ENABLE = 0x00;
    const uint8_t DIS_GYRO = 0x07;
    const uint8_t DIS_ACC = 0x38;
    const uint8_t MOT_DETECT_CTRL = 0x69;
    const uint8_t ACCEL_INTEL_EN = 0x80;
    const uint8_t ACCEL_INTEL_MODE = 0x40;
    const uint8_t WHO_AM_I = 0x75;
    const uint8_t FIFO_EN = 0x23;
    const uint8_t FIFO_TEMP = 0x80;
    const uint8_t FIFO_GYRO = 0x70;
    const uint8_t FIFO_ACCEL = 0x08;
    const uint8_t FIFO_COUNT = 0x72;
    const uint8_t FIFO_READ = 0x74;
    // private functions
    int writeRegister(uint8_t subAddress, uint8_t data);
    int readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
    int whoAmI();
};

class ICM20689_FIFO: public ICM20689 {
  public:
    using ICM20689::ICM20689;
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

#endif // ICM20689_H
