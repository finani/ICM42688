#ifndef ICM42688_H
#define ICM42688_H

#include "Arduino.h"
#include "Wire.h"    // I2C library
#include "SPI.h"     // SPI library

class ICM42688
{
  public:

    enum GyroFS : uint8_t {
      dps2000 = 0x00,
      dps1000 = 0x01,
      dps500 = 0x02,
      dps250 = 0x03,
      dps125 = 0x04,
      dps62_5 = 0x05,
      dps31_25 = 0x06,
      dps15_625 = 0x07
    };

    enum AccelFS : uint8_t {
      gpm16 = 0x00,
      gpm8 = 0x01,
      gpm4 = 0x02,
      gpm2 = 0x03
    };

    enum ODR : uint8_t {
      odr32k = 0x01, // LN mode only
      odr16k = 0x02, // LN mode only
      odr8k = 0x03, // LN mode only
      odr4k = 0x04, // LN mode only
      odr2k = 0x05, // LN mode only
      odr1k = 0x06, // LN mode only
      odr200 = 0x07,
      odr100 = 0x08,
      odr50 = 0x09,
      odr25 = 0x0A,
      odr12_5 = 0x0B,
      odr6a25 = 0x0C, // LP mode only (accel only)
      odr3a125 = 0x0D, // LP mode only (accel only)
      odr1a5625 = 0x0E, // LP mode only (accel only)
      odr500 = 0x0F,
    };

    /**
     * @brief      Constructor for I2C communication
     *
     * @param      bus      I2C bus
     * @param[in]  address  Address of ICM 42688-p device
     */
    ICM42688(TwoWire &bus, uint8_t address);

    /**
     * @brief      Constructor for SPI communication
     *
     * @param      bus    SPI bus
     * @param[in]  csPin  Chip Select pin
     */
    ICM42688(SPIClass &bus, uint8_t csPin);

    /**
     * @brief      Initialize the device.
     *
     * @return     ret < 0 if error
     */
    int begin();

    /**
     * @brief      Sets the full scale range for the accelerometer
     *
     * @param[in]  fssel  Full scale selection
     *
     * @return     ret < 0 if error
     */
    int setAccelFS(AccelFS fssel);

    /**
     * @brief      Sets the full scale range for the gyro
     *
     * @param[in]  fssel  Full scale selection
     *
     * @return     ret < 0 if error
     */
    int setGyroFS(GyroFS fssel);

    /**
     * @brief      Set the ODR for accelerometer
     *
     * @param[in]  odr   Output data rate
     *
     * @return     ret < 0 if error
     */
    int setAccelODR(ODR odr);

    /**
     * @brief      Set the ODR for gyro
     *
     * @param[in]  odr   Output data rate
     *
     * @return     ret < 0 if error
     */
    int setGyroODR(ODR odr);

    int setFilters(bool gyroFilters, bool accFilters);
    int enableDataReadyInterrupt();
    int disableDataReadyInterrupt();
    uint8_t isInterrupted();
    int setUseSPIHS(bool useSPIHS);

    /**
     * @brief      Transfers data from ICM 42688-p to microcontroller.
     *             Must be called to access new measurements.
     *
     * @return     ret < 0 if error
     */
    int getAGT();

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
    ///\brief I2C Communication
    uint8_t _address = 0;
    TwoWire *_i2c = {};
    const uint32_t _i2cRate = 400000; // 400 kHz
    size_t _numBytes = 0; // number of bytes received from I2C

    ///\brief SPI Communication
    SPIClass *_spi = {};
    uint8_t _csPin = 0;
    bool _useSPI = false;
    bool _useSPIHS = false;
    const uint8_t SPI_READ = 0x80;
    const uint32_t SPI_LS_CLOCK = 1000000; // 1 MHz
    const uint32_t SPI_HS_CLOCK = 8000000; // 8 MHz

    // buffer for reading from sensor
    uint8_t _buffer[15] = {};

    // data buffer
    double _t = 0.0;
    double _acc[3] = {};
    double _gyr[3] = {};
    uint8_t _isInterrupted = 0;

    ///\brief Full scale resolution factors
    double _accelScale = 0.0;
    double _gyroScale = 0.0;

    ///\brief Full scale selections
    AccelFS _accelFS;
    GyroFS _gyroFS;

    ///\brief Accel calibration
    double _accBD[3] = {};
    double _accB[3] = {};
    double _accS[3] = {1.0, 1.0, 1.0};
    double _accMax[3] = {};
    double _accMin[3] = {};

    ///\brief Gyro calibration
    double _gyroBD[3] = {};
    double _gyrB[3] = {};

    // gyro bias estimation
    size_t _numSamples = 100;

    ///\brief Unit conversion constants
    static constexpr double G = 9.807f;
    static constexpr double DEG2RAD = 3.14159265359 / 180.0;
    static constexpr double RAD2DEG = 180.0 / 3.14159265359;


    ///\brief Conversion formula to get temperature in Celsius (Sec 4.13)
    static constexpr double TEMP_DATA_REG_SCALE = 132.48;
    static constexpr double TEMP_OFFSET = 25;

    uint8_t _bank = 0; ///< current user bank

    // const uint8_t INT_CONFIG = 0x14;
    const uint8_t INT_HOLD_ANY = 0x08;
    const uint8_t INT_PULSE_100us = 0x03;
    // const uint8_t INT_SOURCE0 = 0x65;
    const uint8_t RESET_DONE_INT1_EN = 0x10;
    const uint8_t UI_DRDY_INT1_EN = 0x10;
    // const uint8_t INT_STATUS = 0x2D;

    // const uint8_t DEVICE_CONFIG = 0x11;
    // const uint8_t INTF_CONFIG1 = 0x4D;
    // const uint8_t PWR_MGMT0 = 0x4E;

    // const uint8_t WHO_AM_I = 0x75;
    const uint8_t FIFO_EN = 0x23;
    const uint8_t FIFO_TEMP_EN = 0x04;
    const uint8_t FIFO_GYRO = 0x02;
    const uint8_t FIFO_ACCEL = 0x01;
    // const uint8_t FIFO_COUNT = 0x2E;
    // const uint8_t FIFO_DATA = 0x30;

    // BANK 1
    // const uint8_t GYRO_CONFIG_STATIC2 = 0x0B;
    const uint8_t GYRO_NF_ENABLE = 0x00;
    const uint8_t GYRO_NF_DISABLE = 0x01;
    const uint8_t GYRO_AAF_ENABLE = 0x00;
    const uint8_t GYRO_AAF_DISABLE = 0x02;

    // BANK 2
    // const uint8_t ACCEL_CONFIG_STATIC2 = 0x03;
    const uint8_t ACCEL_AAF_ENABLE = 0x00;
    const uint8_t ACCEL_AAF_DISABLE = 0x01;

    // private functions
    int writeRegister(uint8_t subAddress, uint8_t data);
    int readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
    int setBank(uint8_t bank);

    /**
     * @brief      Software reset of the device
     */
    void reset();

    /**
     * @brief      Read the WHO_AM_I register
     *
     * @return     Value of WHO_AM_I register
     */
    uint8_t whoAmI();

    static constexpr uint8_t WHO_AM_I = 0x47; ///< expected value in UB0_REG_WHO_AM_I reg
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
