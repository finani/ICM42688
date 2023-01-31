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
    ICM42688(SPIClass &bus, uint8_t csPin, uint32_t SPI_HS_CLK=8000000);

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

    /**
     * @brief      Enables the data ready interrupt.
     *
     *             - routes UI data ready interrupt to INT1
     *             - push-pull, pulsed, active HIGH interrupts
     *
     * @return     ret < 0 if error
     */
    int enableDataReadyInterrupt();

    /**
     * @brief      Masks the data ready interrupt
     *
     * @return     ret < 0 if error
     */
    int disableDataReadyInterrupt();

    /**
     * @brief      Transfers data from ICM 42688-p to microcontroller.
     *             Must be called to access new measurements.
     *
     * @return     ret < 0 if error
     */
    int getAGT();

    /**
     * @brief      Get accelerometer data, per axis
     *
     * @return     Acceleration in g's
     */
    float accX() const { return _acc[0]; }
    float accY() const { return _acc[1]; }
    float accZ() const { return _acc[2]; }

    /**
     * @brief      Get gyro data, per axis
     *
     * @return     Angular velocity in dps
     */
    float gyrX() const { return _gyr[0]; }
    float gyrY() const { return _gyr[1]; }
    float gyrZ() const { return _gyr[2]; }

    /**
     * @brief      Get temperature of gyro die
     *
     * @return     Temperature in Celsius
     */
    float temp() const { return _t; }

    int16_t getAccelX_count();
    int16_t getAccelY_count();
    int16_t getAccelZ_count();
    int16_t getGyroX_count();
    int16_t getGyroY_count();
    int16_t getGyroZ_count();

    int calibrateGyro();
    float getGyroBiasX();
    float getGyroBiasY();
    float getGyroBiasZ();
    void setGyroBiasX(float bias);
    void setGyroBiasY(float bias);
    void setGyroBiasZ(float bias);
    int calibrateAccel();
    float getAccelBiasX_mss();
    float getAccelScaleFactorX();
    float getAccelBiasY_mss();
    float getAccelScaleFactorY();
    float getAccelBiasZ_mss();
    float getAccelScaleFactorZ();
    void setAccelCalX(float bias,float scaleFactor);
    void setAccelCalY(float bias,float scaleFactor);
    void setAccelCalZ(float bias,float scaleFactor);
  protected:
    ///\brief I2C Communication
    uint8_t _address = 0;
    TwoWire *_i2c = {};
    static constexpr uint32_t I2C_CLK = 400000; // 400 kHz
    size_t _numBytes = 0; // number of bytes received from I2C

    int16_t _rawMeas[7]; // temp, accel xyz, gyro xyz

    ///\brief SPI Communication
    SPIClass *_spi = {};
    uint8_t _csPin = 0;
    bool _useSPI = false;
    bool _useSPIHS = false;
    static constexpr uint32_t SPI_LS_CLOCK = 1000000; // 1 MHz
    uint32_t SPI_HS_CLOCK = 8000000; // 8 MHz

    // buffer for reading from sensor
    uint8_t _buffer[15] = {};

    // data buffer
    float _t = 0.0f;
    float _acc[3] = {};
    float _gyr[3] = {};

    ///\brief Full scale resolution factors
    float _accelScale = 0.0f;
    float _gyroScale = 0.0f;

    ///\brief Full scale selections
    AccelFS _accelFS;
    GyroFS _gyroFS;

    ///\brief Accel calibration
    float _accBD[3] = {};
    float _accB[3] = {};
    float _accS[3] = {1.0f, 1.0f, 1.0f};
    float _accMax[3] = {};
    float _accMin[3] = {};

    ///\brief Gyro calibration
    float _gyroBD[3] = {};
    float _gyrB[3] = {};

    ///\brief Constants
    static constexpr uint8_t WHO_AM_I = 0x47; ///< expected value in UB0_REG_WHO_AM_I reg
    static constexpr int NUM_CALIB_SAMPLES = 1000; ///< for gyro/accel bias calib

    ///\brief Conversion formula to get temperature in Celsius (Sec 4.13)
    static constexpr float TEMP_DATA_REG_SCALE = 132.48f;
    static constexpr float TEMP_OFFSET = 25.0f;

    uint8_t _bank = 0; ///< current user bank

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
};

class ICM42688_FIFO: public ICM42688 {
  public:
    using ICM42688::ICM42688;
    int enableFifo(bool accel,bool gyro,bool temp);
    int readFifo();
    void getFifoAccelX_mss(size_t *size,float* data);
    void getFifoAccelY_mss(size_t *size,float* data);
    void getFifoAccelZ_mss(size_t *size,float* data);
    void getFifoGyroX(size_t *size,float* data);
    void getFifoGyroY(size_t *size,float* data);
    void getFifoGyroZ(size_t *size,float* data);
    void getFifoTemperature_C(size_t *size,float* data);
  protected:
    // fifo
    bool _enFifoAccel = false;
    bool _enFifoGyro = false;
    bool _enFifoTemp = false;
    size_t _fifoSize = 0;
    size_t _fifoFrameSize = 0;
    float _axFifo[85] = {};
    float _ayFifo[85] = {};
    float _azFifo[85] = {};
    size_t _aSize = 0;
    float _gxFifo[85] = {};
    float _gyFifo[85] = {};
    float _gzFifo[85] = {};
    size_t _gSize = 0;
    float _tFifo[256] = {};
    size_t _tSize = 0;
};

#endif // ICM42688_H
