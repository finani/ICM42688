#ifndef ICM42688_H
#define ICM42688_H

#include "Arduino.h"
#include "SPI.h"   // SPI library
#include "Wire.h"  // I2C library

class ICM42688 {
 public:
	enum GyroFS : uint8_t {
		dps2000   = 0x00,
		dps1000   = 0x01,
		dps500    = 0x02,
		dps250    = 0x03,
		dps125    = 0x04,
		dps62_5   = 0x05,
		dps31_25  = 0x06,
		dps15_625 = 0x07
	};

	enum AccelFS : uint8_t {
		gpm16 = 0x00,
		gpm8  = 0x01,
		gpm4  = 0x02,
		gpm2  = 0x03
	};

	enum ODR : uint8_t {
		odr32k    = 0x01,  // LN mode only
		odr16k    = 0x02,  // LN mode only
		odr8k     = 0x03,  // LN mode only
		odr4k     = 0x04,  // LN mode only
		odr2k     = 0x05,  // LN mode only
		odr1k     = 0x06,  // LN mode only
		odr200    = 0x07,
		odr100    = 0x08,
		odr50     = 0x09,
		odr25     = 0x0A,
		odr12_5   = 0x0B,
		odr6a25   = 0x0C,  // LP mode only (accel only)
		odr3a125  = 0x0D,  // LP mode only (accel only)
		odr1a5625 = 0x0E,  // LP mode only (accel only)
		odr500    = 0x0F,
	};

	enum GyroNFBWsel : uint8_t {
		nfBW1449Hz = 0x00,
		nfBW680z   = 0x01,
		nfBW329Hz  = 0x02,
		nfBW162Hz  = 0x03,
		nfBW80Hz   = 0x04,
		nfBW40Hz   = 0x05,
		nfBW20Hz   = 0x06,
		nfBW10Hz   = 0x07,
	};

	enum UIFiltOrd : uint8_t {
		first_order  = 0x00,
		second_order = 0x01,
		third_order  = 0x02,
	};

	/**
     * @brief      Constructor for I2C communication
     *
     * @param      bus      I2C bus
     * @param[in]  address  Address of ICM 42688-p device
     */
	ICM42688(TwoWire& bus, uint8_t address);

	/**
     * @brief      Constructor for I2C communication using SDA, SCL pins
     *
     * @param      bus      I2C bus
     * @param[in]  address  Address of ICM 42688-p device
     * @param[in]  sda_pin  GPIO pin to use for I2C SDA signal
     * @param[in]  scl_pin  GPIO pin to use for I2C SCL signal
     */
	ICM42688(TwoWire& bus, uint8_t address, uint8_t sda_pin, uint8_t scl_pin);

	/**
     * @brief      Constructor for SPI communication
     *
     * @param      bus    SPI bus
     * @param[in]  csPin  Chip Select pin
     */
	ICM42688(SPIClass& bus, uint8_t csPin, uint32_t spi_hs_clock = 8'000'000);

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
	int getAccelFS();
	/**
     * @brief      Sets the full scale range for the gyro
     *
     * @param[in]  fssel  Full scale selection
     *
     * @return     ret < 0 if error
     */
	int setGyroFS(GyroFS fssel);
	int getGyroFS();
	/**
     * @brief      Set the ODR for accelerometer
     *
     * @param[in]  odr   Output data rate
     *
     * @return     ret < 0 if error
     */
	int setAccelODR(ODR odr);
	int getAccelODR();

	/**
     * @brief      Set the ODR for gyro
     *
     * @param[in]  odr   Output data rate
     *
     * @return     ret < 0 if error
     */
	int setGyroODR(ODR odr);
	int getGyroODR();

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
     * @brief      Transfers data from ICM 42688-p to mcu.
     *             Must be called to access new measurements.
     *
     * @return     ret < 0 if error
     */
	int getAGT();
	int getRawAGT();

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

	/**
     * @brief      Get accelerometer Raw data, per axis
     *
     * @return     Acceleration in bytes
     */
	int16_t rawAccX() const { return _rawAcc[0]; }

	int16_t rawAccY() const { return _rawAcc[1]; }

	int16_t rawAccZ() const { return _rawAcc[2]; }

	/**
     * @brief      Get gyro raw data, per axis
     *
     * @return     Angular velocity in bytes
     */
	int16_t rawGyrX() const { return _rawGyr[0]; }

	int16_t rawGyrY() const { return _rawGyr[1]; }

	int16_t rawGyrZ() const { return _rawGyr[2]; }

	/**
     * @brief      Get raw temperature of gyro die
     *
     * @return     Temperature in bytes
     */
	int16_t rawTemp() const { return _rawT; }

	/**
     * @brief      Get raw temperature of gyro die
     *
     * @return     Temperature in bytes
     */
	int32_t rawBiasAccX() const { return _rawAccBias[0]; }

	int32_t rawBiasAccY() const { return _rawAccBias[1]; }

	int32_t rawBiasAccZ() const { return _rawAccBias[2]; }

	int32_t rawBiasGyrX() const { return _rawGyrBias[0]; }

	int32_t rawBiasGyrY() const { return _rawGyrBias[1]; }

	int32_t rawBiasGyrZ() const { return _rawGyrBias[2]; }

	int   computeOffsets();
	int   setAllOffsets();                    //Set all Offsets computed
	int   setGyrXOffset(int16_t gyrXoffset);  //#TODO add the getOffset function
	int   setGyrYOffset(int16_t gyrYoffset);
	int   setGyrZOffset(int16_t gyrZoffset);
	int   setAccXOffset(int16_t accXoffset);
	int   setAccYOffset(int16_t accYoffset);
	int   setAccZOffset(int16_t accZoffset);
	float getAccelRes();
	float getGyroRes();
	int   setUIFilterBlock(UIFiltOrd gyroUIFiltOrder, UIFiltOrd accelUIFiltOrder);
	int   setGyroNotchFilter(float gyroNFfreq_x, float gyroNFfreq_y, float gyroNFfreq_z, GyroNFBWsel gyro_nf_bw);  //
	int   selfTest();
	int   testingFunction();

	int   calibrateGyro();
	float getGyroBiasX();
	float getGyroBiasY();
	float getGyroBiasZ();
	void  setGyroBiasX(float bias);
	void  setGyroBiasY(float bias);
	void  setGyroBiasZ(float bias);
	int   calibrateAccel();
	float getAccelBiasX_mss();
	float getAccelScaleFactorX();
	float getAccelBiasY_mss();
	float getAccelScaleFactorY();
	float getAccelBiasZ_mss();
	float getAccelScaleFactorZ();
	void  setAccelCalX(float bias, float scaleFactor);
	void  setAccelCalY(float bias, float scaleFactor);
	void  setAccelCalZ(float bias, float scaleFactor);

 protected:
	///\brief I2C Communication
	uint8_t                   _address  = 0;
	TwoWire*                  _i2c      = {};
	static constexpr uint32_t I2C_CLK   = 400'000;  // 400 kHz
	size_t                    _numBytes = 0;        // number of bytes received from I2C

	///\brief SPI Communication
	SPIClass*                 _spi          = {};
	uint8_t                   _sda_pin      = 18;
	uint8_t                   _scl_pin      = 19;
	uint8_t                   _csPin        = 0;
	bool                      _useSPI       = false;
	bool                      _useSPIHS     = false;
	static constexpr uint32_t SPI_LS_CLOCK  = 1'000'000;  // 1 MHz
	uint32_t                  _spi_hs_clock = 8'000'000;  // 8 MHz

	// buffer for reading from sensor
	uint8_t _buffer[15] = {};

	// data buffer
	float _t      = 0.0f;
	float _acc[3] = {};
	float _gyr[3] = {};

	int16_t _rawT      = 0;
	int16_t _rawAcc[3] = {};
	int16_t _rawGyr[3] = {};

	///\brief Raw Gyro and Accelerometer Bias
	int32_t _rawAccBias[3] = {0, 0, 0};
	int32_t _rawGyrBias[3] = {0, 0, 0};

	///\brief Raw Gyro and Accelerometer Offsets
	int16_t _AccOffset[3] = {0, 0, 0};
	int16_t _GyrOffset[3] = {0, 0, 0};

	///\brief Full scale resolution factors
	float _accelScale = 0.0f;
	float _gyroScale  = 0.0f;

	///\brief Full scale selections
	AccelFS _accelFS = gpm16;
	GyroFS  _gyroFS  = dps2000;

	///\brief Accel calibration
	float _accBD[3]  = {};
	float _accB[3]   = {};
	float _accS[3]   = {1.0f, 1.0f, 1.0f};
	float _accMax[3] = {};
	float _accMin[3] = {};

	///\brief Gyro calibration
	float _gyroBD[3] = {};
	float _gyrB[3]   = {};

	///\brief Constants
	static constexpr uint8_t WHO_AM_I          = 0x47;  ///< expected value in UB0_REG_WHO_AM_I reg
	static constexpr int     NUM_CALIB_SAMPLES = 1000;  ///< for gyro/accel bias calib

	///\brief Conversion formula to get temperature in Celsius (Sec 4.13)
	static constexpr float TEMP_DATA_REG_SCALE = 132.48f;
	static constexpr float TEMP_OFFSET         = 25.0f;

	uint8_t _bank = 0;  ///< current user bank

	const uint8_t FIFO_EN      = 0x5F;
	const uint8_t FIFO_TEMP_EN = 0x04;
	const uint8_t FIFO_GYRO    = 0x02;
	const uint8_t FIFO_ACCEL   = 0x01;
	// const uint8_t FIFO_COUNT = 0x2E;
	// const uint8_t FIFO_DATA = 0x30;

	// BANK 1
	// const uint8_t GYRO_CONFIG_STATIC2 = 0x0B;
	const uint8_t GYRO_NF_ENABLE   = 0x00;
	const uint8_t GYRO_NF_DISABLE  = 0x01;
	const uint8_t GYRO_AAF_ENABLE  = 0x00;
	const uint8_t GYRO_AAF_DISABLE = 0x02;

	// BANK 2
	// const uint8_t ACCEL_CONFIG_STATIC2 = 0x03;
	const uint8_t ACCEL_AAF_ENABLE  = 0x00;
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

	void selfTest(int16_t* accelDiff, int16_t* gyroDiff, float* ratio);
};

class ICM42688_FIFO: public ICM42688 {
 public:
	using ICM42688::ICM42688;
	int  enableFifo(bool accel, bool gyro, bool temp);
	int  streamToFifo();
	int  readFifo();
	void getFifoAccelX_mss(size_t* size, float* data);
	void getFifoAccelY_mss(size_t* size, float* data);
	void getFifoAccelZ_mss(size_t* size, float* data);
	void getFifoGyroX(size_t* size, float* data);
	void getFifoGyroY(size_t* size, float* data);
	void getFifoGyroZ(size_t* size, float* data);
	void getFifoTemperature_C(size_t* size, float* data);

 protected:
	// fifo
	bool   _enFifoAccel     = false;
	bool   _enFifoGyro      = false;
	bool   _enFifoTemp      = false;
	bool   _enFifoTimestamp = false;
	bool   _enFifoHeader    = false;
	size_t _fifoSize        = 0;
	size_t _fifoFrameSize   = 0;
	float  _axFifo[85]      = {};
	float  _ayFifo[85]      = {};
	float  _azFifo[85]      = {};
	size_t _aSize           = 0;
	float  _gxFifo[85]      = {};
	float  _gyFifo[85]      = {};
	float  _gzFifo[85]      = {};
	size_t _gSize           = 0;
	float  _tFifo[256]      = {};
	size_t _tSize           = 0;
};

#endif  // ICM42688_H
