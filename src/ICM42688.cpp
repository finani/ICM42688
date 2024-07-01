#include "ICM42688.h"
#include "registers.h"

using namespace ICM42688reg;

/* ICM42688 object, input the I2C bus and address */
ICM42688::ICM42688(TwoWire& bus, uint8_t address) {
	_i2c     = &bus;     // I2C bus
	_address = address;  // I2C address
	_useSPI  = false;    // set to use I2C
}

/* ICM42688 object, input the I2C bus and address using SDA, SCL pins */
ICM42688::ICM42688(TwoWire& bus, uint8_t address, uint8_t sda_pin, uint8_t scl_pin) {
	_i2c     = &bus;     // I2C bus
	_address = address;  // I2C address
	_useSPI  = false;    // set to use I2C
	_sda_pin = sda_pin;  // set SDA to user defined pin
	_scl_pin = scl_pin;  // set SCL to user defined pin
}

/* ICM42688 object, input the SPI bus and chip select pin */
ICM42688::ICM42688(SPIClass& bus, uint8_t csPin, uint32_t spi_hs_clock) {
	_spi          = &bus;   // SPI bus
	_csPin        = csPin;  // chip select pin
	_useSPI       = true;   // set to use SPI
	_spi_hs_clock = spi_hs_clock;
}

/* starts communication with the ICM42688 */
int ICM42688::begin() {
	if (_useSPI) {  // using SPI for communication
		// use low speed SPI for register setting
		_useSPIHS = false;
		// setting CS pin to output
		pinMode(_csPin, OUTPUT);
		// setting CS pin high
		digitalWrite(_csPin, HIGH);
		// begin SPI communication
		_spi->begin();
	} else {  // using I2C for communication
		// starting the I2C bus
		_i2c->begin(_sda_pin, _scl_pin);
		// setting the I2C clock
		_i2c->setClock(I2C_CLK);
	}

	// reset the ICM42688
	reset();

	// check the WHO AM I byte
	if (whoAmI() != WHO_AM_I) {
		return -3;
	}

	// turn on accel and gyro in Low Noise (LN) Mode
	if (writeRegister(UB0_REG_PWR_MGMT0, 0x0F) < 0) {
		return -4;
	}

	// 16G is default -- do this to set up accel resolution scaling
	int ret = setAccelFS(gpm16);
	if (ret < 0) {
		return ret;
	}

	// 2000DPS is default -- do this to set up gyro resolution scaling
	ret = setGyroFS(dps2000);
	if (ret < 0) {
		return ret;
	}

	// disable inner filters (Notch filter, Anti-alias filter, UI filter block)
	if (setFilters(false, false) < 0) {
		return -7;
	}

	// estimate gyro bias
	if (calibrateGyro() < 0) {
		return -8;
	}
	// successful init, return 1
	return 1;
}

/* sets the accelerometer full scale range to values other than default */
int ICM42688::setAccelFS(AccelFS fssel) {
	// use low speed SPI for register setting
	_useSPIHS = false;

	setBank(0);

	// read current register value
	uint8_t reg;
	if (readRegisters(UB0_REG_ACCEL_CONFIG0, 1, &reg) < 0) {
		return -1;
	}

	// only change FS_SEL in reg
	reg = (fssel << 5) | (reg & 0x1F);

	if (writeRegister(UB0_REG_ACCEL_CONFIG0, reg) < 0) {
		return -2;
	}

	_accelScale = static_cast<float>(1 << (4 - fssel)) / 32768.0f;
	_accelFS    = fssel;

	return 1;
}

/* get the accelerometer full scale range return the ACCEL_FS_SEL value*/
int ICM42688::getAccelFS() {
	// use low speed SPI for register setting
	_useSPIHS = false;
	setBank(0);
	// read current register value
	uint8_t reg;
	if (readRegisters(UB0_REG_ACCEL_CONFIG0, 1, &reg) < 0) {
		return -1;
	}
	return (reg & 0xE0) >> 5;
}

/* sets the gyro full scale range to values other than default */
int ICM42688::setGyroFS(GyroFS fssel) {
	// use low speed SPI for register setting
	_useSPIHS = false;

	setBank(0);

	// read current register value
	uint8_t reg;
	if (readRegisters(UB0_REG_GYRO_CONFIG0, 1, &reg) < 0) {
		return -1;
	}

	// only change FS_SEL in reg
	reg = (fssel << 5) | (reg & 0x1F);

	if (writeRegister(UB0_REG_GYRO_CONFIG0, reg) < 0) {
		return -2;
	}

	_gyroScale = (2000.0f / static_cast<float>(1 << fssel)) / 32768.0f;
	_gyroFS    = fssel;

	return 1;
}

int ICM42688::setAccelODR(ODR odr) {
	// use low speed SPI for register setting
	_useSPIHS = false;

	setBank(0);

	// read current register value
	uint8_t reg;
	if (readRegisters(UB0_REG_ACCEL_CONFIG0, 1, &reg) < 0) {
		return -1;
	}

	// only change ODR in reg
	reg = odr | (reg & 0xF0);

	if (writeRegister(UB0_REG_ACCEL_CONFIG0, reg) < 0) {
		return -2;
	}

	return 1;
}

int ICM42688::setGyroODR(ODR odr) {
	// use low speed SPI for register setting
	_useSPIHS = false;

	setBank(0);

	// read current register value
	uint8_t reg;
	if (readRegisters(UB0_REG_GYRO_CONFIG0, 1, &reg) < 0) {
		return -1;
	}

	// only change ODR in reg
	reg = odr | (reg & 0xF0);

	if (writeRegister(UB0_REG_GYRO_CONFIG0, reg) < 0) {
		return -2;
	}

	return 1;
}

int ICM42688::setFilters(bool gyroFilters, bool accFilters) {
	if (setBank(1) < 0) {
		return -1;
	}

	if (gyroFilters == true) {
		if (writeRegister(UB1_REG_GYRO_CONFIG_STATIC2, GYRO_NF_ENABLE | GYRO_AAF_ENABLE) < 0) {
			return -2;
		}
	} else {
		if (writeRegister(UB1_REG_GYRO_CONFIG_STATIC2, GYRO_NF_DISABLE | GYRO_AAF_DISABLE) < 0) {
			return -3;
		}
	}

	if (setBank(2) < 0) {
		return -4;
	}

	if (accFilters == true) {
		if (writeRegister(UB2_REG_ACCEL_CONFIG_STATIC2, ACCEL_AAF_ENABLE) < 0) {
			return -5;
		}
	} else {
		if (writeRegister(UB2_REG_ACCEL_CONFIG_STATIC2, ACCEL_AAF_DISABLE) < 0) {
			return -6;
		}
	}
	if (setBank(0) < 0) {
		return -7;
	}
	return 1;
}

int ICM42688::enableDataReadyInterrupt() {
	// use low speed SPI for register setting
	_useSPIHS = false;

	// push-pull, pulsed, active HIGH interrupts
	if (writeRegister(UB0_REG_INT_CONFIG, 0x18 | 0x03) < 0) {
		return -1;
	}

	// need to clear bit 4 to allow proper INT1 and INT2 operation
	uint8_t reg;
	if (readRegisters(UB0_REG_INT_CONFIG1, 1, &reg) < 0) {
		return -2;
	}
	reg &= ~0x10;
	if (writeRegister(UB0_REG_INT_CONFIG1, reg) < 0) {
		return -3;
	}

	// route UI data ready interrupt to INT1
	if (writeRegister(UB0_REG_INT_SOURCE0, 0x18) < 0) {
		return -4;
	}

	return 1;
}

int ICM42688::disableDataReadyInterrupt() {
	// use low speed SPI for register setting
	_useSPIHS = false;

	// set pin 4 to return to reset value
	uint8_t reg;
	if (readRegisters(UB0_REG_INT_CONFIG1, 1, &reg) < 0) {
		return -1;
	}
	reg |= 0x10;
	if (writeRegister(UB0_REG_INT_CONFIG1, reg) < 0) {
		return -2;
	}

	// return reg to reset value
	if (writeRegister(UB0_REG_INT_SOURCE0, 0x10) < 0) {
		return -3;
	}

	return 1;
}

/* reads the most current data from ICM42688 and stores in buffer */
int ICM42688::getAGT() {  // modified to use getRawAGT()
	if (getRawAGT() < 0) {
		return -1;
	}

	_t = (static_cast<float>(_rawT) / TEMP_DATA_REG_SCALE) + TEMP_OFFSET;

	_acc[0] = ((_rawAcc[0] * _accelScale) - _accB[0]) * _accS[0];
	_acc[1] = ((_rawAcc[1] * _accelScale) - _accB[1]) * _accS[1];
	_acc[2] = ((_rawAcc[2] * _accelScale) - _accB[2]) * _accS[2];

	_gyr[0] = (_rawGyr[0] * _gyroScale) - _gyrB[0];
	_gyr[1] = (_rawGyr[1] * _gyroScale) - _gyrB[1];
	_gyr[2] = (_rawGyr[2] * _gyroScale) - _gyrB[2];

	return 1;
}

/* reads the most current data from ICM42688 and stores in buffer */
int ICM42688::getRawAGT() {  // Added to return raw data only
	_useSPIHS = true;          // use the high speed SPI for data readout
	// grab the data from the ICM42688
	if (readRegisters(UB0_REG_TEMP_DATA1, 14, _buffer) < 0) {
		return -1;
	}

	// combine bytes into 16 bit values
	int16_t rawMeas[7];  // temp, accel xyz, gyro xyz
	for (size_t i = 0; i < 7; i++) {
		rawMeas[i] = ((int16_t)_buffer[i * 2] << 8) | _buffer[i * 2 + 1];
	}

	_rawT      = rawMeas[0];
	_rawAcc[0] = rawMeas[1];
	_rawAcc[1] = rawMeas[2];
	_rawAcc[2] = rawMeas[3];
	_rawGyr[0] = rawMeas[4];
	_rawGyr[1] = rawMeas[5];
	_rawGyr[2] = rawMeas[6];

	return 1;
}

/* configures and enables the FIFO buffer
  Enforces best-fitting structure (1, 2, or 3) but cannot replicate Packet 4 as choosing between 3/4 requires
  additional argument for high-resolution

  See https://invensense.tdk.com/wp-content/uploads/2020/04/ds-000347_icm-42688-p-datasheet.pdf, 6.1 for details
*/
int ICM42688_FIFO::enableFifo(bool accel, bool gyro, bool temp) {
	_enFifoAccel = accel;
	_enFifoGyro  = gyro;
	_enFifoTemp  = true;  // all structures have 1-byte temp, didn't return error to maintain backwards compatibility
	_enFifoTimestamp =
	  accel && gyro;      // can only read both accel and gyro in Structure 3 or 4, both have 2-byte timestamp
	_enFifoHeader  = accel || gyro;  // if neither sensor requested, FIFO will not send any more packets
	_fifoFrameSize = _enFifoHeader * 1 + _enFifoAccel * 6 + _enFifoGyro * 6 + _enFifoTemp + _enFifoTimestamp * 2;

	// use low speed SPI for register setting
	_useSPIHS = false;
	if (writeRegister(FIFO_EN, (_enFifoAccel * FIFO_ACCEL) | (_enFifoGyro * FIFO_GYRO) | (_enFifoTemp * FIFO_TEMP_EN))
	    < 0) {
		return -2;
	}
	return 1;
}

/* Start streaming, required to read after enableFifo() under most sensor configurations */
int ICM42688_FIFO::streamToFifo() {
	if (writeRegister(ICM42688reg::UB0_REG_FIFO_CONFIG, 1 << 6) < 0) {
		return -2;
	}
	return 1;
}

/* reads data from the ICM42688 FIFO and stores in buffer
  High-resolution mode not yet supported */
int ICM42688_FIFO::readFifo() {
	_useSPIHS = true;  // use the high speed SPI for data readout
	// get the fifo size
	readRegisters(UB0_REG_FIFO_COUNTH, 2, _buffer);
	_fifoSize = (((uint16_t)(_buffer[0] & 0x0F)) << 8) + ((uint16_t)_buffer[1]);

	// precalculate packet structure as per-packet recalculation based on headers isn't reliable
	// header does not confirm whether packet is sized for high-resolution (20-bit) data
	size_t numFrames = _fifoSize / _fifoFrameSize;
	size_t accIndex  = 1;
	size_t gyroIndex = accIndex + _enFifoAccel * 6;
	size_t tempIndex = gyroIndex + _enFifoGyro * 6;
	// read and parse the buffer
	for (size_t i = 0; i < _fifoSize / _fifoFrameSize; i++) {
		// grab the data from the ICM42688
		if (readRegisters(UB0_REG_FIFO_DATA, _fifoFrameSize, _buffer) < 0) {
			return -1;
		}
		if (_enFifoAccel) {
			// combine into 16 bit values
			int16_t rawMeas[3];
			rawMeas[0] = (((int16_t)_buffer[0 + accIndex]) << 8) | _buffer[1 + accIndex];
			rawMeas[1] = (((int16_t)_buffer[2 + accIndex]) << 8) | _buffer[3 + accIndex];
			rawMeas[2] = (((int16_t)_buffer[4 + accIndex]) << 8) | _buffer[5 + accIndex];
			// transform and convert to float values
			_axFifo[i] = ((rawMeas[0] * _accelScale) - _accB[0]) * _accS[0];
			_ayFifo[i] = ((rawMeas[1] * _accelScale) - _accB[1]) * _accS[1];
			_azFifo[i] = ((rawMeas[2] * _accelScale) - _accB[2]) * _accS[2];
			_aSize     = numFrames;
		}
		if (_enFifoTemp) {
			int8_t rawMeas = _buffer[tempIndex + 0];
			// transform and convert to float values
			_tFifo[i] = (static_cast<float>(rawMeas) / TEMP_DATA_REG_SCALE) + TEMP_OFFSET;
			_tSize    = numFrames;
		}
		if (_enFifoGyro) {
			// combine into 16 bit values
			int16_t rawMeas[3];
			rawMeas[0] = (((int16_t)_buffer[0 + gyroIndex]) << 8) | _buffer[1 + gyroIndex];
			rawMeas[1] = (((int16_t)_buffer[2 + gyroIndex]) << 8) | _buffer[3 + gyroIndex];
			rawMeas[2] = (((int16_t)_buffer[4 + gyroIndex]) << 8) | _buffer[5 + gyroIndex];
			// transform and convert to float values
			_gxFifo[i] = (rawMeas[0] * _gyroScale) - _gyrB[0];
			_gyFifo[i] = (rawMeas[1] * _gyroScale) - _gyrB[1];
			_gzFifo[i] = (rawMeas[2] * _gyroScale) - _gyrB[2];
			_gSize     = numFrames;
		}
	}
	return 1;
}

/* returns the accelerometer FIFO size and data in the x direction, m/s/s */
void ICM42688_FIFO::getFifoAccelX_mss(size_t* size, float* data) {
	*size = _aSize;
	memcpy(data, _axFifo, _aSize * sizeof(float));
}

/* returns the accelerometer FIFO size and data in the y direction, m/s/s */
void ICM42688_FIFO::getFifoAccelY_mss(size_t* size, float* data) {
	*size = _aSize;
	memcpy(data, _ayFifo, _aSize * sizeof(float));
}

/* returns the accelerometer FIFO size and data in the z direction, m/s/s */
void ICM42688_FIFO::getFifoAccelZ_mss(size_t* size, float* data) {
	*size = _aSize;
	memcpy(data, _azFifo, _aSize * sizeof(float));
}

/* returns the gyroscope FIFO size and data in the x direction, dps */
void ICM42688_FIFO::getFifoGyroX(size_t* size, float* data) {
	*size = _gSize;
	memcpy(data, _gxFifo, _gSize * sizeof(float));
}

/* returns the gyroscope FIFO size and data in the y direction, dps */
void ICM42688_FIFO::getFifoGyroY(size_t* size, float* data) {
	*size = _gSize;
	memcpy(data, _gyFifo, _gSize * sizeof(float));
}

/* returns the gyroscope FIFO size and data in the z direction, dps */
void ICM42688_FIFO::getFifoGyroZ(size_t* size, float* data) {
	*size = _gSize;
	memcpy(data, _gzFifo, _gSize * sizeof(float));
}

/* returns the die temperature FIFO size and data, C */
void ICM42688_FIFO::getFifoTemperature_C(size_t* size, float* data) {
	*size = _tSize;
	memcpy(data, _tFifo, _tSize * sizeof(float));
}

/* estimates the gyro biases */
int ICM42688::calibrateGyro() {
	// set at a lower range (more resolution) since IMU not moving
	const GyroFS current_fssel = _gyroFS;
	if (setGyroFS(dps250) < 0) {
		return -1;
	}

	// take samples and find bias
	_gyroBD[0] = 0;
	_gyroBD[1] = 0;
	_gyroBD[2] = 0;
	for (size_t i = 0; i < NUM_CALIB_SAMPLES; i++) {
		getAGT();
		_gyroBD[0] += (gyrX() + _gyrB[0]) / NUM_CALIB_SAMPLES;
		_gyroBD[1] += (gyrY() + _gyrB[1]) / NUM_CALIB_SAMPLES;
		_gyroBD[2] += (gyrZ() + _gyrB[2]) / NUM_CALIB_SAMPLES;
		delay(1);
	}
	_gyrB[0] = _gyroBD[0];
	_gyrB[1] = _gyroBD[1];
	_gyrB[2] = _gyroBD[2];

	// recover the full scale setting
	if (setGyroFS(current_fssel) < 0) {
		return -4;
	}
	return 1;
}

/* returns the gyro bias in the X direction, dps */
float ICM42688::getGyroBiasX() {
	return _gyrB[0];
}

/* returns the gyro bias in the Y direction, dps */
float ICM42688::getGyroBiasY() {
	return _gyrB[1];
}

/* returns the gyro bias in the Z direction, dps */
float ICM42688::getGyroBiasZ() {
	return _gyrB[2];
}

/* sets the gyro bias in the X direction to bias, dps */
void ICM42688::setGyroBiasX(float bias) {
	_gyrB[0] = bias;
}

/* sets the gyro bias in the Y direction to bias, dps */
void ICM42688::setGyroBiasY(float bias) {
	_gyrB[1] = bias;
}

/* sets the gyro bias in the Z direction to bias, dps */
void ICM42688::setGyroBiasZ(float bias) {
	_gyrB[2] = bias;
}

/* finds bias and scale factor calibration for the accelerometer,
this should be run for each axis in each direction (6 total) to find
the min and max values along each */
int ICM42688::calibrateAccel() {
	// set at a lower range (more resolution) since IMU not moving
	const AccelFS current_fssel = _accelFS;
	if (setAccelFS(gpm2) < 0) {
		return -1;
	}

	// take samples and find min / max
	_accBD[0] = 0;
	_accBD[1] = 0;
	_accBD[2] = 0;
	for (size_t i = 0; i < NUM_CALIB_SAMPLES; i++) {
		getAGT();
		_accBD[0] += (accX() / _accS[0] + _accB[0]) / NUM_CALIB_SAMPLES;
		_accBD[1] += (accY() / _accS[1] + _accB[1]) / NUM_CALIB_SAMPLES;
		_accBD[2] += (accZ() / _accS[2] + _accB[2]) / NUM_CALIB_SAMPLES;
		delay(1);
	}
	if (_accBD[0] > 0.9f) {
		_accMax[0] = _accBD[0];
	}
	if (_accBD[1] > 0.9f) {
		_accMax[1] = _accBD[1];
	}
	if (_accBD[2] > 0.9f) {
		_accMax[2] = _accBD[2];
	}
	if (_accBD[0] < -0.9f) {
		_accMin[0] = _accBD[0];
	}
	if (_accBD[1] < -0.9f) {
		_accMin[1] = _accBD[1];
	}
	if (_accBD[2] < -0.9f) {
		_accMin[2] = _accBD[2];
	}

	// find bias and scale factor
	if ((abs(_accMin[0]) > 0.9f) && (abs(_accMax[0]) > 0.9f)) {
		_accB[0] = (_accMin[0] + _accMax[0]) / 2.0f;
		_accS[0] = 1 / ((abs(_accMin[0]) + abs(_accMax[0])) / 2.0f);
	}
	if ((abs(_accMin[1]) > 0.9f) && (abs(_accMax[1]) > 0.9f)) {
		_accB[1] = (_accMin[1] + _accMax[1]) / 2.0f;
		_accS[1] = 1 / ((abs(_accMin[1]) + abs(_accMax[1])) / 2.0f);
	}
	if ((abs(_accMin[2]) > 0.9f) && (abs(_accMax[2]) > 0.9f)) {
		_accB[2] = (_accMin[2] + _accMax[2]) / 2.0f;
		_accS[2] = 1 / ((abs(_accMin[2]) + abs(_accMax[2])) / 2.0f);
	}

	// recover the full scale setting
	if (setAccelFS(current_fssel) < 0) {
		return -4;
	}
	return 1;
}

/* returns the accelerometer bias in the X direction, m/s/s */
float ICM42688::getAccelBiasX_mss() {
	return _accB[0];
}

/* returns the accelerometer scale factor in the X direction */
float ICM42688::getAccelScaleFactorX() {
	return _accS[0];
}

/* returns the accelerometer bias in the Y direction, m/s/s */
float ICM42688::getAccelBiasY_mss() {
	return _accB[1];
}

/* returns the accelerometer scale factor in the Y direction */
float ICM42688::getAccelScaleFactorY() {
	return _accS[1];
}

/* returns the accelerometer bias in the Z direction, m/s/s */
float ICM42688::getAccelBiasZ_mss() {
	return _accB[2];
}

/* returns the accelerometer scale factor in the Z direction */
float ICM42688::getAccelScaleFactorZ() {
	return _accS[2];
}

/* sets the accelerometer bias (m/s/s) and scale factor in the X direction */
void ICM42688::setAccelCalX(float bias, float scaleFactor) {
	_accB[0] = bias;
	_accS[0] = scaleFactor;
}

/* sets the accelerometer bias (m/s/s) and scale factor in the Y direction */
void ICM42688::setAccelCalY(float bias, float scaleFactor) {
	_accB[1] = bias;
	_accS[1] = scaleFactor;
}

/* sets the accelerometer bias (m/s/s) and scale factor in the Z direction */
void ICM42688::setAccelCalZ(float bias, float scaleFactor) {
	_accB[2] = bias;
	_accS[2] = scaleFactor;
}

/* writes a byte to ICM42688 register given a register address and data */
int ICM42688::writeRegister(uint8_t subAddress, uint8_t data) {
	/* write data to device */
	if (_useSPI) {
		_spi->beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));  // begin the transaction
		digitalWrite(_csPin, LOW);                                               // select the ICM42688 chip
		_spi->transfer(subAddress);                                              // write the register address
		_spi->transfer(data);                                                    // write the data
		digitalWrite(_csPin, HIGH);                                              // deselect the ICM42688 chip
		_spi->endTransaction();                                                  // end the transaction
	} else {
		_i2c->beginTransmission(_address);                                       // open the device
		_i2c->write(subAddress);                                                 // write the register address
		_i2c->write(data);                                                       // write the data
		_i2c->endTransmission();
	}

	delay(10);

	/* read back the register */
	readRegisters(subAddress, 1, _buffer);
	/* check the read back register against the written register */
	if (_buffer[0] == data) {
		return 1;
	} else {
		return -1;
	}
}

/* reads registers from ICM42688 given a starting register address, number of bytes, and a pointer to store data */
int ICM42688::readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest) {
	if (_useSPI) {
		// begin the transaction
		if (_useSPIHS) {
			_spi->beginTransaction(SPISettings(_spi_hs_clock, MSBFIRST, SPI_MODE3));
		} else {
			_spi->beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
		}
		digitalWrite(_csPin, LOW);                       // select the ICM42688 chip
		_spi->transfer(subAddress | 0x80);               // specify the starting register address
		for (uint8_t i = 0; i < count; i++) {
			dest[i] = _spi->transfer(0x00);                // read the data
		}
		digitalWrite(_csPin, HIGH);                      // deselect the ICM42688 chip
		_spi->endTransaction();                          // end the transaction
		return 1;
	} else {
		_i2c->beginTransmission(_address);               // open the device
		_i2c->write(subAddress);                         // specify the starting register address
		_i2c->endTransmission(false);
		_numBytes = _i2c->requestFrom(_address, count);  // specify the number of bytes to receive
		if (_numBytes == count) {
			for (uint8_t i = 0; i < count; i++) {
				dest[i] = _i2c->read();
			}
			return 1;
		} else {
			return -1;
		}
	}
}

int ICM42688::setBank(uint8_t bank) {
	// if we are already on this bank, bail
	if (_bank == bank) {
		return 1;
	}

	_bank = bank;

	return writeRegister(REG_BANK_SEL, bank);
}

void ICM42688::reset() {
	setBank(0);

	writeRegister(UB0_REG_DEVICE_CONFIG, 0x01);

	// wait for ICM42688 to come back up
	delay(1);
}

/* gets the ICM42688 WHO_AM_I register value */
uint8_t ICM42688::whoAmI() {
	setBank(0);

	// read the WHO AM I register
	if (readRegisters(UB0_REG_WHO_AM_I, 1, _buffer) < 0) {
		return -1;
	}
	// return the register value
	return _buffer[0];
}

/* get Raw Bias (Offsets)*/  //Added to use Offsets rather than compensating through additional code
int ICM42688::computeOffsets() {
	const AccelFS current_Accelfssel = _accelFS;
	const GyroFS  current_Gyrofssel  = _gyroFS;

	// set the IMU at the correct resolution
	setAccelFS(ICM42688::AccelFS::gpm2);
	setGyroFS(ICM42688::GyroFS::dps250);  //check if this is the right one
	int16_t FullScale_Acc = 2;
	int16_t FullScale_Gyr = 250;

	// reset the Offset_user
	setBank(4);
	if (writeRegister(UB4_REG_OFFSET_USER5, 0) < 0) {
		return -2;  // lower Ax byte
	}
	if (writeRegister(UB4_REG_OFFSET_USER6, 0) < 0) {
		return -2;  // lower Ay byte
	}
	if (writeRegister(UB4_REG_OFFSET_USER8, 0) < 0) {
		return -2;  // lower Az byte
	}
	if (writeRegister(UB4_REG_OFFSET_USER2, 0) < 0) {
		return -2;  // lower Gy byte
	}
	if (writeRegister(UB4_REG_OFFSET_USER3, 0) < 0) {
		return -2;  // lower Gz byte
	}
	if (writeRegister(UB4_REG_OFFSET_USER0, 0) < 0) {
		return -2;  // lower Gx byte
	}
	if (writeRegister(UB4_REG_OFFSET_USER4, 0) < 0) {
		return -2;  // upper Ax and Gz bytes
	}
	if (writeRegister(UB4_REG_OFFSET_USER7, 0) < 0) {
		return -2;  // upper Az and Ay bytes
	}
	if (writeRegister(UB4_REG_OFFSET_USER1, 0) < 0) {
		return -2;  // upper Gy and Gx bytes
	}
	setBank(0);
	// reinitialize the _rawAccBias and _rawGyrBias
	for (size_t ii = 1; ii < 3; ii++) {
		_rawAccBias[ii] = 0;
		_rawGyrBias[ii] = 0;
	}
	// record raw values and add samples
	for (size_t i = 0; i < NUM_CALIB_SAMPLES; i++) {
		getRawAGT();
		_rawAccBias[0] += _rawAcc[0];
		_rawAccBias[1] += _rawAcc[1];
		_rawAccBias[2] += _rawAcc[2];
		_rawGyrBias[0] += _rawGyr[0];
		_rawGyrBias[1] += _rawGyr[1];
		_rawGyrBias[2] += _rawGyr[2];
		delay(1);
	}

	// Average
	_rawAccBias[0] = (int32_t)((double)_rawAccBias[0] / (double)NUM_CALIB_SAMPLES);
	_rawAccBias[1] = (int32_t)((double)_rawAccBias[1] / (double)NUM_CALIB_SAMPLES);
	_rawAccBias[2] = (int32_t)((double)_rawAccBias[2] / (double)NUM_CALIB_SAMPLES);
	_rawGyrBias[0] = (int32_t)((double)_rawGyrBias[0] / (double)NUM_CALIB_SAMPLES);
	_rawGyrBias[1] = (int32_t)((double)_rawGyrBias[1] / (double)NUM_CALIB_SAMPLES);
	_rawGyrBias[2] = (int32_t)((double)_rawGyrBias[2] / (double)NUM_CALIB_SAMPLES);

	//compensate gravity and compute the _AccOffset and _GyrOffset
	for (size_t ii = 0; ii < 3; ii++) {
		_AccOffset[ii] =
		  (int16_t)(-(_rawAccBias[ii]) * (FullScale_Acc / 32768.0f * 2048));  //*2048));  // 0.5 mg resolution
		if (_rawAccBias[ii] * FullScale_Acc > 26'000) {
			_AccOffset[ii] = (int16_t)(-(_rawAccBias[ii] - 32'768 / FullScale_Acc) * (FullScale_Acc / 32768.0f * 2048));
		}  //26000 ~80% of 32768
		if (_rawAccBias[ii] * FullScale_Acc < -26'000) {
			_AccOffset[ii] = (int16_t)(-(_rawAccBias[ii] + 32'768 / FullScale_Acc) * (FullScale_Acc / 32768.0f * 2048));
		}
		_GyrOffset[ii] = (int16_t)((-_rawGyrBias[ii]) * (FullScale_Gyr / 32768.0f * 32));  //1/32 dps resolution
	}

	// Serial.println("The new raw Bias are:");
	// for(size_t ii = 0; ii< 3; ii++){
	//   Serial.print(_rawAccBias[ii]);Serial.print("\t");
	// }
	//  for(size_t ii = 0; ii< 3; ii++){
	//   Serial.print(_rawGyrBias[ii]);Serial.print("\t");
	// }
	// Serial.println("");

	// Serial.println("The new Offsets are:");
	// for(size_t ii = 0; ii< 3; ii++){
	//   Serial.print(_AccOffset[ii]);Serial.print("\t");
	// }
	//  for(size_t ii = 0; ii< 3; ii++){
	//   Serial.print(_GyrOffset[ii]);Serial.print("\t");
	// }
	// Serial.println("");

	if (setAccelFS(current_Accelfssel) < 0) {
		return -4;
	}
	if (setGyroFS(current_Gyrofssel) < 0) {
		return -4;
	}
	return 1;
}

int ICM42688::setAllOffsets() {
	setBank(4);
	uint8_t reg;

	// clear all offsets:
	if (writeRegister(UB4_REG_OFFSET_USER0, 0) < 0) {
		return -2;
	}
	if (writeRegister(UB4_REG_OFFSET_USER1, 0) < 0) {
		return -2;
	}
	if (writeRegister(UB4_REG_OFFSET_USER2, 0) < 0) {
		return -2;
	}
	if (writeRegister(UB4_REG_OFFSET_USER3, 0) < 0) {
		return -2;
	}
	if (writeRegister(UB4_REG_OFFSET_USER4, 0) < 0) {
		return -2;
	}
	if (writeRegister(UB4_REG_OFFSET_USER5, 0) < 0) {
		return -2;
	}
	if (writeRegister(UB4_REG_OFFSET_USER6, 0) < 0) {
		return -2;
	}
	if (writeRegister(UB4_REG_OFFSET_USER7, 0) < 0) {
		return -2;
	}

	reg = _AccOffset[0] & 0x00FF;  // lower Ax byte
	if (writeRegister(UB4_REG_OFFSET_USER5, reg) < 0) {
		return -2;
	}
	reg = _AccOffset[1] & 0x00FF;  // lower Ay byte
	if (writeRegister(UB4_REG_OFFSET_USER6, reg) < 0) {
		return -2;
	}
	reg = _AccOffset[2] & 0x00FF;  // lower Az byte
	if (writeRegister(UB4_REG_OFFSET_USER8, reg) < 0) {
		return -2;
	}

	reg = _GyrOffset[1] & 0x00FF;  // lower Gy byte
	if (writeRegister(UB4_REG_OFFSET_USER2, reg) < 0) {
		return -2;
	}
	reg = _GyrOffset[2] & 0x00FF;  // lower Gz byte
	if (writeRegister(UB4_REG_OFFSET_USER3, reg) < 0) {
		return -2;
	}
	reg = _GyrOffset[0] & 0x00FF;  // lower Gx byte
	if (writeRegister(UB4_REG_OFFSET_USER0, reg) < 0) {
		return -2;
	}

	reg = (_AccOffset[0] & 0x0F00) >> 4 | (_GyrOffset[2] & 0x0F00) >> 8;  // upper Ax and Gz bytes
	if (writeRegister(UB4_REG_OFFSET_USER4, reg) < 0) {
		return -2;
	}
	reg = (_AccOffset[2] & 0x0F00) >> 4 | (_AccOffset[1] & 0x0F00) >> 8;  // upper Az and Ay bytes
	if (writeRegister(UB4_REG_OFFSET_USER7, reg) < 0) {
		return -2;
	}
	reg = (_GyrOffset[1] & 0x0F00) >> 4 | (_GyrOffset[0] & 0x0F00) >> 8;  // upper Gy and Gx bytes
	if (writeRegister(UB4_REG_OFFSET_USER1, reg) < 0) {
		return -2;
	}
	setBank(0);
	return 1;
}

/* Set Gyro and Accel Offsets individually*/
int ICM42688::setAccXOffset(int16_t accXoffset) {
	setBank(4);
	uint8_t reg1 = (accXoffset & 0x00FF);
	uint8_t reg2;
	if (readRegisters(UB4_REG_OFFSET_USER4, 1, &reg2) < 0) {
		return -1;
	}
	reg2 = (reg2 & 0x0F) | ((accXoffset & 0x0F00) >> 4);
	if (writeRegister(UB4_REG_OFFSET_USER5, reg1) < 0) {
		return -2;
	}
	if (writeRegister(UB4_REG_OFFSET_USER4, reg2) < 0) {
		return -2;
	}
	setBank(0);
	return 1;
}

int ICM42688::setAccYOffset(int16_t accYoffset) {
	setBank(4);
	uint8_t reg1 = (accYoffset & 0x00FF);
	uint8_t reg2;
	if (readRegisters(UB4_REG_OFFSET_USER7, 1, &reg2) < 0) {
		return -1;
	}
	reg2 = (reg2 & 0xF0) | ((accYoffset & 0x0F00) >> 8);
	if (writeRegister(UB4_REG_OFFSET_USER6, reg1) < 0) {
		return -2;
	}
	if (writeRegister(UB4_REG_OFFSET_USER7, reg2) < 0) {
		return -2;
	}
	setBank(0);
	return 1;
}

int ICM42688::setAccZOffset(int16_t accZoffset) {
	setBank(4);
	uint8_t reg1 = accZoffset & 0x00FF;
	uint8_t reg2;
	if (readRegisters(UB4_REG_OFFSET_USER7, 1, &reg2) < 0) {
		return -1;
	}
	reg2 = (reg2 & 0x0F) | ((accZoffset & 0x0F00) >> 4);
	if (writeRegister(UB4_REG_OFFSET_USER8, reg1) < 0) {
		return -2;
	}
	if (writeRegister(UB4_REG_OFFSET_USER7, reg2) < 0) {
		return -2;
	}
	setBank(0);
	return 1;
}

int ICM42688::setGyrXOffset(int16_t gyrXoffset) {
	setBank(4);
	uint8_t reg1 = gyrXoffset & 0x00FF;
	uint8_t reg2;
	if (readRegisters(UB4_REG_OFFSET_USER1, 1, &reg2) < 0) {
		return -1;
	}
	reg2 = (reg2 & 0xF0) | ((gyrXoffset & 0x0F00) >> 8);
	if (writeRegister(UB4_REG_OFFSET_USER0, reg1) < 0) {
		return -2;
	}
	if (writeRegister(UB4_REG_OFFSET_USER1, reg2) < 0) {
		return -2;
	}
	setBank(0);
	return 1;
}

int ICM42688::setGyrYOffset(int16_t gyrYoffset) {
	setBank(4);
	uint8_t reg1 = gyrYoffset & 0x00FF;
	uint8_t reg2;
	if (readRegisters(UB4_REG_OFFSET_USER1, 1, &reg2) < 0) {
		return -1;
	}
	reg2 = (reg2 & 0x0F) | ((gyrYoffset & 0x0F00) >> 4);
	reg2 = (gyrYoffset & 0x0F00) >> 4 | (reg2 & 0x0F00) >> 4;
	if (writeRegister(UB4_REG_OFFSET_USER2, reg1) < 0) {
		return -2;
	}
	if (writeRegister(UB4_REG_OFFSET_USER1, reg2) < 0) {
		return -2;
	}
	setBank(0);
	return 1;
}

int ICM42688::setGyrZOffset(int16_t gyrZoffset) {
	setBank(4);
	uint8_t reg1 = gyrZoffset & 0x00FF;
	uint8_t reg2;
	if (readRegisters(UB4_REG_OFFSET_USER4, 1, &reg2) < 0) {
		return -1;
	}
	reg2 = (reg2 & 0xF0) | ((gyrZoffset & 0x0F00) >> 8);
	if (writeRegister(UB4_REG_OFFSET_USER3, reg1) < 0) {
		return -2;
	}
	if (writeRegister(UB4_REG_OFFSET_USER4, reg2) < 0) {
		return -2;
	}
	setBank(0);
	return 1;
}

int ICM42688::setUIFilterBlock(UIFiltOrd gyroUIFiltOrder, UIFiltOrd accelUIFiltOrder) {
	return 1;
}

int ICM42688::setGyroNotchFilter(float gyroNFfreq_x, float gyroNFfreq_y, float gyroNFfreq_z, GyroNFBWsel gyro_nf_bw) {
	setBank(3);
	// get clock div
	uint8_t reg;
	if (readRegisters(UB0_REG_GYRO_CONFIG0, 1, &reg) < 0) {
		return -1;
	}
	uint8_t clkdiv = reg & 0x3F;
	setBank(1);
	uint16_t nf_coswz;
	//uint8_t nf_coswz_sel = 0;
	uint8_t     gyro_nf_coswz_low[3] = {0};
	uint8_t     buff                 = 0;
	float       Fdrv                 = 19'200 / (clkdiv * 10.0f);          // in kHz  (19.2MHz = 19200 kHz)
	const float fdesired[3] = {gyroNFfreq_x, gyroNFfreq_y, gyroNFfreq_z};  // in kHz - fesdeired between 1kz and 3 kHz
	// float coswz = 0;
	for (size_t ii = 0; ii < 3; ii++) {
		float coswz = cos(2 * PI * fdesired[ii] / Fdrv);
		if (coswz <= 0.875) {
			nf_coswz              = (uint16_t)round(coswz * 256);
			gyro_nf_coswz_low[ii] = (uint8_t)(nf_coswz & 0x00FF);    //take lower part
			buff = buff & (((nf_coswz & 0xFF00) >> 8) << ii);        //take upper part and concatenate in the buffer
		} else {
			buff = buff & (1 << (3 + ii));                           //nf_coswz_sel =  nf_coswz_sel & (1<<(3+ii));
			if (coswz > 0.875) {
				nf_coswz              = (uint16_t)round(8 * (1 - coswz) * 256);
				gyro_nf_coswz_low[ii] = (uint8_t)(nf_coswz & 0x00FF);  //take lower part
				buff = buff & (((nf_coswz & 0xFF00) >> 8) << ii);      //take upper part and concatenate in the buffer
			} else if (coswz < -0.875) {
				nf_coswz              = (uint16_t)round(-8 * (1 - coswz) * 256);
				gyro_nf_coswz_low[ii] = (uint8_t)(nf_coswz & 0x00FF);  //take lower part
				buff = buff & (((nf_coswz & 0xFF00) >> 8) << ii);      //take upper part and concatenate in the buffer}
			}
		}
	}
	// write to the Registers
	if (writeRegister(UB1_REG_GYRO_CONFIG_STATIC6, gyro_nf_coswz_low[0]) < 0) {
		return -2;
	}
	if (writeRegister(UB1_REG_GYRO_CONFIG_STATIC7, gyro_nf_coswz_low[1]) < 0) {
		return -2;
	}
	if (writeRegister(UB1_REG_GYRO_CONFIG_STATIC8, gyro_nf_coswz_low[2]) < 0) {
		return -2;
	}
	if (writeRegister(UB1_REG_GYRO_CONFIG_STATIC9, buff) < 0) {
		return -2;
	}
	if (writeRegister(UB1_REG_GYRO_CONFIG_STATIC10, gyro_nf_bw) < 0) {
		return -2;
	}
	//Set Bank 0 to allow data measurements
	setBank(0);
	return 1;
}

/* for testing only*/
int ICM42688::testingFunction() {
	return 1;
}

/* Get Resolution FullScale */
float ICM42688::getAccelRes() {  // read  ACCEL_CONFIG0 and get ACCEL_FS_SEL value
	int   currentAccFS = getAccelFS();
	float accRes;
	switch (currentAccFS) {
	case ICM42688::AccelFS::gpm2:
		accRes = 16.0f / (32768.0f);
		break;
	case ICM42688::AccelFS::gpm4:
		accRes = 4.0f / (32768.0f);
		break;
	case ICM42688::AccelFS::gpm8:
		accRes = 8.0f / (32768.0f);
		break;
	case ICM42688::AccelFS::gpm16:
		accRes = 16.0f / (32768.0f);
		break;
	}
	return accRes;
}

/* Get Resolution FullScale */
float ICM42688::getGyroRes() {
	int   currentGyroFS = getGyroFS();
	float gyroRes;
	switch (currentGyroFS) {
	case ICM42688::GyroFS::dps2000:
		gyroRes = 2000.0f / 32768.0f;
		break;
	case ICM42688::GyroFS::dps1000:
		gyroRes = 1000.0f / 32768.0f;
		break;
	case ICM42688::GyroFS::dps500:
		gyroRes = 500.0f / 32768.0f;
		break;
	case ICM42688::GyroFS::dps250:
		gyroRes = 250.0f / 32768.0f;
		break;
	case ICM42688::GyroFS::dps125:
		gyroRes = 125.0f / 32768.0f;
		break;
	case ICM42688::GyroFS::dps62_5:
		gyroRes = 62.5f / 32768.0f;
		break;
	case ICM42688::GyroFS::dps31_25:
		gyroRes = 31.25f / 32768.0f;
		break;
	case ICM42688::GyroFS::dps15_625:
		gyroRes = 15.625f / 32768.0f;
		break;
	}
	return gyroRes;
}

/* Self Test*/
int ICM42688::selfTest() {
	return 1;
}

/*//#TODO
//High priority
raw data reading          <-- Validated
Offset Bias compute       <-- Validated
Offset Bias set           <-- Validated
get Full scale resolution <-- To be tested
Notch Filter              <-- To be tested
AAF Filter                <-- To be developed
UI Filter Block           <-- To be developed
Self test                 <-- To be developed



//Low priority
Read INT_STATUS          <-- get info if data are available

ApexStatus   => INT_STATUS2 and INT_STATUS3
8.1 APEX ODR SUPPORT
8.2 DMP POWER SAVE MODE
8.3 PEDOMETER PROGRAMMING
8.4 TILT DETECTION PROGRAMMING
8.5 RAISE TO WAKE/SLEEP PROGRAMMING
8.6 TAP DETECTION PROGRAMMING
8.7 WAKE ON MOTION PROGRAMMING
8.8 SIGNIFICANT MOTION DETECTION PROGRAMMING  p47
*/
