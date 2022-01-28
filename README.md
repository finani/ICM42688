# ICM42688
Arduino library for communicating with the [ICM42688](https://invensense.tdk.com/products/motion-tracking/6-axis/icm-42688-p/) six-axis Inertial Measurement Units (IMU).

# Description
The InvenSense ICM42688 supports I2C, up to 400 kHz, and SPI communication, up to 1 MHz for register setup and 8 MHz for data reading. The following selectable full scale sensor ranges are available:

| Gyroscope Full Scale Range | Accelerometer Full Scale Range |
| --- | --- |
| +/- 250 (deg/s)  | +/- 2 (g)  |
| +/- 500 (deg/s)  | +/- 4 (g)  |
| +/- 1000 (deg/s) | +/- 8 (g)  |
| +/- 2000 (deg/s) | +/- 16 (g) |

The ICM42688 samples the gyroscopes, and accelerometers with 16 bit analog to digital converters. It also features programmable digital filters, a precision clock, an embedded temperature sensor, programmable interrupts (including wake on motion), and a 512 byte FIFO buffer.

# Usage
This library supports both I2C and SPI commmunication with the ICM42688.

## Installation
Simply clone or download this library into your Arduino/libraries folder.

## Function Description
This library supports both I2C and SPI communication with the ICM42688. The *ICM42688* object declaration is overloaded with different declarations for I2C and SPI communication. All other functions remain the same. Additionally, a derived class, *ICM42688FIFO*, is included, which provides FIFO setup and data collection functionality in addition to all of the functionality included in the base *ICM42688* class.

## ICM42688 Class

### I2C Object Declaration

**ICM42688(TwoWire &bus,uint8_t address)**
An ICM42688 object should be declared, specifying the I2C bus and ICM42688 I2C address. The ICM42688 I2C address will be 0x68 if the AD0 pin is grounded or 0x69 if the AD0 pin is pulled high. For example, the following code declares an ICM42688 object called *IMU* with an ICM42688 sensor located on I2C bus 0 with a sensor address of 0x68 (AD0 grounded).

```C++
ICM42688 IMU(Wire,0x68);
```

### SPI Object Declaratioon

**ICM42688(SPIClass &bus,uint8_t csPin)**
An ICM42688 object should be declared, specifying the SPI bus and chip select pin used. Multiple ICM42688 or other SPI objects could be used on the same SPI bus, each with their own chip select pin. The chip select pin can be any available digital pin. For example, the following code declares an ICM42688 object called *IMU* with an ICM42688 sensor located on SPI bus 0 with chip select pin 10.

```C++
ICM42688 IMU(SPI,10);
```

### Common Setup Functions
The following functions are used to setup the ICM42688 sensor. These should be called once before data collection, typically this is done in the Arduino *void setup()* function. The *begin* function should always be used. Optionally, the *setAccelRange* and *setGyroRange*, *setDlpfBandwidth*, and *setSrd* functions can be used to set the accelerometer and gyroscope full scale ranges, DLPF bandwidth, and SRD to values other than default. The *enableDataReadyInterrupt* and *disableDataReadyInterrupt* control whether the ICM42688 generates an interrupt on data ready. The *enableWakeOnMotion* puts the ICM42688 into a low power mode and enables an interrupt when motion detected is above a given threshold. Finally, *enableFifo* sets up and enables the FIFO buffer. These functions are described in detail, below.

**int begin()**
This should be called in your setup function. It initializes communication with the ICM42688, sets up the sensor for reading data, and estimates the gyro bias, which is removed from the sensor data. This function returns a positive value on a successful initialization and returns a negative value on an unsuccesful initialization. If unsuccessful, please check your wiring or try resetting power to the sensor. The following is an example of setting up the ICM42688.

```C++
int status;
status = IMU.begin();
```

#### Configuration Functions

**(optional) int setAccelRange(AccelRange range)**
This function sets the accelerometer full scale range to the given  value. By default, if this function is not called, a full scale range of +/- 16 g will be used. The enumerated accelerometer full scale ranges are:

| Accelerometer Name | Accelerometer Full Scale Range |
| ------------------ | ------------------------------ |
| ACCEL_RANGE_2G     | +/- 2 (g)                      |
| ACCEL_RANGE_4G     | +/- 4 (g)                      |
| ACCEL_RANGE_8G     | +/- 8 (g)                      |
| ACCEL_RANGE_16G    | +/- 16 (g)                     |

This function returns a positive value on success and a negative value on failure. Please see the *Advanced_I2C example*. The following is an example of selecting an accelerometer full scale range of +/- 8g.

```C++
status = IMU.setAccelRange(ICM42688::ACCEL_RANGE_8G);
```

**(optional) int setGyroRange(GyroRange range)**
This function sets the gyroscope full scale range to the given  value. By default, if this function is not called, a full scale range of +/- 2000 deg/s will be used. The enumerated gyroscope full scale ranges are:

| Gyroscope Name     | Gyroscope Full Scale Range |
| ------------------ | -------------------------- |
| GYRO_RANGE_250DPS  | +/- 250 (deg/s)            |
| GYRO_RANGE_500DPS  | +/- 500 (deg/s)            |
| GYRO_RANGE_1000DPS | +/- 1000 (deg/s)           |
| GYRO_RANGE_2000DPS | +/- 2000 (deg/s)           |

This function returns a positive value on success and a negative value on failure. Please see the *Advanced_I2C example*. The following is an example of selecting an gyroscope full scale range of +/- 250 deg/s.

```C++
status = IMU.setGyroRange(ICM42688::GYRO_RANGE_250DPS);
```

**(optional) int setFilters(bool gyroFilters, bool accFilters)**
This is an optional function to set the programmable Filters (Notch Filter, Anti-Alias Filter, UI Filter Block). By default, All filters are turn on. The following figure shows a block diagram of the signal path for ICM42688:

<img src="https://github.com/finani/ICM42688/blob/master/extras/signal_path.png" alt="Signal Path" width="800">

This function returns a positive value on success and a negative value on failure. The following is an example of turning the filters on.

```C++
status = IMU.setFilters(true, true);
```

**(optional) int enableDataReadyInterrupt()**
An interrupt is tied to the data output rate. The ICM42688 *INT* pin will issue a 50us pulse when data is ready. This is extremely useful for using interrupts to clock data collection that should occur at a regular interval. Please see the *Interrupt_SPI example*. This function enables this interrupt, which will occur at a frequency given by the SRD. This function returns a positive value on success and a negative value on failure. The following is an example of enabling the data ready interrupt.

```C++
status = IMU.enableDataReadyInterrupt();
```

**(optional) int disableDataReadyInterrupt()**
This function disables the data ready interrupt, described above. This function returns a positive value on success and a negative value on failure. The following is an example of disabling the data ready interrupt.

```C++
status = IMU.disableDataReadyInterrupt();
```

**(optional) uin8_t isInterrupted()**
This function read the data ready interrupt register. This function returns true when it is interrupted. The following is an example of read the data ready interrupt register.

```C++
status = IMU.isInterrupted();
```

#### Calibration Functions

**(optional) int calibrateGyro()**
The gyro bias is automatically estimated during the *begin()* function and removed from sensor measurements. This function will re-estimate the gyro bias and remove the new bias from future sensor measurements. The sensor should be stationary during this process. This function returns a positive value on success and a negative value on failure. The following is an example of estimating new gyro biases.

```C++
status = IMU.calibrateGyro();
```

**(optional) float getGyroBiasX_rads()**
This function returns the current gyro bias in the X direction in units of rad/s.

```C++
float gxb;
gxb = IMU.getGyroBiasX_rads();
```

**(optional) float getGyroBiasY_rads()**
This function returns the current gyro bias in the Y direction in units of rad/s.

```C++
float gyb;
gyb = IMU.getGyroBiasY_rads();
```

**(optional) float getGyroBiasZ_rads()**
This function returns the current gyro bias in the Z direction in units of rad/s.

```C++
float gzb;
gzb = IMU.getGyroBiasZ_rads();
```

**(optional) void setGyroBiasX_rads(float bias)**
This function sets the gyro bias being used in the X direction to the input value in units of rad/s.

```C++
float gxb = 0.001; // gyro bias of 0.001 rad/s
IMU.setGyroBiasX_rads(gxb);
```

**(optional) void setGyroBiasY_rads(float bias)**
This function sets the gyro bias being used in the Y direction to the input value in units of rad/s.

```C++
float gyb = 0.001; // gyro bias of 0.001 rad/s
IMU.setGyroBiasY_rads(gyb);
```

**(optional) void setGyroBiasZ_rads(float bias)**
This function sets the gyro bias being used in the Z direction to the input value in units of rad/s.

```C++
float gzb = 0.001; // gyro bias of 0.001 rad/s
IMU.setGyroBiasZ_rads(gzb);
```

**(optional) int calibrateAccel()**
This function will estimate the bias and scale factor needed to calibrate the accelerometers. This function works one axis at a time and needs to be run for all 6 sensor orientations. After it has collected enough sensor data, it will estimate the bias and scale factor for all three accelerometer channels and apply these corrections to the measured data. Accelerometer calibration only needs to be performed once on the IMU, the get and set functions detailed below can be used to retrieve the estimated bias and scale factors and use them during future power cycles or operations with the IMU. This function returns a positive value on success and a negative value on failure.

```C++
status = IMU.calibrateAccel();
```

**(optional) float getAccelBiasX_mss()**
This function returns the current accelerometer bias in the X direction in units of m/s/s.

```C++
float axb;
axb = IMU.getAccelBiasX_mss();
```

**(optional) float getAccelScaleFactorX()**
This function returns the current accelerometer scale factor in the X direction.

```C++
float axs;
axs = IMU.getAccelScaleFactorX();
```

**(optional) float getAccelBiasY_mss()**
This function returns the current accelerometer bias in the Y direction in units of m/s/s.

```C++
float ayb;
ayb = IMU.getAccelBiasY_mss();
```

**(optional) float getAccelScaleFactorY()**
This function returns the current accelerometer scale factor in the Y direction.

```C++
float ays;
ays = IMU.getAccelScaleFactorY();
```

**(optional) float getAccelBiasZ_mss()**
This function returns the current accelerometer bias in the Z direction in units of m/s/s.

```C++
float azb;
azb = IMU.getAccelBiasZ_mss();
```

**(optional) float getAccelScaleFactorZ()**
This function returns the current accelerometer scale factor in the Z direction.

```C++
float azs;
azs = IMU.getAccelScaleFactorZ();
```

**(optional) void setAccelCalX(float bias,float scaleFactor)**
This function sets the accelerometer bias (m/s/s) and scale factor being used in the X direction to the input values.

```C++
float axb = 0.01; // accel bias of 0.01 m/s/s
float axs = 0.97; // accel scale factor of 0.97
IMU.setAccelCalX(axb,axs);
```

**(optional) void setAccelCalY(float bias,float scaleFactor)**
This function sets the accelerometer bias (m/s/s) and scale factor being used in the Y direction to the input values.

```C++
float ayb = 0.01; // accel bias of 0.01 m/s/s
float ays = 0.97; // accel scale factor of 0.97
IMU.setAccelCalY(ayb,ays);
```

**(optional) void setAccelCalZ(float bias,float scaleFactor)**
This function sets the accelerometer bias (m/s/s) and scale factor being used in the Z direction to the input values.

```C++
float azb = 0.01; // accel bias of 0.01 m/s/s
float azs = 0.97; // accel scale factor of 0.97
IMU.setAccelCalZ(azb,azs);
```

#### Wake on Motion Setup

**(optional) int enableWakeOnMotion(float womThresh_mg,LpAccelOdr odr)**
This function enables the MPU-9250 wake on motion interrupt functionality. It places the MPU-9250 into a low power state, with the MPU-9250 waking up at an interval determined by the Low Power Accelerometer Output Data Rate. If the accelerometer detects motion in excess of the threshold given, it generates a 50us pulse from the MPU-9250 INT pin. The following enumerated Low Power Accelerometer Output Data Rates are supported:

| LpAccelOdr Name      | Output Data Rate |
| ------------------   | ---------------- |
| LP_ACCEL_ODR_0_24HZ  | 0.24 Hz          |
| LP_ACCEL_ODR_0_49HZ  | 0.49 Hz          |
| LP_ACCEL_ODR_0_98HZ  | 0.98 Hz          |
| LP_ACCEL_ODR_1_95HZ  | 1.95 Hz          |
| LP_ACCEL_ODR_3_91HZ  | 3.91 Hz          |
| LP_ACCEL_ODR_7_81HZ  | 7.81 Hz          |
| LP_ACCEL_ODR_15_63HZ | 15.63 Hz         |
| LP_ACCEL_ODR_31_25HZ | 31.25 Hz         |
| LP_ACCEL_ODR_62_50HZ | 62.50 Hz         |
| LP_ACCEL_ODR_125HZ   | 125 Hz           |
| LP_ACCEL_ODR_250HZ   | 250 Hz           |
| LP_ACCEL_ODR_500HZ   | 500 Hz           |

The motion threshold is given as a float value between 0 and 1020 mg mapped, which is internally mapped to a single byte, 0-255 value. This function returns a positive value on success and a negative value on failure. Please see the *WOM_I2C example*. The following is an example of enabling the wake on motion with a 400 mg threshold and a ODR of 31.25 Hz.

```C++
status = IMU.enableWakeOnMotion(400,MPU9250::LP_ACCEL_ODR_31_25HZ);
```

**int setUseSPIHS(bool useSPIHS)** set SPI Mode. This function returns a positive value on success and a negative value on failure. (It always returns a positive value because It try to change member variable.)

```C++
IMU.setUseSPIHS(bool useSPIHS);
```

### Common Data Collection Functions
The functions below are used to collect data from the ICM42688 sensor. Data is returned scaled to engineering units and transformed to a [common axis system](#sensor-orientation).

#### Real-Time Data Collection
**int readSensor()** reads the sensor and stores the newest data in a buffer, it should be called every time you would like to retrieve data from the sensor. This function returns a positive value on success and a negative value on failure.

```C++
IMU.readSensor();
```

**int readAcc(double* acc)** reads the accelerometer and stores the newest data in acc, it should be called every time you would like to retrieve data from the accelerometer. This function returns a positive value on success and a negative value on failure.

```C++
IMU.readAcc(double* acc);
```

**int readGyro(double* gyro)** reads the gyroscope and stores the newest data in gyro, it should be called every time you would like to retrieve data from the gyroscope. This function returns a positive value on success and a negative value on failure.

```C++
IMU.readGyro(double* gyro);
```

**int readAccGyro(double* accGyro)** reads the accelerometer and the gyroscope, and stores the newest data in accGyro, it should be called every time you would like to retrieve data from the accelerometer and the gyroscope. This function returns a positive value on success and a negative value on failure.

```C++
IMU.readAccGyro(double* accGyro);
```

**float getAccelX_mss()** gets the accelerometer value from the data buffer in the X direction and returns it in units of m/s/s.

```C++
float ax;
ax = IMU.getAccelX_mss();
```

**float getAccelY_mss()** gets the accelerometer value from the data buffer in the Y direction and returns it in units of m/s/s.

```C++
float ay;
ay = IMU.getAccelY_mss();
```

**float getAccelZ_mss()** gets the accelerometer value from the data buffer in the Z direction and returns it in units of m/s/s.

```C++
float az;
az = IMU.getAccelZ_mss();
```

**float getGyroX_rads()** gets the gyroscope value from the data buffer in the X direction and returns it in units of rad/s.

```C++
float gx;
gx = IMU.getGyroX_rads();
```

**float getGyroY_rads()** gets the gyroscope value from the data buffer in the Y direction and returns it in units of rad/s.

```C++
float gy;
gy = IMU.getGyroY_rads();
```

**float getGyroZ_rads()** gets the gyroscope value from the data buffer in the Z direction and returns it in units of rad/s.

```C++
float gz;
gz = IMU.getGyroZ_rads();
```

**float getTemperature_C()** gets the die temperature value from the data buffer and returns it in units of C.

```C++
float temperature;
temperature = IMU.getTemperature_C();
```

## ICM42688FIFO Class
The *ICM42688FIFO* derived class extends the functionality provided by the *ICM42688* base class by providing support for setting up and reading the ICM42688FIFO buffer. All of the functions described above, as part of the *ICM42688* class are also available to the *ICM42688FIFO* class.

### I2C Object Declaration

**ICM42688FIFO(TwoWire &bus,uint8_t address)**
An ICM42688FIFO object should be declared, specifying the I2C bus and ICM42688 I2C address. The ICM42688 I2C address will be 0x68 if the AD0 pin is grounded or 0x69 if the AD0 pin is pulled high. For example, the following code declares an ICM42688FIFO object called *IMU* with an ICM42688 sensor located on I2C bus 0 with a sensor address of 0x68 (AD0 grounded).

```C++
ICM42688FIFO IMU(Wire,0x68);
```

### SPI Object Declaratioon

**ICM42688FIFO(SPIClass &bus,uint8_t csPin)**
An ICM42688FIFO object should be declared, specifying the SPI bus and chip select pin used. Multiple ICM42688 or other SPI objects could be used on the same SPI bus, each with their own chip select pin. The chip select pin can be any available digital pin. For example, the following code declares an ICM42688FIFO object called *IMU* with an ICM42688 sensor located on SPI bus 0 with chip select pin 10.

```C++
ICM42688FIFO IMU(SPI,10);
```

### FIFO Setup
**(optional) int enableFifo(bool accel,bool gyro,bool temp)**
This function configures and enables the ICM42688 FIFO buffer. This 512 byte buffer samples data at the data output rate set by the SRD and enables the microcontroller to bulk read the data, reducing microcontroller workload for certain applications. It is configured with a set of boolean values describing which data to buffer in the FIFO: accelerometer, gyroscope, or temperature. The accelerometer and gyroscope data each take 6 bytes of space per sample and the temperature 2 bytes. It's important to select only the data sources desired to ensure that the FIFO does not overrun between reading it. For example, enabling all of the data sources would take 21 bytes per sample allowing the FIFO to hold only 24 samples before overflowing. If only the accelerometer data is needed, this increases to 85 samples before overflowing. This function returns a positive value on success and a negative value on failure. Please see the *FIFO_SPI example*. The following is an example of enabling the FIFO to buffer accelerometer and gyroscope data.

```C++
status = IMU.enableFifo(true,true,false,false);
```

### FIFO Data Collection
**int readFifo()** reads the FIFO buffer from the ICM42688, parses it and stores the data in buffers on the microcontroller. It should be called every time you would like to retrieve data from the FIFO buffer. This function returns a positive value on success and a negative value on failure.

```C++
IMU.readFifo();
```

**void getFifoAccelX_mss(size_t *size,float* data)** gets the accelerometer value from the data buffer in the X direction and returns it in units of m/s/s. The data is returned as an array along with the number of elements within that array. Ensure that the buffer you are transfering to has enough capacity to store the data.

```C++
float ax[100];
size_t samples;
IMU.getFifoAccelX_mss(&samples,ax);
```

**void getFifoAccelY_mss(size_t *size,float* data)** gets the accelerometer value from the data buffer in the Y direction and returns it in units of m/s/s. The data is returned as an array along with the number of elements within that array. Ensure that the buffer you are transfering to has enough capacity to store the data.

```C++
float ay[100];
size_t samples;
IMU.getFifoAccelY_mss(&samples,ay);
```

**void getFifoAccelZ_mss(size_t *size,float* data)** gets the accelerometer value from the data buffer in the Z direction and returns it in units of m/s/s. The data is returned as an array along with the number of elements within that array. Ensure that the buffer you are transfering to has enough capacity to store the data.

```C++
float az[100];
size_t samples;
IMU.getFifoAccelZ_mss(&samples,az);
```

**void getFifoGyroX_rads(size_t *size,float* data)** gets the gyroscope value from the data buffer in the X direction and returns it in units of rad/s. The data is returned as an array along with the number of elements within that array. Ensure that the buffer you are transfering to has enough capacity to store the data.

```C++
float gx[100];
size_t samples;
IMU.getFifoGyroX_rads(&samples,gx);
```

**void getFifoGyroY_rads(size_t *size,float* data)** gets the gyroscope value from the data buffer in the Y direction and returns it in units of rad/s. The data is returned as an array along with the number of elements within that array. Ensure that the buffer you are transfering to has enough capacity to store the data.

```C++
float gy[100];
size_t samples;
IMU.getFifoGyroY_rads(&samples,gy);
```

**void getFifoGyroZ_rads(size_t *size,float* data)** gets the gyroscope value from the data buffer in the Z direction and returns it in units of rad/s. The data is returned as an array along with the number of elements within that array. Ensure that the buffer you are transfering to has enough capacity to store the data.

```C++
float gz[100];
size_t samples;
IMU.getFifoGyroZ_rads(&samples,gx);
```

**void getFifoTemperature_C(size_t *size,float* data)** gets the die temperature value from the data buffer and returns it in units of C. The data is returned as an array along with the number of elements within that array. Ensure that the buffer you are transfering to has enough capacity to store the data.

```C++
float temp[100];
size_t samples;
IMU.getFifoTemperature_C(&samples,temp);
```

## <a name="sensor-orientation"></a>Sensor Orientation
This library transforms all data to a common axis system before it is returned. It is a right handed coordinate system with the z-axis positive down, common in aircraft dynamics.

**Caution!** This axis system is shown relative to the ICM42688 sensor. The sensor may be rotated relative to the breakout board.

## Example List

* **Basic_I2C**: demonstrates declaring an *ICM42688* object, initializing the sensor, and collecting data. I2C is used to communicate with the ICM42688 sensor.
* **Basic_SPI**: demonstrates declaring an *ICM42688* object, initializing the sensor, and collecting data. SPI is used to communicate with the ICM42688 sensor.
* **Advanced_I2C**: demonstrates a more advanced setup. In this case, the accelerometer and gyroscope full scale ranges, DLPF, and SRD are set to non-default values. I2C is used to communicate with the ICM42688 sensor.
* **Interrupt_SPI**: demonstrates having the ICM42688 sensor create an interrupt pulse when data is ready, which is used to drive data collection at the specified rate. SPI is used to communicate with the ICM42688 sensor.
* **WOM_I2C**: demonstrates setting up and using the wake on motion interrupt. I2C is used to communicate with the ICM42688 sensor.
* **FIFO_SPI**: demonstrates setting up and using the FIFO buffer. SPI is used to communicate with the ICM42688 sensor.

# Wiring and Pullups

Please refer to the [ICM42688 datasheet](https://github.com/finani/ICM42688/blob/master/extras/InvenSense-ICM-42688-P-datasheet.pdf). This library should work well for other breakout boards or embedded sensors, please refer to your vendor's pinout diagram.

## I2C

The ICM42688 pins should be connected as:
   * 3V3: this should be a 3.0V to 3.6V power source.
   * GND: ground.
   * INT: (optional) used for the interrupt output setup in *enableDataReadyInterrupt* and *enableWakeOnMotion*. Connect to interruptable pin on microcontroller.
   * SDA: connect to SDA.
   * SCL: connect to SCL.

4.7 kOhm resistors should be used as pullups on SDA and SCL, these resistors should pullup with a 3.3V source.

## SPI

The ICM42688 pins should be connected as:
   * 3V3: this should be a 3.0V to 3.6V power source.
   * GND: ground.
   * INT: (optional) used for the interrupt output setup in *enableDataReadyInterrupt* and *enableWakeOnMotion*. Connect to interruptable pin on microcontroller.
   * SDI: connect to MOSI.
   * SCK: connect to SCK.
   * SDO: connect to MISO.
   * CS: connect to chip select pin. Pin 10 was used in the code snippets in this document and the included examples, but any digital I/O pin can be used.

Some breakout boards, including the Embedded Masters breakout board, require slight modification to enable SPI. Please refer to your vendor's documentation.
