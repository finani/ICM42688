# ICM42688

Arduino library for communicating with the [ICM42688](https://invensense.tdk.com/products/motion-tracking/6-axis/icm-42688-p/) six-axis Inertial Measurement Units (IMU).

## Description

The InvenSense ICM42688 supports I2C, up to 400 kHz, and SPI communication, up to 1 MHz for register setup and 24 MHz for data reading. The following selectable full scale sensor ranges are available:

| Gyroscope Full Scale Range | Accelerometer Full Scale Range |
| -------------------------- | ------------------------------ |
| +/- 15.6 (deg/s)           | -                              |
| +/- 31.2 (deg/s)           | -                              |
| +/- 62.5 (deg/s)           | -                              |
| +/- 125 (deg/s)            | -                              |
| +/- 250 (deg/s)            | +/- 2 (g)                      |
| +/- 500 (deg/s)            | +/- 4 (g)                      |
| +/- 1000 (deg/s)           | +/- 8 (g)                      |
| +/- 2000 (deg/s)           | +/- 16 (g)                     |

The ICM42688 samples the gyroscopes, and accelerometers with 16 bit analog to digital converters. It also features programmable digital filters, a precision clock, an embedded temperature sensor, programmable interrupts (including wake on motion), and a 512 byte FIFO buffer.

## Usage

This library supports both I2C and SPI communication with the ICM42688.

### Installation

Simply clone or download this library into your Arduino/libraries folder.

### Function Description

This library supports both I2C and SPI communication with the ICM42688. The *ICM42688* object declaration is overloaded with different declarations for I2C and SPI communication. All other functions remain the same. Additionally, a derived class, *ICM42688FIFO*, is included, which provides FIFO setup and data collection functionality in addition to all of the functionality included in the base *ICM42688* class.

### ICM42688 Class

#### I2C Object Declaration

**ICM42688(TwoWire &bus, uint8_t address)**
An ICM42688 object should be declared, specifying the I2C bus and ICM42688 I2C address. The ICM42688 I2C address will be 0x68 if the AD0 pin is grounded or 0x69 if the AD0 pin is pulled high. For example, the following code declares an ICM42688 object called *IMU* with an ICM42688 sensor located on I2C bus 0 with a sensor address of 0x68 (AD0 grounded).

```C++
ICM42688 IMU(Wire, 0x68);
```

**ICM42688(TwoWire &bus, uint8_t address)**
An ICM42688 object should be declared, specifying the I2C bus and ICM42688 I2C address. The ICM42688 I2C address will be 0x68 if the AD0 pin is grounded or 0x69 if the AD0 pin is pulled high. For example, the following code declares an ICM42688 object called *IMU* with an ICM42688 sensor located on I2C bus 0 with a sensor address of 0x68 (AD0 grounded). You should specify the SDA and SCL pins for your I2C connection (default for Arduino is SDA=18, SCL=19, ESP32 is SDA=21, SCL=22)

```C++
ICM42688 IMU(Wire, 0x68, _sda_pin, _scl_pin);
```

#### SPI Object Declaration

**ICM42688FIFO(SPIClass &bus, uint8_t csPin, uint32_t SPI_HS_CLK=8000000)**
An ICM42688 object should be declared, specifying the SPI bus and chip select pin used. Multiple ICM42688 or other SPI objects could be used on the same SPI bus, each with their own chip select pin. The chip select pin can be any available digital pin. For example, the following code declares an ICM42688 object called *IMU* with an ICM42688 sensor located on SPI bus 0 with chip select pin 10.

```C++
ICM42688 IMU(SPI, 10);
```

Note that the default high-speed SPI bus clock is set to 8 MHz, but the ICM 42688-p supports up to 24 MHz SPI clock. Use a faster clock for faster SPI data transfers when reading data.

#### Common Setup Functions

The following functions are used to setup the ICM42688 sensor. These should be called once before data collection, typically this is done in the Arduino `setup()` function. The `begin()` function should always be used. Optionally, the `setAccelFS` and `setGyroFS`, `setAccelODR`, and `setGyroODR` functions can be used to set the accelerometer and gyroscope full scale ranges and output data rate to values other than default. The `enableDataReadyInterrupt` and `disableDataReadyInterrupt` control whether the ICM42688 generates an interrupt on data ready. The `enableFifo` sets up and enables the FIFO buffer. These functions are described in detail, below.

**int begin()**
This should be called in your setup function. It initializes communication with the ICM42688, sets up the sensor for reading data, and estimates the gyro bias, which is removed from the sensor data. This function returns a positive value on a successful initialization and returns a negative value on an unsuccessful initialization. If unsuccessful, please check your wiring or try resetting power to the sensor. The following is an example of setting up the ICM42688.

```C++
int status = IMU.begin();
```

#### Configuration Functions

**(optional) int setAccelRange(AccelRange range)**
This function sets the accelerometer full scale range to the given  value. By default, if this function is not called, a full scale range of +/- 16 g will be used. The enumerated accelerometer full scale ranges are:

| Accelerometer Name | Accelerometer Full Scale Range |
| ------------------ | ------------------------------ |
| gpm2               | +/- 2 (g)                      |
| gpm4               | +/- 4 (g)                      |
| gpm8               | +/- 8 (g)                      |
| gpm16              | +/- 16 (g)                     |

This function returns a positive value on success and a negative value on failure. Please see the *Advanced_I2C example*. The following is an example of selecting an accelerometer full scale range of +/- 8g.

```C++
status = IMU.setAccelFS(ICM42688::gpm2);
```

**(optional) int setGyroRange(GyroRange range)**
This function sets the gyroscope full scale range to the given  value. By default, if this function is not called, a full scale range of +/- 2000 deg/s will be used. The enumerated gyroscope full scale ranges are:

| Gyroscope Name | Gyroscope Full Scale Range |
| -------------- | -------------------------- |
| dps15_625      | +/- 15.625 (deg/s)         |
| dps31_25       | +/- 31.25 (deg/s)          |
| dps62_5        | +/- 62.5 (deg/s)           |
| dps125         | +/- 125 (deg/s)            |
| dps250         | +/- 250 (deg/s)            |
| dps500         | +/- 500 (deg/s)            |
| dps1000        | +/- 1000 (deg/s)           |
| dps2000        | +/- 2000 (deg/s)           |

This function returns a positive value on success and a negative value on failure. Please see the *Advanced_I2C example*. The following is an example of selecting an gyroscope full scale range of +/- 250 deg/s.

```C++
status = IMU.setGyroFS(ICM42688::dps250);
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

**(optional) float getGyroBiasX()**
This function returns the current gyro bias in the X direction in units of deg/s.

```C++
float gxb;
gxb = IMU.getGyroBiasX();
```

**(optional) float getGyroBiasY()**
This function returns the current gyro bias in the Y direction in units of deg/s.

```C++
float gyb;
gyb = IMU.getGyroBiasY();
```

**(optional) float getGyroBiasZ()**
This function returns the current gyro bias in the Z direction in units of deg/s.

```C++
float gzb;
gzb = IMU.getGyroBiasZ();
```

**(optional) void setGyroBiasX(float bias)**
This function sets the gyro bias being used in the X direction to the input value in units of deg/s.

```C++
float gxb = 0.001; // gyro bias of 0.001 deg/s
IMU.setGyroBiasX(gxb);
```

**(optional) void setGyroBiasY(float bias)**
This function sets the gyro bias being used in the Y direction to the input value in units of deg/s.

```C++
float gyb = 0.001; // gyro bias of 0.001 deg/s
IMU.setGyroBiasY(gyb);
```

**(optional) void setGyroBiasZ(float bias)**
This function sets the gyro bias being used in the Z direction to the input value in units of deg/s.

```C++
float gzb = 0.001; // gyro bias of 0.001 deg/s
IMU.setGyroBiasZ(gzb);
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

#### Common Data Collection Functions

The functions below are used to collect data from the ICM42688 sensor. Please refer to the datasheet Section 10.1 for the orientation of the sensitive axes.

#### Real-Time Data Collection

**int getAGT()** reads the sensor and stores the newest data in a buffer, it should be called every time you would like to retrieve data from the sensor. This function returns a positive value on success and a negative value on failure.

```C++
IMU.getAGT();
```

**float accX()** gets the accelerometer value from the data buffer in the X direction and returns it in units of g's.

```C++
float ax = IMU.accX();
```

**float accY()** gets the accelerometer value from the data buffer in the Y direction and returns it in units of g's.

```C++
float ay = IMU.accY();
```

**float accZ()** gets the accelerometer value from the data buffer in the Z direction and returns it in units of g's.

```C++
float az = IMU.accZ();
```

**float gyrX()** gets the gyroscope value from the data buffer in the X direction and returns it in units of deg/s.

```C++
float gx = IMU.gyrX();
```

**float gyrY()** gets the gyroscope value from the data buffer in the Y direction and returns it in units of deg/s.

```C++
float gy = IMU.gyrY();
```

**float gyrZ()** gets the gyroscope value from the data buffer in the Z direction and returns it in units of deg/s.

```C++
float gz = IMU.gyrZ();
```

**float temp()** gets the die temperature value from the data buffer and returns it in units of C.

```C++
float temperature = IMU.temp();
```

### ICM42688FIFO Class

The *ICM42688FIFO* derived class extends the functionality provided by the *ICM42688* base class by providing support for setting up and reading the ICM42688FIFO buffer. All of the functions described above, as part of the *ICM42688* class are also available to the *ICM42688FIFO* class.

#### I2C Object Declaration (FIFO)

**ICM42688FIFO(TwoWire &bus, uint8_t address)**
An ICM42688FIFO object should be declared, specifying the I2C bus and ICM42688 I2C address. The ICM42688 I2C address will be 0x68 if the AD0 pin is grounded or 0x69 if the AD0 pin is pulled high. For example, the following code declares an ICM42688FIFO object called *IMU* with an ICM42688 sensor located on I2C bus 0 with a sensor address of 0x68 (AD0 grounded).

```C++
ICM42688FIFO IMU(Wire, 0x68);
```

#### SPI Object Declaration (FIFO)

**ICM42688FIFO(SPIClass &bus, uint8_t csPin, uint32_t SPI_HS_CLK=8000000)**
An ICM42688FIFO object should be declared, specifying the SPI bus and chip select pin used. Multiple ICM42688 or other SPI objects could be used on the same SPI bus, each with their own chip select pin. The chip select pin can be any available digital pin. For example, the following code declares an ICM42688FIFO object called *IMU* with an ICM42688 sensor located on SPI bus 0 with chip select pin 10.

```C++
ICM42688FIFO IMU(SPI, 10);
```

Note that the default high-speed SPI bus clock is set to 8 MHz, but the ICM 42688-p supports up to 24 MHz SPI clock. Use a faster clock for faster SPI data transfers when reading data.

#### FIFO Setup

**(optional) int enableFifo(bool accel,bool gyro,bool temp)**
This function configures and enables the ICM42688 FIFO buffer. This 512 byte buffer samples data at the data output rate set by the SRD and enables the micro controller to bulk read the data, reducing micro controller workload for certain applications. It is configured with a set of boolean values describing which data to buffer in the FIFO: accelerometer, gyroscope, or temperature. The accelerometer and gyroscope data each take 6 bytes of space per sample and the temperature 2 bytes. It's important to select only the data sources desired to ensure that the FIFO does not overrun between reading it. For example, enabling all of the data sources would take 21 bytes per sample allowing the FIFO to hold only 24 samples before overflowing. If only the accelerometer data is needed, this increases to 85 samples before overflowing. This function returns a positive value on success and a negative value on failure. Please see the *FIFO_SPI example*. The following is an example of enabling the FIFO to buffer accelerometer and gyroscope data.

```C++
status = IMU.enableFifo(true,true,false,false);
```

#### FIFO Data Collection

**int readFifo()** reads the FIFO buffer from the ICM42688, parses it and stores the data in buffers on the micro controller. It should be called every time you would like to retrieve data from the FIFO buffer. This function returns a positive value on success and a negative value on failure.

```C++
IMU.readFifo();
```

**void getFifoAccelX_mss(size_t *size,float* data)** gets the accelerometer value from the data buffer in the X direction and returns it in units of m/s/s. The data is returned as an array along with the number of elements within that array. Ensure that the buffer you are transferring to has enough capacity to store the data.

```C++
float ax[100];
size_t samples;
IMU.getFifoAccelX_mss(&samples,ax);
```

**void getFifoAccelY_mss(size_t *size,float* data)** gets the accelerometer value from the data buffer in the Y direction and returns it in units of m/s/s. The data is returned as an array along with the number of elements within that array. Ensure that the buffer you are transferring to has enough capacity to store the data.

```C++
float ay[100];
size_t samples;
IMU.getFifoAccelY_mss(&samples,ay);
```

**void getFifoAccelZ_mss(size_t *size,float* data)** gets the accelerometer value from the data buffer in the Z direction and returns it in units of m/s/s. The data is returned as an array along with the number of elements within that array. Ensure that the buffer you are transferring to has enough capacity to store the data.

```C++
float az[100];
size_t samples;
IMU.getFifoAccelZ_mss(&samples,az);
```

**void getFifoGyroX(size_t *size,float* data)** gets the gyroscope value from the data buffer in the X direction and returns it in units of deg/s. The data is returned as an array along with the number of elements within that array. Ensure that the buffer you are transferring to has enough capacity to store the data.

```C++
float gx[100];
size_t samples;
IMU.getFifoGyroX(&samples,gx);
```

**void getFifoGyroY(size_t *size,float* data)** gets the gyroscope value from the data buffer in the Y direction and returns it in units of deg/s. The data is returned as an array along with the number of elements within that array. Ensure that the buffer you are transferring to has enough capacity to store the data.

```C++
float gy[100];
size_t samples;
IMU.getFifoGyroY(&samples,gy);
```

**void getFifoGyroZ(size_t *size,float* data)** gets the gyroscope value from the data buffer in the Z direction and returns it in units of deg/s. The data is returned as an array along with the number of elements within that array. Ensure that the buffer you are transferring to has enough capacity to store the data.

```C++
float gz[100];
size_t samples;
IMU.getFifoGyroZ(&samples,gx);
```

**void getFifoTemperature_C(size_t *size,float* data)** gets the die temperature value from the data buffer and returns it in units of C. The data is returned as an array along with the number of elements within that array. Ensure that the buffer you are transferring to has enough capacity to store the data.

```C++
float temp[100];
size_t samples;
IMU.getFifoTemperature_C(&samples,temp);
```

## Example List

* **Basic_I2C**: demonstrates declaring an *ICM42688* object, initializing the sensor, and collecting data. I2C is used to communicate with the ICM42688 sensor.
* **Basic_SPI**: demonstrates declaring an *ICM42688* object, initializing the sensor, and collecting data. SPI is used to communicate with the ICM42688 sensor.
* **Advanced_I2C**: demonstrates a more advanced setup. In this case, the accelerometer and gyroscope full scale ranges are set to non-default values. I2C is used to communicate with the ICM42688 sensor.
* **Interrupt_SPI**: demonstrates having the ICM42688 sensor create an interrupt pulse when data is ready, which is used to drive data collection at the specified rate. SPI is used to communicate with the ICM42688 sensor.
* **FIFO_SPI**: demonstrates setting up and using the FIFO buffer. SPI is used to communicate with the ICM42688 sensor.

## Wiring and Pullups

Please refer to the [ICM42688 datasheet](https://github.com/finani/ICM42688/blob/master/extras/InvenSense-ICM-42688-P-datasheet.pdf). This library should work well for other breakout boards or embedded sensors, please refer to your vendor's pinout diagram.

### I2C

The ICM42688 pins should be connected as:

* 3V3: this should be a 3.0V to 3.6V power source.
* GND: ground.
* INT1: (optional) used for the interrupt output setup in *enableDataReadyInterrupt* and *enableWakeOnMotion*. Connect to interruptable pin on micro controller.
* SDA: connect to SDA.
* SCL: connect to SCL.

4.7 kOhm resistors should be used as pullups on SDA and SCL, these resistors should pullup with a 3.3V source.

## SPI

The ICM42688 pins should be connected as:

* 3V3: this should be a 3.0V to 3.6V power source.
* GND: ground.
* INT1: (optional) used for the interrupt output setup in *enableDataReadyInterrupt* and *enableWakeOnMotion*. Connect to interruptable pin on micro controller.
* SDI: connect to MOSI.
* SCK: connect to SCK.
* SDO: connect to MISO.
* CS: connect to chip select pin. Pin 10 was used in the code snippets in this document and the included examples, but any digital I/O pin can be used.

Some breakout boards, including the Embedded Masters breakout board, require slight modification to enable SPI. Please refer to your vendor's documentation.
