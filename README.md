# BME280 Barometric Pressure, Temperature  and Humidity Sensor C Driver
* version 1.0
* Reza Ebrahimi

BME280 is a combined digital humidity, pressure and temperature sensor based on proven sensing principles. Its small dimensions and its low power consumption allow the implementation in battery driven devices such as handsets, GPS modules or watches. This library is written in **portable C**, and is **MCU independent**. In order to implement it to your MCU of choice, you need to manipulate functions inside bme280_low_level.c file (I2C configurations, delay function and math operations) and leave other files as they are.

BME280 C library abstracts away the internals of the hardware, using high level functions (description below). BME280 is capable of interfacing with SPI and I2C; Only I2C is implemented in this library.

This library is based on the 'BME280 Combined Humidity and Pressure Sensor Datasheet' V. 1.6.

## GENERAL INFORMATION

BME280 consists of three sensors on one module: temperature, barometric pressure and humidity. The barometric pressure value needs to be calibrated and compensated using temperature data, so a reading of temperature must be done prior to a pressure reading.

The BME280 has three modes of operation (Normal for continuous reading, Forced for one time reading and Sleep for lowest power consumption), an optional IIR filter (to omit short term fluctuations in pressure, like slamming a door) for noisy environments, optional timing in normal mode (continuous reading) and optional oversampling.

It can also be used as an altimeter based on barometric pressure. Two mathematical functions are provided for this purpose.

An added feature is automatic detection of BME280 sensors on I2C bus and their addresses.

## HOW TO USE

As mentioned earlier, first thing to do is implementing low level settings inside bme280_low_level.c which consists of I2C settings, a delay function and math calculations for altitude. In order to use, the first function to call is bme280_init(), which initializes I2C interface, detects BME280 sensors and their addresses, soft resets and loads them with default settings, reads the non volatile memory of BME280 and stores their data for temperature, pressure  and humidity value compensations, and returns the detected addresses.

In the next step, BME280 settings could be changed using the bme280_set(...) function, as well as mode of operation using only bme280_mode(...). Both are optional and not mandatory for a sensor reading, because the module is loaded with default settings through bme280_init() function.
A full sensors reading could be done with:

**sensors_t bme280_get_all(uint8_t deviceAddress)**

The return value will be of custom type struct sensors_t, which consists of temperature, pressure, humidity and altitude. Device address is detected beforehand, using bmp280_init(). There are individual functions provided for custom readings and custom code.


