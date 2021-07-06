/*bme280 barometric pressure, temperature and humidity sensor C Driver*/
/*Reza Ebrahimi - https://github.com/ebrezadev */
/*Version 1.0*/

#include "bme280.h"

#define I2C_SPEED   400000

/*writes an array (data[]) of arbitrary size (dataLength) to I2C address (deviceAddress), starting from an internal register address (startRegisterAddress)*/
void bme280_write_array(uint8_t deviceAddress, uint8_t startRegisterAddress, uint8_t *data, uint8_t dataLength)
{
}

/*reads an array (data[]) of arbitrary size (dataLength) from I2C address (deviceAddress), starting from an internal register address (startRegisterAddress)*/
void bme280_read_array (uint8_t deviceAddress, uint8_t startRegisterAddress, uint8_t *data, uint8_t dataLength)
{
}

/*initiates the I2C peripheral and sets its speed*/
void bme280_i2c_init()        
{
}

/*a delay function for milliseconds delay*/
void delay_function (uint32_t delayMS)
{
}

/*implements a power function (used in altitude calculation)*/
float power_function (float x, float y)
{
}
