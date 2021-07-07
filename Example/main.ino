//Bosch Sensortec bme280 barometric pressure, temperature and humidity sensor
//example provided for arduino
#include "bme280.h"

uint8_t deviceAddress;        //device i2c address (which will be detected automatically using bme280_init function)
sensors_t sensorsData;        //struct container consisting of temperature, pressure, humidity and altitude; needed to get full set of data

void setup() {
  Serial.begin(9600);

  if ((deviceAddress = bme280_init()) == NONE_DETECTED)   //bme_init() initializes the i2c interface, detects up to two devices connected with their addresses and configures them in default settings
  {
    Serial.println("Check connected BME280 chip or wiring.");
    while (1)
      ;
  }

  //next line is not mandatory. using bme280_set, you can manipulate all the bme280 settings using one function
  //bme280_set(deviceAddress, MODE_NORMAL, OVERSAMPLING_4X, OVERSAMPLING_16X, OVERSAMPLING_4X, T_STANDBY_250MS, FILTER_16X);
}

void loop() {
  sensorsData = bme280_get_all(deviceAddress);

  float altitudeHypsometric = bme280_calculate_altitude_hypsometric(deviceAddress, sensorsData.pressure, sensorsData.temperature);        //second method for a more precise altitude calculation using temperature

  Serial.print("Temperature = ");
  Serial.print(sensorsData.temperature, 2);
  Serial.println(" °C");

  Serial.print("Pressure = ");
  Serial.print(sensorsData.pressure);
  Serial.println(" Pa");

  Serial.print("Humidity = ");
  Serial.print(sensorsData.humidity);
  Serial.println(" %");


  Serial.print("Altitude (quick) = ");
  Serial.print(sensorsData.altitude);
  Serial.println(" m");

  Serial.print("Altitude (hypsometric) = ");
  Serial.print(altitudeHypsometric, 2);
  Serial.println(" m\n");

  delay(2000);
}
