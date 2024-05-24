# DFRobot_BME68x

* [中文版](./README_CN.md)

BME68x is an integrated environmental sensor developed specifically for mobile applications and wearables 
where size and low power consumption are key requirements. Expanding Bosch Sensortec’s existing family of environmental sensors, 
the BME68x integrates for the first time individual high linearity and high accuracy sensors for gas, pressure, humidity and temperature.

![产品效果图](./resources/images/SEN0617.png) 



## Product Link(https://www.dfrobot.com)
    SKU: SEN0617
    

## Table of Contents

* [Summary](#summary)
* [Installation](#installation)
* [Methods](#methods)
* [Compatibility](#compatibility)
* [History](#history)
* [Credits](#credits)

## Summary
Provides an Arduino library for reading and interpreting Bosch BME68x data over I2C over SPI.Reads temperature, humidity, gas, IAQ(details in examples\readme), pressure and calculates altitude.


## Installation

To use this library download the zip file, uncompress it to a folder named DFRobot_BME68x. 
Download the zip file first to use this library and uncompress it to a folder named DFRobot_BME68x. 

## Methods

```C++
  /**
   * @fn begin
   * @brief begin BME68x device
   * @return result
   * @retval  non-zero : falid
   * @retval  0        : succussful
   */
  int16_t begin(void);
  /**
   * @fn update
   * @brief update all data to MCU ram
   */
  void    update(void);
  /**
   * @fn iaqUpdate
   * @brief update all data to MCU ram with IAQ (only for esp8266 now)
   *
   * @return result:
   * @retval 0 :complete
   * @retval 1 :busy
   */
  int8_t  iaqUpdate(void);
  /**
   * @fn startConvert
   * @brief start convert to get a accurate values
   */
  void  startConvert(void);
  /**
   * @fn readTemperature
   * @brief read the temperature value (unit C)
   *
   * @return temperature valu, this value has two decimal points
   */
  float readTemperature(void);
  /**
   * @fn readPressure
   * @brief read the pressure value (unit pa)
   *
   * @return pressure value, this value has two decimal points
   */
  float readPressure(void);
  /**
   * @fn readHumidity
   * @brief read the humidity value (unit %rh)
   * @return humidity value, this value has two decimal points
   */
  float readHumidity(void);
  /**
   * @fn readAltitude
   * @brief read the altitude (unit meter)
   * @return altitude value, this value has two decimal points
   */
  float readAltitude(void);
  /**
   * @fn readCalibratedAltitude
   * @brief read the Calibrated altitude (unit meter)
   *
   * @param seaLevel  normalised atmospheric pressure
   *
   * @return calibrated altitude value , this value has two decimal points
   */
  float readCalibratedAltitude(float seaLevel);
  /**
   * @fn readGasResistance
   * @brief read the gas resistance(unit ohm)
   * @return temperature value, this value has two decimal points
   */
  float readGasResistance(void);
  /**
   * @fn readSeaLevel
   * @brief read normalised atmospheric pressure (unit pa)
   * @param altitude   accurate altitude for normalising
   * @return normalised atmospheric pressure
   */
  float readSeaLevel(float altitude);
  /**
   * @fn readIAQ
   * @brief read IAQ
   * @return The result of IAQ
   */
  float readIAQ(void);
  /**
   * @fn setGasHeater
   * @brief set bme68x gas heater
   * @param temp        :your object temp
   * @param t           :time spend in milliseconds
   */
   void    setGasHeater(uint16_t temp, uint16_t t);
  /**
   * @fn isIAQReady
   * @brief check IAQ ready
   * @return result:
   * @retval 0 :ready
   * @retval 1 :not ready
   */
  uint8_t isIAQReady(void);

```

## Compatibility

MCU                | Work Well | Work Wrong | Untested  | Remarks
------------------ | :----------: | :----------: | :---------: | -----
FireBeetle-ESP32  |      √       |             |            | 
FireBeetle-ESP8266  |      √       |             |            | 
FireBeetle-BLE4.1 |       √      |             |            | 
Arduino uno |       √      |             |            | 
Arduino leonardo |      √       |             |            | 

## History

- 2024/05/06 - Version 1.0.0 released.

## Credits

Written by [GDuang](yonglei.ren@dfrobot.com), 2024. (Welcome to our [website](https://www.dfrobot.com/))
