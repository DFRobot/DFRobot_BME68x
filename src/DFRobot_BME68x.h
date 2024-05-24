/**
 * @file DFRobot_BME68x.h
 * @brief Defines the infrastructure of the DFRobot_BME68x class
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [GDuang](yonglei.ren@dfrobot.com)
 * @version  V1.0
 * @date  2024-04-25
 * @url https://github.com/DFRobot/DFRobot_BME68x
 */

#ifndef DFROBOT_BME68X_H
#define DFROBOT_BME68X_H

#include "Arduino.h"
#include "SPI.h"
#include "Wire.h"

#include "bsec_integration.h"

#define BME68X_SEALEVEL 1015

/**\name C standard macros */
#ifndef NULL
#ifdef __cplusplus
#define NULL   0
#else
#define NULL   ((void *) 0)
#endif
#endif

enum eBME68X_INTERFACE {
  eBME68X_INTERFACE_SPI,
  eBME68X_INTERFACE_I2C
};

typedef void (*pfStartConvert_t)(void);
typedef void (*pfUpdate_t)(void);

#define supportIAQ()        setConvertAndUpdate();

void bme68x_delay_ms(uint32_t period);
int64_t bme68x_getus(void);

void bme68x_outputReady(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature, float humidity,
                        float pressure, float raw_temperature, float raw_humidity, float gas, bsec_library_return_t bsec_status);

typedef enum {
  eBME68X_PARAM_TEMPSAMP,
  eBME68X_PARAM_HUMISAMP,
  eBME68X_PARAM_PREESAMP,
  eBME68X_PARAM_IIRSIZE
} eBME68X_param_t;
                        
class DFRobot_BME68x
{
public:
  DFRobot_BME68x(bme68x_com_fptr_t readReg, bme68x_com_fptr_t writeReg, bme68x_delay_fptr_t delayMS, eBME68X_INTERFACE interface);
  uint8_t bme68x_I2CAddr = 0;
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
   * @fn isIAQReady
   * @brief check IAQ ready
   * @return result:
   * @retval 0 :ready
   * @retval 1 :not ready
   */
  uint8_t isIAQReady(void);

  /**
   * @fn setGasHeater
   * @brief Set the target temperature of the heating layer and the heating time
   * @param heaterTemp :Target temperature of the heating layer (unit: Celsius)
   * @param heaterTime  :The duration of heating (unit: milliseconds)
   * @return result:
   * @retval true :succussful
   * @retval false :failed
   */
  bool setGasHeater(uint16_t heaterTemp, uint16_t heaterTime);

  private:
    void writeParamHelper(uint8_t reg, uint8_t dat, uint8_t addr);
};


class DFRobot_BME68x_I2C : public DFRobot_BME68x
{
  public:
    DFRobot_BME68x_I2C(uint8_t I2CAddr);

    void setConvertAndUpdate(void);
};

class DFRobot_BME68x_SPI : public DFRobot_BME68x
{
  public:
    DFRobot_BME68x_SPI(uint8_t pin_cs);

    void setConvertAndUpdate(void);
};

#endif


