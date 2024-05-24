/*!
 * @file bme68x.h
 *
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [GDuang](yonglei.ren@dfrobot.com)
 * @version  V1.0
 * @date  2024-04-25
 * @url https://github.com/DFRobot/DFRobot_BME68x
 */

#ifndef BME68X_H_
#define BME68X_H_

/*! CPP guard */
#ifdef __cplusplus
extern "C"
{
#endif

/* Header includes */
#include "bme68x_defs.h"

/**
 * @fn bme68x_init
 * @brief This API is the entry point.
 * @n It reads the chip-id and calibration data from the sensor.
 * @param dev : Structure instance of bme68x_dev
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bme68x_init(struct bme68x_dev *dev);

/**
 * @fn bme68x_set_regs
 * @brief This API writes the given data to the register address of the sensor.
 * @param reg_addr : Register address from where the data to be written.
 * @param reg_data : Pointer to data buffer which is to be written in the sensor.
 * @param len : No of bytes of data to write..
 * @param dev : Structure instance of bme68x_dev.
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bme68x_set_regs(const uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len, struct bme68x_dev *dev);

/**
 * @fn bme68x_get_regs
 * @brief This API reads the data from the given register address of the sensor.
 * @param reg_addr : Register address from where the data to be read
 * @param reg_data : Pointer to data buffer to store the read data.
 * @param len : No of bytes of data to be read.
 * @param dev : Structure instance of bme68x_dev.
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bme68x_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, struct bme68x_dev *dev);

/**
 * @fn bme68x_soft_reset
 * @brief This API performs the soft reset of the sensor.
 * @param dev : Structure instance of bme68x_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error.
 */
int8_t bme68x_soft_reset(struct bme68x_dev *dev);

/**
 * @fn bme68x_set_sensor_mode
 * @brief This API is used to set the power mode of the sensor.
 * @param dev : Structure instance of bme68x_dev
 * @note : Pass the value to bme68x_dev.power_mode structure variable.
 * @n value	|	mode
 * @n ------|------------------
 * @n 0x00	|	BME68X_SLEEP_MODE
 * @n 0x01	|	BME68X_FORCED_MODE
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bme68x_set_sensor_mode(struct bme68x_dev *dev);

/**
 * @brief This API is used to get the power mode of the sensor.
 * @param dev : Structure instance of bme68x_dev
 * @note : bme68x_dev.power_mode structure variable hold the power mode.
 * @n
 * @n  value	|	mode
 * @n  ---------|------------------
 * @n	0x00	|	BME68X_SLEEP_MODE
 * @n	0x01	|	BME68X_FORCED_MODE
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bme68x_get_sensor_mode(struct bme68x_dev *dev);

/**
 * @brief This API is used to set the profile duration of the sensor.
 * @param dev	   : Structure instance of bme68x_dev.
 * @param duration : Duration of the measurement in ms.
 * @return Nothing
 */
void bme68x_set_profile_dur(uint16_t duration, struct bme68x_dev *dev);

/**
 * @fn bme68x_get_profile_dur
 * @brief This API is used to get the profile duration of the sensor.
 *
 * @param duration : Duration of the measurement in ms.
 * @param dev	   : Structure instance of bme68x_dev.
 * @return Nothing
 */
void bme68x_get_profile_dur(uint16_t *duration, const struct bme68x_dev *dev);

/**
 * @fn bme68x_get_sensor_data
 * @brief This API reads the pressure, temperature and humidity and gas data
 * @n from the sensor, compensates the data and store it in the bme68x_data
 * @n structure instance passed by the user.
 * @param data: Structure instance to hold the data.
 * @param dev : Structure instance of bme68x_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bme68x_get_sensor_data(struct bme68x_field_data *data, struct bme68x_dev *dev);

/**
 * @fn bme68x_set_sensor_settings
 * @brief This API is used to set the oversampling, filter and T,P,H, gas selection settings in the sensor.
 *
 * @param desired_settings : Variable used to select the settings which are to be set in the sensor.
 * @n 
 * @n 	 Macros	                   |  Functionality
 * @n ---------------------------------|----------------------------------------------
 * @n 	BME68X_OST_SEL             |    To set temperature oversampling.
 * @n 	BME68X_OSP_SEL             |    To set pressure oversampling.
 * @n 	BME68X_OSH_SEL             |    To set humidity oversampling.
 * @n 	BME68X_GAS_MEAS_SEL        |    To set gas measurement setting.
 * @n 	BME68X_FILTER_SEL          |    To set filter setting.
 * @n 	BME68X_HCNTRL_SEL          |    To set humidity control setting.
 * @n 	BME68X_RUN_GAS_SEL         |    To set run gas setting.
 * @n 	BME68X_NBCONV_SEL          |    To set NB conversion setting.
 * @n 	BME68X_GAS_SENSOR_SEL      |    To set all gas sensor related settings
 * @param dev : Structure instance of bme68x_dev.
 *
 * @note : Below are the macros to be used by the user for selecting the
 * @n desired settings. User can do OR operation of these macros for configuring multiple settings.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error.
 */
int8_t bme68x_set_sensor_settings(uint16_t desired_settings, struct bme68x_dev *dev);

/**
 * @fn bme68x_get_sensor_settings
 * @brief This API is used to get the oversampling, filter and T,P,H, gas selection
 * @n settings in the sensor.
 *
 * @param desired_settings : Variable used to select the settings which
 * @n are to be get from the sensor.
 * @param dev : Structure instance of bme68x_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error.
 */
int8_t bme68x_get_sensor_settings(uint16_t desired_settings, struct bme68x_dev *dev);

uint32_t calc_gas_resistance_low(uint16_t gas_res_adc, uint8_t gas_range, const struct bme68x_dev *dev);
uint32_t calc_gas_resistance_high(uint16_t gas_res_adc, uint8_t gas_range);
#ifdef __cplusplus
}
#endif 
#endif