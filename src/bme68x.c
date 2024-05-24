/**
 * @file bme68x.c
 * @brief Sensor driver for BME68x sensor.
 *
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [GDuang](yonglei.ren@dfrobot.com)
 * @version  V1.0
 * @date  2024-04-25
 * @url https://github.com/DFRobot/DFRobot_BME68x
 */

#include "bme68x.h"

/**
 * @fn get_calib_data
 * @brief This internal API is used to read the calibrated data from the sensor.
 * @n This function is used to retrieve the calibration data from the image registers of the sensor.
 * @note Registers 89h  to A1h for calibration data 1 to 24  from bit 0 to 7 
 * @note Registers E1h to F0h for calibration data 25 to 40 from bit 0 to 7
 * @param dev	:Structure instance of bme68x_dev.
 *
 * @return Result of API execution status.
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t get_calib_data(struct bme68x_dev *dev);

/**
 * @fn set_gas_config
 * @brief This internal API is used to set the gas configuration of the sensor.
 *
 * @param dev	:Structure instance of bme68x_dev.
 *
 * @return Result of API execution status.
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t set_gas_config(struct bme68x_dev *dev);

/**
 * @fn get_gas_config
 * @brief This internal API is used to get the gas configuration of the sensor.
 * @param dev	:Structure instance of bme68x_dev.
 * @return Result of API execution status.
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t get_gas_config(struct bme68x_dev *dev);

/**
 * @fn calc_heater_dur
 * @brief This internal API is used to calculate the Heat duration value.
 * @param dur	:Value of the duration to be shared.
 * @return uint8_t threshold duration after calculation.
 */
static uint8_t calc_heater_dur(uint16_t dur);

/**
 * @fn calc_temperature
 * @brief This internal API is used to calculate the temperature value.
 * @param temp_adc	:Contains the temperature ADC value .
 * @param dev	:Structure instance of bme68x_dev.
 * @return uint32_t calculated temperature.
 */
static int16_t calc_temperature(uint32_t temp_adc, struct bme68x_dev *dev);

/**
 * @fn calc_pressure
 * @brief This internal API is used to calculate the pressure value.
 * @param pres_adc	:Contains the pressure ADC value .
 * @param dev	:Structure instance of bme68x_dev.
 * @return uint32_t calculated pressure.
 */
static uint32_t calc_pressure(uint32_t pres_adc, const struct bme68x_dev *dev);

/**
 * @fn calc_humidity
 * @brief This internal API is used to calculate the humidity value.
 * @param hum_adc	:Contains the humidity ADC value.
 * @param dev	:Structure instance of bme68x_dev.
 *
 * @return uint32_t calculated humidity.
 */
static uint32_t calc_humidity(uint16_t hum_adc, const struct bme68x_dev *dev);

/**
 * @fn calc_heater_res
 * @brief This internal API is used to calculate the Heat Resistance value.
 * @param temp	:Contains the temporary value.
 * @param dev	:Structure instance of bme68x_dev.
 * @return uint8_t calculated heater resistance.
 */
static uint8_t calc_heater_res(uint16_t temp, const struct bme68x_dev *dev);

/**
 * @fn read_field_data
 * @brief This internal API is used to calculate the field data of sensor.
 *
 * @param data :Structure instance to hold the data
 * @param dev	:Structure instance of bme68x_dev.
 *
 * @return int8_t result of the field data from sensor.
 */
static int8_t read_field_data(struct bme68x_field_data *data, struct bme68x_dev *dev);

/**
 * @fn set_mem_page
 * @brief This internal API is used to set the memory page based on register address.
 *
 * @n The value of memory page
 * @n  value  | Description
 * @n --------|--------------
 * @n   0     | BME68X_PAGE0_SPI
 * @n   1     | BME68X_PAGE1_SPI
 *
 * @param reg_addr	:Contains the register address array.
 * @param dev	:Structure instance of bme68x_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t set_mem_page(uint8_t reg_addr, struct bme68x_dev *dev);

/**
 * @fn get_mem_page
 * @brief This internal API is used to get the memory page based on register address.
 * @n The value of memory page
 * @n  value  | Description
 * @n --------|--------------
 * @n   0     | BME68X_PAGE0_SPI
 * @n   1     | BME68X_PAGE1_SPI
 *
 * @param dev	:Structure instance of bme68x_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t get_mem_page(struct bme68x_dev *dev);

/**
 * @fn null_ptr_check
 * @brief This internal API is used to validate the device pointer for null conditions.
 *
 * @param dev	:Structure instance of bme68x_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t null_ptr_check(const struct bme68x_dev *dev);

/**
 * @fn boundary_check
 * @brief This internal API is used to check the boundary conditions.
 *
 * @param value	:pointer to the value.
 * @param min	:minimum value.
 * @param max	:maximum value.
 * @param dev	:Structure instance of bme68x_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t boundary_check(uint8_t *value, uint8_t min, uint8_t max, struct bme68x_dev *dev);


/* This internal API is used to read variant ID information register status */
static int8_t read_variant_id(struct bme68x_dev *dev);	
																		  

/****************** Global Function Definitions *******************************/
int8_t bme68x_init(struct bme68x_dev *dev)
{
	int8_t rslt;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);
	if (rslt == BME68X_OK) {
		/* Soft reset to restore it to default values*/
		rslt = bme68x_soft_reset(dev);
		if (rslt == BME68X_OK) {
			rslt = bme68x_get_regs(BME68X_CHIP_ID_ADDR, &dev->chip_id, 1, dev);
			if (rslt == BME68X_OK) {
				if (dev->chip_id == BME68X_CHIP_ID) {						   
					/* Read Variant ID */
					rslt = read_variant_id(dev);

					if (rslt == BME68X_OK)
					{
						/* Get the Calibration data */
						rslt = get_calib_data(dev);
					}
				} else {
					rslt = BME68X_E_DEV_NOT_FOUND;
				}
			}
		}
	}

	return rslt;
}

int8_t bme68x_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, struct bme68x_dev *dev)
{
	int8_t rslt;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);
	if (rslt == BME68X_OK) {
		if (dev->intf == BME68X_SPI_INTF) {
			/* Set the memory page */
			rslt = set_mem_page(reg_addr, dev);
			if (rslt == BME68X_OK)
				reg_addr = reg_addr | BME68X_SPI_RD_MSK;
		}
		dev->com_rslt = dev->read(dev->dev_id, reg_addr, reg_data, len);
		if (dev->com_rslt != 0)
			rslt = BME68X_E_COM_FAIL;
	}

	return rslt;
}

int8_t bme68x_set_regs(const uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len, struct bme68x_dev *dev)
{
	int8_t rslt;
	/* Length of the temporary buffer is 2*(length of register)*/
	uint8_t tmp_buff[BME68X_TMP_BUFFER_LENGTH] = { 0 };
	uint16_t index;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);
	if (rslt == BME68X_OK) {
		if ((len > 0) && (len < BME68X_TMP_BUFFER_LENGTH / 2)) {
			/* Interleave the 2 arrays */
			for (index = 0; index < len; index++) {
				if (dev->intf == BME68X_SPI_INTF) {
					/* Set the memory page */
					rslt = set_mem_page(reg_addr[index], dev);
					tmp_buff[(2 * index)] = reg_addr[index] & BME68X_SPI_WR_MSK;
				} else {
					tmp_buff[(2 * index)] = reg_addr[index];
				}
				tmp_buff[(2 * index) + 1] = reg_data[index];
			}
			/* Write the interleaved array */
			if (rslt == BME68X_OK) {
				dev->com_rslt = dev->write(dev->dev_id, tmp_buff[0], &tmp_buff[1], (2 * len) - 1);
				if (dev->com_rslt != 0)
					rslt = BME68X_E_COM_FAIL;
			}
		} else {
			rslt = BME68X_E_INVALID_LENGTH;
		}
	}

	return rslt;
}

int8_t bme68x_soft_reset(struct bme68x_dev *dev)
{
	int8_t rslt;
	uint8_t reg_addr = BME68X_SOFT_RESET_ADDR;
	/* 0xb6 is the soft reset command */
	uint8_t soft_rst_cmd = BME68X_SOFT_RESET_CMD;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);
	if (rslt == BME68X_OK) {
		if (dev->intf == BME68X_SPI_INTF)
			rslt = get_mem_page(dev);

		/* Reset the device */
		if (rslt == BME68X_OK) {
			rslt = bme68x_set_regs(&reg_addr, &soft_rst_cmd, 1, dev);
			/* Wait for 5ms */
			dev->delay_ms(BME68X_RESET_PERIOD);

			if (rslt == BME68X_OK) {
				/* After reset get the memory page */
				if (dev->intf == BME68X_SPI_INTF)
					rslt = get_mem_page(dev);
			}
		}
	}

	return rslt;
}

int8_t bme68x_set_sensor_settings(uint16_t desired_settings, struct bme68x_dev *dev)
{
										
	int8_t rslt;
	uint8_t reg_addr;
	uint8_t data = 0;
	uint8_t count = 0;
	uint8_t reg_array[BME68X_REG_BUFFER_LENGTH] = { 0 };
	uint8_t data_array[BME68X_REG_BUFFER_LENGTH] = { 0 };
	uint8_t intended_power_mode = dev->power_mode; /* Save intended power mode */

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);
	if (rslt == BME68X_OK) {
		if (desired_settings & BME68X_GAS_MEAS_SEL)
			rslt = set_gas_config(dev);

		dev->power_mode = BME68X_SLEEP_MODE;
		if (rslt == BME68X_OK)
			rslt = bme68x_set_sensor_mode(dev);
									 

		/* Selecting the filter */
		if (desired_settings & BME68X_FILTER_SEL) {
			rslt = boundary_check(&dev->tph_sett.filter, BME68X_FILTER_SIZE_0, BME68X_FILTER_SIZE_127, dev);
			reg_addr = BME68X_CONF_ODR_FILT_ADDR;

			if (rslt == BME68X_OK)
				rslt = bme68x_get_regs(reg_addr, &data, 1, dev);

			if (desired_settings & BME68X_FILTER_SEL)
				data = BME68X_SET_BITS(data, BME68X_FILTER, dev->tph_sett.filter);

			reg_array[count] = reg_addr; /* Append configuration */
			data_array[count] = data;
			count++;
												  
		}

		/* Selecting heater control for the sensor */
		if (desired_settings & BME68X_HCNTRL_SEL) {
			rslt = boundary_check(&dev->gas_sett.heatr_ctrl, BME68X_ENABLE_HEATER,
				BME68X_DISABLE_HEATER, dev);
			reg_addr = BME68X_CONF_HEAT_CTRL_ADDR;

			if (rslt == BME68X_OK)
				rslt = bme68x_get_regs(reg_addr, &data, 1, dev);
			data = BME68X_SET_BITS_POS_0(data, BME68X_HCTRL, dev->gas_sett.heatr_ctrl);

			reg_array[count] = reg_addr; /* Append configuration */
			data_array[count] = data;
			count++;
												   
		}

		/* Selecting heater T,P oversampling for the sensor */
		if (desired_settings & (BME68X_OST_SEL | BME68X_OSP_SEL)) {
			rslt = boundary_check(&dev->tph_sett.os_temp, BME68X_OS_NONE, BME68X_OS_16X, dev);
			reg_addr = BME68X_CONF_T_P_MODE_ADDR;

			if (rslt == BME68X_OK)
				rslt = bme68x_get_regs(reg_addr, &data, 1, dev);

			if (desired_settings & BME68X_OST_SEL)
				data = BME68X_SET_BITS(data, BME68X_OST, dev->tph_sett.os_temp);

			if (desired_settings & BME68X_OSP_SEL)
				data = BME68X_SET_BITS(data, BME68X_OSP, dev->tph_sett.os_pres);

			reg_array[count] = reg_addr;
			data_array[count] = data;
			count++;
															   
		}

		/* Selecting humidity oversampling for the sensor */
		if (desired_settings & BME68X_OSH_SEL) {
			rslt = boundary_check(&dev->tph_sett.os_hum, BME68X_OS_NONE, BME68X_OS_16X, dev);
			reg_addr = BME68X_CONF_OS_H_ADDR;

			if (rslt == BME68X_OK)
				rslt = bme68x_get_regs(reg_addr, &data, 1, dev);
			data = BME68X_SET_BITS_POS_0(data, BME68X_OSH, dev->tph_sett.os_hum);

			reg_array[count] = reg_addr; /* Append configuration */
			data_array[count] = data;
			count++;
												
		}

		/* Selecting the runGas and NB conversion settings for the sensor */
		if (desired_settings & (BME68X_RUN_GAS_SEL | BME68X_NBCONV_SEL)) {
			rslt = boundary_check(&dev->gas_sett.run_gas, BME68X_RUN_GAS_DISABLE,
				BME68X_RUN_GAS_ENABLE, dev);
			if (rslt == BME68X_OK) {
				/* Validate boundary conditions */
				rslt = boundary_check(&dev->gas_sett.nb_conv, BME68X_NBCONV_MIN,
					BME68X_NBCONV_MAX, dev);
			}

			reg_addr = BME68X_CONF_ODR_RUN_GAS_NBC_ADDR;

			if (rslt == BME68X_OK)
				rslt = bme68x_get_regs(reg_addr, &data, 1, dev);

			if (desired_settings & BME68X_RUN_GAS_SEL){
			  
				if (dev->variant_id == BME68X_VARIANT_GAS_HIGHV)
				{
					data = BME68X_SET_BITS(data, BME68X_RUN_GAS_HIGHV, dev->gas_sett.run_gas);
																						  
				}else{
					data = BME68X_SET_BITS(data, BME68X_RUN_GAS_LOWV, dev->gas_sett.run_gas);
																						  
				}
			}																	  																	  

			if (desired_settings & BME68X_NBCONV_SEL)
				data = BME68X_SET_BITS_POS_0(data, BME68X_NBCONV, dev->gas_sett.nb_conv);

			reg_array[count] = reg_addr; /* Append configuration */
			data_array[count] = data;
			count++;
   
		}

		if (rslt == BME68X_OK)
			rslt = bme68x_set_regs(reg_array, data_array, count, dev);
							
																															 
	
   

		/* Restore previous intended power mode */
		dev->power_mode = intended_power_mode;
								
	}

	return rslt;
}

int8_t bme68x_get_sensor_settings(uint16_t desired_settings, struct bme68x_dev *dev)
{
	int8_t rslt;
	/* starting address of the register array for burst read*/
	uint8_t reg_addr = BME68X_CONF_HEAT_CTRL_ADDR;
	uint8_t data_array[BME68X_REG_BUFFER_LENGTH] = { 0 };

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);
	if (rslt == BME68X_OK) {
		rslt = bme68x_get_regs(reg_addr, data_array, BME68X_REG_BUFFER_LENGTH, dev);

		if (rslt == BME68X_OK) {
			if (desired_settings & BME68X_GAS_MEAS_SEL)
				rslt = get_gas_config(dev);

			/* get the T,P,H ,Filter,ODR settings here */
			if (desired_settings & BME68X_FILTER_SEL)
				dev->tph_sett.filter = BME68X_GET_BITS(data_array[BME68X_REG_FILTER_INDEX],
					BME68X_FILTER);

			if (desired_settings & (BME68X_OST_SEL | BME68X_OSP_SEL)) {
				dev->tph_sett.os_temp = BME68X_GET_BITS(data_array[BME68X_REG_TEMP_INDEX], BME68X_OST);
				dev->tph_sett.os_pres = BME68X_GET_BITS(data_array[BME68X_REG_PRES_INDEX], BME68X_OSP);
			}

			if (desired_settings & BME68X_OSH_SEL)
				dev->tph_sett.os_hum = BME68X_GET_BITS_POS_0(data_array[BME68X_REG_HUM_INDEX],
					BME68X_OSH);

			/* get the gas related settings */
			if (desired_settings & BME68X_HCNTRL_SEL)
				dev->gas_sett.heatr_ctrl = BME68X_GET_BITS_POS_0(data_array[BME68X_REG_HCTRL_INDEX],
					BME68X_HCTRL);

			if (desired_settings & (BME68X_RUN_GAS_SEL | BME68X_NBCONV_SEL)) {
				dev->gas_sett.nb_conv = BME68X_GET_BITS_POS_0(data_array[BME68X_REG_NBCONV_INDEX], BME68X_NBCONV);
				if (dev->variant_id == BME68X_VARIANT_GAS_HIGHV){
					dev->gas_sett.run_gas = BME68X_GET_BITS(data_array[BME68X_REG_RUN_GAS_INDEX], BME68X_RUN_GAS_HIGHV);
				}else{
					dev->gas_sett.run_gas = BME68X_GET_BITS(data_array[BME68X_REG_RUN_GAS_INDEX], BME68X_RUN_GAS_LOWV);
				}
			}
		}
	} else {
		rslt = BME68X_E_NULL_PTR;
	}

	return rslt;
}

int8_t bme68x_set_sensor_mode(struct bme68x_dev *dev)
{
	int8_t rslt;
	uint8_t tmp_pow_mode;
	uint8_t pow_mode = 0;
	uint8_t reg_addr = BME68X_CONF_T_P_MODE_ADDR;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);
	if (rslt == BME68X_OK) {
		/* Call recursively until in sleep */
		do {
			rslt = bme68x_get_regs(BME68X_CONF_T_P_MODE_ADDR, &tmp_pow_mode, 1, dev);
			if (rslt == BME68X_OK) {
				/* Put to sleep before changing mode */
				pow_mode = (tmp_pow_mode & BME68X_MODE_MSK);

				if (pow_mode != BME68X_SLEEP_MODE) {
					tmp_pow_mode = tmp_pow_mode & (~BME68X_MODE_MSK); /* Set to sleep */
					rslt = bme68x_set_regs(&reg_addr, &tmp_pow_mode, 1, dev);
					dev->delay_ms(BME68X_POLL_PERIOD_MS);
				}
			}
		} while (pow_mode != BME68X_SLEEP_MODE);

		/* Already in sleep */
		if (dev->power_mode != BME68X_SLEEP_MODE) {
			tmp_pow_mode = (tmp_pow_mode & ~BME68X_MODE_MSK) | (dev->power_mode & BME68X_MODE_MSK);
			if (rslt == BME68X_OK)
				rslt = bme68x_set_regs(&reg_addr, &tmp_pow_mode, 1, dev);
		}
	}

	return rslt;
}

int8_t bme68x_get_sensor_mode(struct bme68x_dev *dev)
{
	int8_t rslt;
	uint8_t mode;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);
	if (rslt == BME68X_OK) {
		rslt = bme68x_get_regs(BME68X_CONF_T_P_MODE_ADDR, &mode, 1, dev);
		/* Masking the other register bit info*/
		dev->power_mode = mode & BME68X_MODE_MSK;
	}

	return rslt;
}

void bme68x_set_profile_dur(uint16_t duration, struct bme68x_dev *dev)
{
	uint32_t tph_dur; /* Calculate in us */

	/* TPH measurement duration */
	tph_dur = ((uint32_t) (dev->tph_sett.os_temp + dev->tph_sett.os_pres + dev->tph_sett.os_hum) * UINT32_C(1963));
	tph_dur += UINT32_C(477 * 4); /* TPH switching duration */
	tph_dur += UINT32_C(477 * 5); /* Gas measurement duration */
	tph_dur += UINT32_C(500); /* Get it to the closest whole number.*/
	tph_dur /= UINT32_C(1000); /* Convert to ms */

	tph_dur += UINT32_C(1); /* Wake up duration of 1ms */
	/* The remaining time should be used for heating */
	dev->gas_sett.heatr_dur = duration - (uint16_t) tph_dur;
}

void bme68x_get_profile_dur(uint16_t *duration, const struct bme68x_dev *dev)
{
	uint32_t tph_dur; /* Calculate in us */

	/* TPH measurement duration */
	tph_dur = ((uint32_t) (dev->tph_sett.os_temp + dev->tph_sett.os_pres + dev->tph_sett.os_hum) * UINT32_C(1963));
	tph_dur += UINT32_C(477 * 4); /* TPH switching duration */
	tph_dur += UINT32_C(477 * 5); /* Gas measurement duration */
	tph_dur += UINT32_C(500); /* Get it to the closest whole number.*/
	tph_dur /= UINT32_C(1000); /* Convert to ms */

	tph_dur += UINT32_C(1); /* Wake up duration of 1ms */

	*duration = (uint16_t) tph_dur;

	/* Get the gas duration only when the run gas is enabled */
	if (dev->gas_sett.run_gas) {
		/* The remaining time should be used for heating */
		*duration += dev->gas_sett.heatr_dur;
	}
}

int8_t bme68x_get_sensor_data(struct bme68x_field_data *data, struct bme68x_dev *dev)
{
	int8_t rslt;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);
	if (rslt == BME68X_OK) {
		/* Reading the sensor data in forced mode only */
		rslt = read_field_data(data, dev);
		if (rslt == BME68X_OK) {
			if (data->status & BME68X_NEW_DATA_MSK)
				dev->new_fields = 1;
			else
				dev->new_fields = 0;
		}
	}

	return rslt;
}

static int8_t get_calib_data(struct bme68x_dev *dev)
{
	int8_t rslt;
	uint8_t coeff_array[BME68X_COEFF_SIZE] = { 0 };
	uint8_t temp_var = 0; /* Temporary variable */

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);
	if (rslt == BME68X_OK) {
		rslt = bme68x_get_regs(BME68X_COEFF_ADDR1, coeff_array, BME68X_COEFF_ADDR1_LEN, dev);
		/* Append the second half in the same array */
		if (rslt == BME68X_OK)
			rslt = bme68x_get_regs(BME68X_COEFF_ADDR2, &coeff_array[BME68X_COEFF_ADDR1_LEN]
			, BME68X_COEFF_ADDR2_LEN, dev);

		/* Temperature related coefficients */
		dev->calib.par_t1 = (uint16_t) (BME68X_CONCAT_BYTES(coeff_array[BME68X_T1_MSB_REG],
			coeff_array[BME68X_T1_LSB_REG]));
		dev->calib.par_t2 = (int16_t) (BME68X_CONCAT_BYTES(coeff_array[BME68X_T2_MSB_REG],
			coeff_array[BME68X_T2_LSB_REG]));
		dev->calib.par_t3 = (int8_t) (coeff_array[BME68X_T3_REG]);

		/* Pressure related coefficients */
		dev->calib.par_p1 = (uint16_t) (BME68X_CONCAT_BYTES(coeff_array[BME68X_P1_MSB_REG],
			coeff_array[BME68X_P1_LSB_REG]));
		dev->calib.par_p2 = (int16_t) (BME68X_CONCAT_BYTES(coeff_array[BME68X_P2_MSB_REG],
			coeff_array[BME68X_P2_LSB_REG]));
		dev->calib.par_p3 = (int8_t) coeff_array[BME68X_P3_REG];
		dev->calib.par_p4 = (int16_t) (BME68X_CONCAT_BYTES(coeff_array[BME68X_P4_MSB_REG],
			coeff_array[BME68X_P4_LSB_REG]));
		dev->calib.par_p5 = (int16_t) (BME68X_CONCAT_BYTES(coeff_array[BME68X_P5_MSB_REG],
			coeff_array[BME68X_P5_LSB_REG]));
		dev->calib.par_p6 = (int8_t) (coeff_array[BME68X_P6_REG]);
		dev->calib.par_p7 = (int8_t) (coeff_array[BME68X_P7_REG]);
		dev->calib.par_p8 = (int16_t) (BME68X_CONCAT_BYTES(coeff_array[BME68X_P8_MSB_REG],
			coeff_array[BME68X_P8_LSB_REG]));
		dev->calib.par_p9 = (int16_t) (BME68X_CONCAT_BYTES(coeff_array[BME68X_P9_MSB_REG],
			coeff_array[BME68X_P9_LSB_REG]));
		dev->calib.par_p10 = (uint8_t) (coeff_array[BME68X_P10_REG]);

		/* Humidity related coefficients */
		dev->calib.par_h1 = (uint16_t) (((uint16_t) coeff_array[BME68X_H1_MSB_REG] << BME68X_HUM_REG_SHIFT_VAL)
			| (coeff_array[BME68X_H1_LSB_REG] & BME68X_BIT_H1_DATA_MSK));
		dev->calib.par_h2 = (uint16_t) (((uint16_t) coeff_array[BME68X_H2_MSB_REG] << BME68X_HUM_REG_SHIFT_VAL)
			| ((coeff_array[BME68X_H2_LSB_REG]) >> BME68X_HUM_REG_SHIFT_VAL));
		dev->calib.par_h3 = (int8_t) coeff_array[BME68X_H3_REG];
		dev->calib.par_h4 = (int8_t) coeff_array[BME68X_H4_REG];
		dev->calib.par_h5 = (int8_t) coeff_array[BME68X_H5_REG];
		dev->calib.par_h6 = (uint8_t) coeff_array[BME68X_H6_REG];
		dev->calib.par_h7 = (int8_t) coeff_array[BME68X_H7_REG];

		/* Gas heater related coefficients */
		dev->calib.par_gh1 = (int8_t) coeff_array[BME68X_GH1_REG];
		dev->calib.par_gh2 = (int16_t) (BME68X_CONCAT_BYTES(coeff_array[BME68X_GH2_MSB_REG],
			coeff_array[BME68X_GH2_LSB_REG]));
		dev->calib.par_gh3 = (int8_t) coeff_array[BME68X_GH3_REG];

		/* Other coefficients */
		if (rslt == BME68X_OK) {
			rslt = bme68x_get_regs(BME68X_ADDR_RES_HEAT_RANGE_ADDR, &temp_var, 1, dev);

			dev->calib.res_heat_range = ((temp_var & BME68X_RHRANGE_MSK) / 16);
			if (rslt == BME68X_OK) {
				rslt = bme68x_get_regs(BME68X_ADDR_RES_HEAT_VAL_ADDR, &temp_var, 1, dev);

				dev->calib.res_heat_val = (int8_t) temp_var;
				if (rslt == BME68X_OK)
					rslt = bme68x_get_regs(BME68X_ADDR_RANGE_SW_ERR_ADDR, &temp_var, 1, dev);
			}
		}
		dev->calib.range_sw_err = ((int8_t) temp_var & (int8_t) BME68X_RSERROR_MSK) / 16;
	}

	return rslt;
}

static int8_t set_gas_config(struct bme68x_dev *dev)
{
	int8_t rslt;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);
	if (rslt == BME68X_OK) {

		uint8_t reg_addr[2] = {0};
		uint8_t reg_data[2] = {0};

																		
		if (dev->power_mode == BME68X_FORCED_MODE) {
			reg_addr[0] = BME68X_RES_HEAT0_ADDR;
			reg_data[0] = calc_heater_res(dev->gas_sett.heatr_temp, dev);
			reg_addr[1] = BME68X_GAS_WAIT0_ADDR;
			reg_data[1] = calc_heater_dur(dev->gas_sett.heatr_dur);
			dev->gas_sett.nb_conv = 0;
		} else {
			rslt = BME68X_W_DEFINE_PWR_MODE;
		}
		if (rslt == BME68X_OK)
			rslt = bme68x_set_regs(reg_addr, reg_data, 2, dev);
	}

	return rslt;
}

static int8_t get_gas_config(struct bme68x_dev *dev)
{
	int8_t rslt;
	/* starting address of the register array for burst read*/
	uint8_t reg_addr1 = BME68X_ADDR_SENS_CONF_START;
	uint8_t reg_addr2 = BME68X_ADDR_GAS_CONF_START;
	uint8_t data_array[BME68X_GAS_HEATER_PROF_LEN_MAX] = { 0 };
	uint8_t index;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);
	if (rslt == BME68X_OK) {
		if (BME68X_SPI_INTF == dev->intf) {
			/* Memory page switch the SPI address*/
			rslt = set_mem_page(reg_addr1, dev);
		}

		if (rslt == BME68X_OK) {
			rslt = bme68x_get_regs(reg_addr1, data_array, BME68X_GAS_HEATER_PROF_LEN_MAX, dev);
			if (rslt == BME68X_OK) {
				for (index = 0; index < BME68X_GAS_HEATER_PROF_LEN_MAX; index++)
					dev->gas_sett.heatr_temp = data_array[index];
			}

			rslt = bme68x_get_regs(reg_addr2, data_array, BME68X_GAS_HEATER_PROF_LEN_MAX, dev);
			if (rslt == BME68X_OK) {
				for (index = 0; index < BME68X_GAS_HEATER_PROF_LEN_MAX; index++)
					dev->gas_sett.heatr_dur = data_array[index];
			}
		}
	}

	return rslt;
}

static int16_t calc_temperature(uint32_t temp_adc, struct bme68x_dev *dev)
{
	int64_t var1;
	int64_t var2;
	int64_t var3;
	int16_t calc_temp;

	var1 = ((int32_t) temp_adc >> 3) - ((int32_t) dev->calib.par_t1 << 1);
	var2 = (var1 * (int32_t) dev->calib.par_t2) >> 11;
	var3 = ((var1 >> 1) * (var1 >> 1)) >> 12;
	var3 = ((var3) * ((int32_t) dev->calib.par_t3 << 4)) >> 14;
	dev->calib.t_fine = (int32_t) (var2 + var3);
	calc_temp = (int16_t) (((dev->calib.t_fine * 5) + 128) >> 8);

	return calc_temp;
}

static uint32_t calc_pressure(uint32_t pres_adc, const struct bme68x_dev *dev)
{
	int32_t var1 = 0;
	int32_t var2 = 0;
	int32_t var3 = 0;
	int32_t var4 = 0;
	int32_t pressure_comp = 0;

	var1 = (((int32_t)dev->calib.t_fine) >> 1) - 64000;
	var2 = ((((var1 >> 2) * (var1 >> 2)) >> 11) *
		(int32_t)dev->calib.par_p6) >> 2;
	var2 = var2 + ((var1 * (int32_t)dev->calib.par_p5) << 1);
	var2 = (var2 >> 2) + ((int32_t)dev->calib.par_p4 << 16);
	var1 = (((((var1 >> 2) * (var1 >> 2)) >> 13) *
		((int32_t)dev->calib.par_p3 << 5)) >> 3) +
		(((int32_t)dev->calib.par_p2 * var1) >> 1);
	var1 = var1 >> 18;
	var1 = ((32768 + var1) * (int32_t)dev->calib.par_p1) >> 15;
	pressure_comp = 1048576 - pres_adc;
	pressure_comp = (int32_t)((pressure_comp - (var2 >> 12)) * ((uint32_t)3125));
	var4 = (1L << 31);
	if (pressure_comp >= var4)
		pressure_comp = ((pressure_comp / (uint32_t)var1) << 1);
	else
		pressure_comp = ((pressure_comp << 1) / (uint32_t)var1);
	var1 = ((int32_t)dev->calib.par_p9 * (int32_t)(((pressure_comp >> 3) *
		(pressure_comp >> 3)) >> 13)) >> 12;
	var2 = ((int32_t)(pressure_comp >> 2) *
		(int32_t)dev->calib.par_p8) >> 13;
	var3 = ((int32_t)(pressure_comp >> 8) * (int32_t)(pressure_comp >> 8) *
		(int32_t)(pressure_comp >> 8) *
		(int32_t)dev->calib.par_p10) >> 17;

	pressure_comp = (int32_t)(pressure_comp) + ((var1 + var2 + var3 +
		((int32_t)dev->calib.par_p7 << 7)) >> 4);

	return (uint32_t)pressure_comp;

}

static uint32_t calc_humidity(uint16_t hum_adc, const struct bme68x_dev *dev)
{
	int32_t var1;
	int32_t var2;
	int32_t var3;
	int32_t var4;
	int32_t var5;
	int32_t var6;
	int32_t temp_scaled;
	int32_t calc_hum;

	temp_scaled = (((int32_t) dev->calib.t_fine * 5) + 128) >> 8;
	var1 = (int32_t) (hum_adc - ((int32_t) ((int32_t) dev->calib.par_h1 * 16)))
		- (((temp_scaled * (int32_t) dev->calib.par_h3) / ((int32_t) 100)) >> 1);
	var2 = ((int32_t) dev->calib.par_h2
		* (((temp_scaled * (int32_t) dev->calib.par_h4) / ((int32_t) 100))
			+ (((temp_scaled * ((temp_scaled * (int32_t) dev->calib.par_h5) / ((int32_t) 100))) >> 6)
				/ ((int32_t) 100)) + (int32_t) (1 << 14))) >> 10;
	var3 = var1 * var2;
	var4 = (int32_t) dev->calib.par_h6 << 7;
	var4 = ((var4) + ((temp_scaled * (int32_t) dev->calib.par_h7) / ((int32_t) 100))) >> 4;
	var5 = ((var3 >> 14) * (var3 >> 14)) >> 10;
	var6 = (var4 * var5) >> 1;
	calc_hum = (((var3 + var6) >> 10) * ((int32_t) 1000)) >> 12;

	if (calc_hum > 100000) /* Cap at 100%rH */
		calc_hum = 100000;
	else if (calc_hum < 0)
		calc_hum = 0;

	return (uint32_t) calc_hum;
}

#ifdef BME68X_USE_FLOAT
/* This internal API is used to calculate the gas resistance low value in float */
static float calc_gas_resistance_low(uint16_t gas_res_adc, uint8_t gas_range, const struct bme68x_dev *dev)
{
    float calc_gas_res;
    float var1;
    float var2;
    float var3;
    float gas_res_f = gas_res_adc;
    float gas_range_f = (1U << gas_range); /*lint !e790 / Suspicious truncation, integral to float */
    const float lookup_k1_range[16] = {
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, -0.8f, 0.0f, 0.0f, -0.2f, -0.5f, 0.0f, -1.0f, 0.0f, 0.0f
    };
    const float lookup_k2_range[16] = {
        0.0f, 0.0f, 0.0f, 0.0f, 0.1f, 0.7f, 0.0f, -0.8f, -0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
    };

    var1 = (1340.0f + (5.0f * dev->calib.range_sw_err));
    var2 = (var1) * (1.0f + lookup_k1_range[gas_range] / 100.0f);
    var3 = 1.0f + (lookup_k2_range[gas_range] / 100.0f);
    calc_gas_res = 1.0f / (float)(var3 * (0.000000125f) * gas_range_f * (((gas_res_f - 512.0f) / var2) + 1.0f));

    return calc_gas_res;
}

/* This internal API is used to calculate the gas resistance value in float */
static float calc_gas_resistance_high(uint16_t gas_res_adc, uint8_t gas_range)
{
    float calc_gas_res;
    uint32_t var1 = UINT32_C(262144) >> gas_range;
    int32_t var2 = (int32_t)gas_res_adc - INT32_C(512);

    var2 *= INT32_C(3);
    var2 = INT32_C(4096) + var2;

    calc_gas_res = 1000000.0f * (float)var1 / (float)var2;

    return calc_gas_res;
}

#else
/* This internal API is used to calculate the gas resistance low */
uint32_t calc_gas_resistance_low(uint16_t gas_res_adc, uint8_t gas_range, const struct bme68x_dev *dev)
{
	int64_t var1;
    uint64_t var2;
    int64_t var3;
    uint32_t calc_gas_res;
    uint32_t lookup_table1[16] = {
        UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2147483647),
        UINT32_C(2126008810), UINT32_C(2147483647), UINT32_C(2130303777), UINT32_C(2147483647), UINT32_C(2147483647),
        UINT32_C(2143188679), UINT32_C(2136746228), UINT32_C(2147483647), UINT32_C(2126008810), UINT32_C(2147483647),
        UINT32_C(2147483647)
    };
    uint32_t lookup_table2[16] = {
        UINT32_C(4096000000), UINT32_C(2048000000), UINT32_C(1024000000), UINT32_C(512000000), UINT32_C(255744255),
        UINT32_C(127110228), UINT32_C(64000000), UINT32_C(32258064), UINT32_C(16016016), UINT32_C(8000000), UINT32_C(
            4000000), UINT32_C(2000000), UINT32_C(1000000), UINT32_C(500000), UINT32_C(250000), UINT32_C(125000)
    };

    /*lint -save -e704 */
    var1 = (int64_t)((1340 + (5 * (int64_t)dev->calib.range_sw_err)) * ((int64_t)lookup_table1[gas_range])) >> 16;
    var2 = (((int64_t)((int64_t)gas_res_adc << 15) - (int64_t)(16777216)) + var1);
    var3 = (((int64_t)lookup_table2[gas_range] * (int64_t)var1) >> 9);
    calc_gas_res = (uint32_t)((var3 + ((int64_t)var2 >> 1)) / (int64_t)var2);

    /*lint -restore */
    return calc_gas_res;
}

/* This internal API is used to calculate the gas resistance */
uint32_t calc_gas_resistance_high(uint16_t gas_res_adc, uint8_t gas_range)
{
	uint32_t calc_gas_res;
    uint32_t var1 = UINT32_C(262144) >> gas_range;
    int32_t var2 = (int32_t)gas_res_adc - INT32_C(512);

    var2 *= INT32_C(3);
    var2 = INT32_C(4096) + var2;

    /* multiplying 10000 then dividing then multiplying by 100 instead of multiplying by 1000000 to prevent overflow */
    calc_gas_res = (UINT32_C(10000) * var1) / (uint32_t)var2;
    calc_gas_res = calc_gas_res * 100;

    return calc_gas_res;
}
#endif


static uint8_t calc_heater_res(uint16_t temp, const struct bme68x_dev *dev)
{
			   
																			 
	uint8_t heatr_res;
	int32_t var1;
	int32_t var2;
	int32_t var3;
	int32_t var4;
	int32_t var5;
	int32_t heatr_res_x100;

	if (temp < 200) /* Cap temperature */
		temp = 200;
	else if (temp > 400)
		temp = 400;

	var1 = (((int32_t) dev->amb_temp * dev->calib.par_gh3) / 1000) * 256;
	var2 = (dev->calib.par_gh1 + 784) * (((((dev->calib.par_gh2 + 154009) * temp * 5) / 100) + 3276800) / 10);
	var3 = var1 + (var2 / 2);
	var4 = (var3 / (dev->calib.res_heat_range + 4));
	var5 = (131 * dev->calib.res_heat_val) + 65536;
	heatr_res_x100 = (int32_t) (((var4 / var5) - 250) * 34);
	heatr_res = (uint8_t) ((heatr_res_x100 + 50) / 100);

	return heatr_res;
}

static uint8_t calc_heater_dur(uint16_t dur)
{
			  
																		   
	uint8_t factor = 0;
	uint8_t durval;

	if (dur >= 0xfc0) {
		durval = 0xff; /* Max duration*/
	} else {
		while (dur > 0x3F) {
			dur = dur / 4;
			factor += 1;
		}
		durval = (uint8_t) (dur + (factor * 64));
	}

	return durval;
}


static int8_t read_field_data(struct bme68x_field_data *data, struct bme68x_dev *dev)
{
	int8_t rslt = BME68X_OK;
	uint8_t buff[BME68X_FIELD_ADDR_OFFSET] = { 0 };

	uint8_t gas_range;
	uint32_t adc_temp;
	uint32_t adc_pres;
	uint16_t adc_hum;
	uint16_t adc_gas_res;
	uint8_t tries = 10;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);
	do {
		if (rslt == BME68X_OK) {
			rslt = bme68x_get_regs(((uint8_t) (BME68X_FIELD0_ADDR)), buff, (uint16_t) BME68X_FIELD_ADDR_OFFSET, dev);
		 

			data->status = buff[0] & BME68X_NEW_DATA_MSK;
			data->gas_index = buff[0] & BME68X_GAS_INDEX_MSK;
			data->meas_index = buff[1];

			/* read the raw data from the sensor */
			adc_pres = (uint32_t) (((uint32_t) buff[2] * 4096) | ((uint32_t) buff[3] * 16) | ((uint32_t) buff[4] / 16));
								 
			adc_temp = (uint32_t) (((uint32_t) buff[5] * 4096) | ((uint32_t) buff[6] * 16) | ((uint32_t) buff[7] / 16));
								 
			adc_hum = (uint16_t) (((uint32_t) buff[8] * 256) | (uint32_t) buff[9]);

			if (dev->variant_id == BME68X_VARIANT_GAS_HIGHV)
			{
				adc_gas_res = (uint16_t) ((uint32_t) buff[15] * 4 | (((uint32_t) buff[16]) / 64));
				gas_range = buff[16] & BME68X_GAS_RANGE_MSK;
				data->status |= buff[16] & BME68X_GASM_VALID_MSK;
				data->status |= buff[16] & BME68X_HEAT_STAB_MSK;
																														
			}else{
				adc_gas_res = (uint16_t) ((uint32_t) buff[13] * 4 | (((uint32_t) buff[14]) / 64));
				gas_range = buff[14] & BME68X_GAS_RANGE_MSK;

				data->status |= buff[14] & BME68X_GASM_VALID_MSK;
				data->status |= buff[14] & BME68X_HEAT_STAB_MSK;
																														
			}

			if (data->status & BME68X_NEW_DATA_MSK) {
				data->temperature = calc_temperature(adc_temp, dev);
				data->pressure = calc_pressure(adc_pres, dev);
				data->humidity = calc_humidity(adc_hum, dev);
				if (dev->variant_id == BME68X_VARIANT_GAS_HIGHV)
				{
					data->gas_resistance = calc_gas_resistance_high(adc_gas_res, gas_range);
				}else{
					data->gas_resistance = calc_gas_resistance_low(adc_gas_res, gas_range, dev);
				}															   
				break;
			}
			/* Delay to poll the data */
			dev->delay_ms(BME68X_POLL_PERIOD_MS);
		}
		tries--;
	} while (tries);

	if (!tries)
		rslt = BME68X_W_NO_NEW_DATA;

	return rslt;
}

static int8_t set_mem_page(uint8_t reg_addr, struct bme68x_dev *dev)
{
	int8_t rslt;
	uint8_t reg;
	uint8_t mem_page;

	/* Check for null pointers in the device structure*/
	rslt = null_ptr_check(dev);
	if (rslt == BME68X_OK) {
		if (reg_addr > 0x7f)
			mem_page = BME68X_MEM_PAGE1;
		else
			mem_page = BME68X_MEM_PAGE0;

		if (mem_page != dev->mem_page) {
			dev->mem_page = mem_page;

			dev->com_rslt = dev->read(dev->dev_id, BME68X_MEM_PAGE_ADDR | BME68X_SPI_RD_MSK, &reg, 1);
			if (dev->com_rslt != 0)
				rslt = BME68X_E_COM_FAIL;

			if (rslt == BME68X_OK) {
				reg = reg & (~BME68X_MEM_PAGE_MSK);
				reg = reg | (dev->mem_page & BME68X_MEM_PAGE_MSK);

				dev->com_rslt = dev->write(dev->dev_id, BME68X_MEM_PAGE_ADDR & BME68X_SPI_WR_MSK,
					&reg, 1);
				if (dev->com_rslt != 0)
					rslt = BME68X_E_COM_FAIL;
			}
		}
	}

	return rslt;
}

static int8_t get_mem_page(struct bme68x_dev *dev)
{
	int8_t rslt;
	uint8_t reg;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);
	if (rslt == BME68X_OK) {
		dev->com_rslt = dev->read(dev->dev_id, BME68X_MEM_PAGE_ADDR | BME68X_SPI_RD_MSK, &reg, 1);
		if (dev->com_rslt != 0)
			rslt = BME68X_E_COM_FAIL;
		else
			dev->mem_page = reg & BME68X_MEM_PAGE_MSK;
	}

	return rslt;
}

static int8_t boundary_check(uint8_t *value, uint8_t min, uint8_t max, struct bme68x_dev *dev)
{
	int8_t rslt = BME68X_OK;

	if (value != NULL) {
		/* Check if value is below minimum value */
		if (*value < min) {
			/* Auto correct the invalid value to minimum value */
			*value = min;
			dev->info_msg |= BME68X_I_MIN_CORRECTION;
		}
		/* Check if value is above maximum value */
		if (*value > max) {
			/* Auto correct the invalid value to maximum value */
			*value = max;
			dev->info_msg |= BME68X_I_MAX_CORRECTION;
		}
	} else {
		rslt = BME68X_E_NULL_PTR;
	}

	return rslt;
}

static int8_t null_ptr_check(const struct bme68x_dev *dev)
{
	int8_t rslt;

	if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_ms == NULL)) {
		/* Device structure pointer is not valid */
		rslt = BME68X_E_NULL_PTR;
	} else {
		/* Device structure is fine */
		rslt = BME68X_OK;
	}

	return rslt;
}
																									
/* This internal API is used to read variant ID information from the register */
static int8_t read_variant_id(struct bme68x_dev *dev)
{
    int8_t rslt;
    uint8_t reg_data = 0;

    /* Read variant ID information register */
    rslt = bme68x_get_regs(BME68X_REG_VARIANT_ID, &reg_data, 1, dev);

    if (rslt == BME68X_OK)
    {
        dev->variant_id = reg_data;
    }

    return rslt;
}		 