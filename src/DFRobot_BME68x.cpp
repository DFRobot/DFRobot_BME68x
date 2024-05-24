/**
 * @file DFRobot_BME68x.cpp
 * @brief  Defines the infrastructure of the DFRobot_BME68x class and the implementation of the underlying methods
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [GDuang](yonglei.ren@dfrobot.com)
 * @version  V1.0
 * @date  2024-04-25
 * @url https://github.com/DFRobot/DFRobot_BME68x
 */
#include "DFRobot_BME68x.h"

static struct        bme68x_dev bme68x_sensor;
static struct        bme68x_field_data bme68x_data;
static uint8_t       convertCmd = (0x05 << 5) | (0x05 << 2) | (0x01);
static uint8_t       iaqReady_ = 0;



void bme68x_outputReady(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature,
                        float humidity, float pressure, float raw_temperature, float raw_humidity,
                        float gas, bsec_library_return_t bsec_status)
{
  UNUSED(timestamp);
  UNUSED(raw_temperature);
  UNUSED(raw_humidity);
  UNUSED(gas);
  UNUSED(bsec_status);
  if(iaq_accuracy != 0) bme68x_data.gas_index = iaq;
  bme68x_data.temperature = temperature;
  bme68x_data.humidity = humidity;
  bme68x_data.pressure = pressure;
  if(iaq_accuracy != 0) iaqReady_ = 1;
}

void bme68x_delay_ms(uint32_t period)
{
  delay(period);
}

int64_t bme68x_getus(void)
{
  return (millis() * 1000);
}

DFRobot_BME68x::DFRobot_BME68x(bme68x_com_fptr_t readReg, bme68x_com_fptr_t writeReg, 
                               bme68x_delay_fptr_t delayMS, eBME68X_INTERFACE interface)
{
  bme68x_sensor.read = readReg;
  bme68x_sensor.write = writeReg;
  bme68x_sensor.delay_ms = delayMS;
  switch(interface) {
    case eBME68X_INTERFACE_I2C: bme68x_sensor.intf = BME68X_I2C_INTF; break;
    case eBME68X_INTERFACE_SPI: bme68x_sensor.intf = BME68X_SPI_INTF; break;
  }
}

int16_t DFRobot_BME68x::begin(void)
{
  bme68x_sensor.dev_id = this->bme68x_I2CAddr;
  if(bme68x_init(&bme68x_sensor) != BME68X_OK) {
    return -1;
  }
  uint8_t       set_required_settings;

	/* Set the temperature, pressure and humidity settings */
	bme68x_sensor.tph_sett.os_hum = BME68X_OS_2X;
	bme68x_sensor.tph_sett.os_pres = BME68X_OS_4X;
	bme68x_sensor.tph_sett.os_temp = BME68X_OS_8X;
	bme68x_sensor.tph_sett.filter = BME68X_FILTER_SIZE_3;
  setGasHeater(320, 150);

  /* Select the power mode */
  /* Must be set before writing the sensor configuration */
  bme68x_sensor.power_mode = BME68X_FORCED_MODE; 

  /* Set the required sensor settings needed */
  set_required_settings = BME68X_OST_SEL | BME68X_OSP_SEL | BME68X_OSH_SEL | BME68X_FILTER_SEL 
   | BME68X_GAS_SENSOR_SEL;

  /* Set the desired sensor configuration */
  bme68x_set_sensor_settings(set_required_settings, &bme68x_sensor);

	/* Set the power mode */
	bme68x_set_sensor_mode(&bme68x_sensor);

	/* Get the total measurement duration so as to sleep or wait till the
	 * measurement is complete */
	uint16_t meas_period;
	bme68x_get_profile_dur(&meas_period, &bme68x_sensor);
	bme68x_sensor.delay_ms(meas_period); /* Delay till the measurement is ready */
  return 0;
}

void DFRobot_BME68x::startConvert(void)
{
  bme68x_sensor.write(bme68x_sensor.dev_id, 0x74, &convertCmd, 1);
}

void DFRobot_BME68x::update(void)
{
  bme68x_get_sensor_data(&bme68x_data, &bme68x_sensor);
}

int8_t DFRobot_BME68x::iaqUpdate(void)
{
  uint8_t       gasBuf[2] = {0};
  uint16_t      gasADC = 0;
  uint8_t       gasRange = 0;
  if(bsec_iot_loop(bme68x_delay_ms, bme68x_getus, bme68x_outputReady) == 0) {

    if (bme68x_sensor.variant_id == BME68X_VARIANT_GAS_HIGHV){
      bme68x_get_regs(0x2c, gasBuf, 2, &bme68x_sensor);
    }else{
      bme68x_get_regs(0x2a, gasBuf, 2, &bme68x_sensor);
    }
    gasADC = (uint16_t) ((uint32_t) gasBuf[0] * 4 | (((uint32_t) gasBuf[1]) / 64));
    gasRange = gasBuf[1] & BME68X_GAS_RANGE_MSK;
    if (bme68x_sensor.variant_id == BME68X_VARIANT_GAS_HIGHV){
      bme68x_data.gas_resistance = calc_gas_resistance_high(gasADC, gasRange);
    }else{
      bme68x_data.gas_resistance = calc_gas_resistance_low(gasADC, gasRange, &bme68x_sensor);
    }
    return 0;
  }
  return 1;
}

float DFRobot_BME68x::readTemperature(void)
{
  return bme68x_data.temperature;
}

float DFRobot_BME68x::readPressure(void)
{
  return bme68x_data.pressure;
}

float DFRobot_BME68x::readHumidity(void)
{
  return bme68x_data.humidity;
}

float DFRobot_BME68x::readAltitude(void)
{
  return (1.0 - pow((float)bme68x_data.pressure / 100.0f / BME68X_SEALEVEL, 0.190284)) * 287.15 / 0.0065;
}

float DFRobot_BME68x::readCalibratedAltitude(float seaLevel)
{
   return (1.0 - pow((float) bme68x_data.pressure / seaLevel, 0.190284)) * 287.15 / 0.0065;
}

float DFRobot_BME68x::readGasResistance(void)
{
  return bme68x_data.gas_resistance;
}

float DFRobot_BME68x::readSeaLevel(float altitude)
{
  return (bme68x_data.pressure / pow(1.0 - (altitude / 44330.0), 5.255));
}

float DFRobot_BME68x::readIAQ(void)
{
  return bme68x_data.gas_index;
}

bool DFRobot_BME68x::setGasHeater(uint16_t heaterTemp, uint16_t heaterTime)
{
  if((heaterTemp == 0) || (heaterTime == 0)){
    bme68x_sensor.gas_sett.heatr_ctrl = BME68X_DISABLE_HEATER;
    bme68x_sensor.gas_sett.run_gas = BME68X_DISABLE_GAS_MEAS;
  }else{
    bme68x_sensor.gas_sett.heatr_ctrl = BME68X_ENABLE_HEATER;
    bme68x_sensor.gas_sett.run_gas = BME68X_ENABLE_GAS_MEAS;
    bme68x_sensor.gas_sett.heatr_temp = heaterTemp; /* degree Celsius */
    bme68x_sensor.gas_sett.heatr_dur = heaterTime; /* milliseconds */
  }
  uint8_t set_required_settings = BME68X_GAS_SENSOR_SEL;
  uint8_t rslt = bme68x_set_sensor_settings(set_required_settings, &bme68x_sensor);
  return rslt == 0;
}

uint8_t DFRobot_BME68x::isIAQReady(void)
{
  return iaqReady_;
}

void DFRobot_BME68x::writeParamHelper(uint8_t reg, uint8_t dat, uint8_t addr)
{
  uint8_t       var1 = 0;
  uint8_t       addrCount = 0;
  if(bme68x_sensor.intf == BME68X_SPI_INTF) bme68x_sensor.write(bme68x_sensor.dev_id, 0x73, 0x00, 1);
  bme68x_sensor.read(bme68x_sensor.dev_id, reg, &var1, 1);
  var1 &= ~addr;
  while(!(addr & 0x01)) {
    addrCount ++;
    addr >>= 1;
  }
  var1 |= dat << addrCount;
  bme68x_sensor.write(bme68x_sensor.dev_id, reg, &var1, 1);
}

static int8_t bme68x_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  Wire.begin();
  Wire.beginTransmission(dev_id);
  Wire.write(reg_addr);
  Wire.endTransmission();
  Wire.requestFrom(dev_id, (uint8_t)len);
  while(Wire.available()) {
    *data = Wire.read();
    data ++;
  }
  return 0;
}

static int8_t bme68x_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  Wire.begin();
  Wire.beginTransmission(dev_id);
  Wire.write(reg_addr);
  while(len --) {
    Wire.write(*data);
    data ++;
  }
  Wire.endTransmission();
  return 0;
}

DFRobot_BME68x_I2C::DFRobot_BME68x_I2C(uint8_t I2CAddr_) :
                    DFRobot_BME68x(bme68x_i2c_read, bme68x_i2c_write, bme68x_delay_ms, eBME68X_INTERFACE_I2C)
{
  bme68x_I2CAddr = I2CAddr_;
}

void DFRobot_BME68x_I2C::setConvertAndUpdate()
{
  bsec_iot_init(0.33333f, 0, bme68x_i2c_write, bme68x_i2c_read, bme68x_delay_ms, bme68x_I2CAddr, BME68X_I2C_INTF);
}

static uint8_t bme68x_cs = 0;

static int8_t bme68x_spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  UNUSED(dev_id);
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(bme68x_cs, 0);
  SPI.transfer(reg_addr | 0x80);
  while(len --) {
    *data = SPI.transfer(0x00);
    data ++;
  }
  digitalWrite(bme68x_cs, 1);
  SPI.endTransaction();
  return 0;
}

static int8_t bme68x_spi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  UNUSED(dev_id); 
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(bme68x_cs, 0);
  SPI.transfer(reg_addr);
  while(len --) {
    SPI.transfer(*data);
    data ++;
  }
  digitalWrite(bme68x_cs, 1);
  SPI.endTransaction();
  return 0;
}

DFRobot_BME68x_SPI::DFRobot_BME68x_SPI(uint8_t pin_cs) :
                    DFRobot_BME68x(bme68x_spi_read, bme68x_spi_write, bme68x_delay_ms, eBME68X_INTERFACE_SPI)
{
  bme68x_cs = pin_cs;
  pinMode(bme68x_cs, OUTPUT);
  digitalWrite(bme68x_cs, 1);
}

void DFRobot_BME68x_SPI::setConvertAndUpdate()
{
  bsec_iot_init(0.33333f, 0, bme68x_spi_write, bme68x_spi_read, bme68x_delay_ms, 0x77, BME68X_SPI_INTF);
}

