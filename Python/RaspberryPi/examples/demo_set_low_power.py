# -*- coding:utf-8 -*-
from __future__ import print_function


'''!
  @file demo_set_low_power.py
  @brief connect bme68x I2C interface with your board (please reference board compatibility)
  @n You can set sensor low power mode or wake up mode.
  @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license     The MIT License (MIT)
  @author [GDuang](yonglei.ren@dfrobot.com)
  @version  V1.0
  @date  2024-04-25
  @url https://github.com/DFRobot/DFRobot_BME68x
'''
import sys
sys.path.append('../')
from DFRobot_BME68x import DFRobot_BME68x
import time

sensor = DFRobot_BME68x()

#sensor.set_power_mode(sensor.SLEEP_MODE) # set low power mode
sensor.set_power_mode(sensor.FORCED_MODE) # set wake up mode
sensor.get_power_mode() # get current power mode

if sensor.get_power_mode() == 141:
    print("Current power mode is forced")
else:
    print("Current power mode is sleep")

while True:
    time.sleep(1)
