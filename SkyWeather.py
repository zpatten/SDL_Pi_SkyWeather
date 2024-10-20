#
# SkyWeather Solar Powered Weather Station
# February 2019
#
# SwitchDoc Labs
# www.switchdoc.com
#
#


# imports
# Check for user imports
try:
  import conflocal as config
except ImportError:
  import config

config.SWVERSION = "055"

import gc
gc.enable()

import sys
import time
import traceback
import atexit

from datetime import datetime

import random 
import re
import math
import os
import threading
import commands

import pytz
timezone = pytz.timezone('US/Central')

import logging
logging.basicConfig()

import pclogging

import state

import json
import paho.mqtt.client as mqtt


sys.path.append('./TSL2591')
sys.path.append('./SDL_Pi_SI1145')
sys.path.append('./SDL_Pi_TCA9545')

sys.path.append('./SDL_Pi_SSD1306')
sys.path.append('./Adafruit_Python_SSD1306')
sys.path.append('./RTC_SDL_DS3231')
sys.path.append('./Adafruit_Python_BMP')
sys.path.append('./Adafruit_Python_GPIO')
sys.path.append('./SDL_Pi_WeatherRack')
sys.path.append('./RaspberryPi-AS3935/RPi_AS3935')
sys.path.append('./SDL_Pi_INA3221')
sys.path.append('./SDL_Pi_HDC1000')
sys.path.append('./SDL_Pi_AM2315')
sys.path.append('./SDL_Pi_SHT30')
sys.path.append('./BME680')

sys.path.append('./SDL_Pi_GrovePowerDrive')

RAIN_ARRAY_SIZE = 20

import subprocess
import RPi.GPIO as GPIO
import smbus

import struct

import SDL_Pi_HDC1000


from apscheduler.schedulers.background import BackgroundScheduler

import apscheduler.events

if config.enable_MySQL_Logging:
  import MySQLdb as mdb

import DustSensor

#import GroveGPS

import util

################
# Device Present State Variables
###############

config.TCA9545_I2CMux_Present = False
config.INA3221_Present = False
config.AS3935_Present = False
config.DS3231_Present = False
config.BMP280_Present = False
config.BME680_Present = False
config.AM2315_Present = False
config.ADS1015_Present = False
config.ADS1115_Present = False
config.OLED_Present = False
config.SI1145_Present = False
config.TSL2591_Present = False
config.GPS_Present = False

import SDL_Pi_INA3221
import SDL_DS3231
import Adafruit_BMP.BMP280 as BMP280
import SDL_Pi_WeatherRack as SDL_Pi_WeatherRack
import bme680 as BME680
import BME680_Functions

from RPi_AS3935 import RPi_AS3935


import Adafruit_SSD1306

import Scroll_SSD1306


import SDL_Pi_SI1145
import SI1145Lux

import TSL2591
import SDL_Pi_TCA9545

# semaphore primitives for preventing I2C conflicts
I2C_Lock = threading.Lock()



################################################################################
# MQTT

state.mqtt_client = mqtt.Client(client_id="SkyWeather")
state.mqtt_client.username_pw_set(config.MQTT_Username, config.MQTT_Password)
state.mqtt_client.connect(config.MQTT_ServerURL, port=config.MQTT_Port)
state.mqtt_client.loop_start()

def mqtt_publish(topic, message):
  data = json.dumps(message)
  print("MQTT[%s]: %s" % (topic, data))
  state.mqtt_client.publish(topic, data, retain=True)



################################################################################

def print_exception(message=None):
  print("=" * 80)
  if message:
    print("EXCEPTION: %s" % message)
    print("-" * 80)
  print(traceback.format_exc())
  print("=" * 80)



################################################################################

def patTheDog():
  # pat the dog
  print "------Patting The Dog------- "
  GPIO.setup(config.WATCHDOGTRIGGER, GPIO.OUT)
  GPIO.output(config.WATCHDOGTRIGGER, False)
  time.sleep(0.2)
  GPIO.output(config.WATCHDOGTRIGGER, True)
  GPIO.setup(config.WATCHDOGTRIGGER, GPIO.IN)

#patTheDog()



################################################################################

def returnStatusLine(device, state):
  fill_length = 20 - len(device)
  line = device
  line += "." * fill_length
  line += ": "
  if state:
    line += "Present"
  else:
    line += "Not Present"
  return line



################################################################################
# TCA9545 I2C Mux 

#/*=========================================================================
#    I2C ADDRESS/BITS
#    -----------------------------------------------------------------------*/
TCA9545_ADDRESS     = (0x73)    # 1110011 (A0+A1=VDD)
#/*=========================================================================*/

#/*=========================================================================
#    CONFIG REGISTER (R/W)
#    -----------------------------------------------------------------------*/
TCA9545_REG_CONFIG  = (0x00)
#    /*---------------------------------------------------------------------*/

TCA9545_CONFIG_BUS0 = (0x01)  # 1 = enable, 0 = disable 
TCA9545_CONFIG_BUS1 = (0x02)  # 1 = enable, 0 = disable 
TCA9545_CONFIG_BUS2 = (0x04)  # 1 = enable, 0 = disable 
TCA9545_CONFIG_BUS3 = (0x08)  # 1 = enable, 0 = disable 

#/*=========================================================================*/

BUS_MAP = {
  TCA9545_CONFIG_BUS0: 0,
  TCA9545_CONFIG_BUS1: 1,
  TCA9545_CONFIG_BUS2: 2,
  TCA9545_CONFIG_BUS3: 3
}

BUS_DELAY = 1.0

# I2C Mux TCA9545 Detection
try:
  tca9545 = SDL_Pi_TCA9545.SDL_Pi_TCA9545(addr=TCA9545_ADDRESS, bus_enable = TCA9545_CONFIG_BUS0)

  # turn I2CBus 1 on
  tca9545.write_control_register(TCA9545_CONFIG_BUS2)
  config.TCA9545_I2CMux_Present = True
except:
  print("TCA9545 I2C Mux Not Present")
  config.TCA9545_I2CMux_Present = False


def activate_bus(requested_bus, lock=False):
  global previous_bus

  if lock:
    I2C_Lock.acquire()

  if config.TCA9545_I2CMux_Present:
    try:
      previous_bus = tca9545.read_control_register()
      if previous_bus != requested_bus:
        print("TCA9545: Activating BUS%d" % BUS_MAP[requested_bus])
        tca9545.write_control_register(requested_bus)
        time.sleep(BUS_DELAY)
    except:
      print_exception()
      print("TCA9545: Activating BUS%d" % BUS_MAP[requested_bus])
      try:
        tca9545.write_control_register(requested_bus)
        time.sleep(BUS_DELAY)
      except:
        print_exception()

def restore_bus(lock=False):
  global previous_bus

  if lock:
    I2C_Lock.release()

  if config.TCA9545_I2CMux_Present:
    try:
      current_bus = tca9545.read_control_register()
      if previous_bus != current_bus:
        print("TCA9545: Restoring BUS%d" % BUS_MAP[previous_bus])
        tca9545.write_control_register(previous_bus)
        time.sleep(BUS_DELAY)

    except:
      print_exception()
      try:
        print("TCA9545: Restoring BUS%d" % BUS_MAP[previous_bus])
        tca9545.write_control_register(previous_bus)
        time.sleep(BUS_DELAY)

      except:
        print_exception()



################################################################################
# Grove Power Saver

def removePower(GroveSavePin):
  GPIO.setup(GroveSavePin, GPIO.OUT)
  GPIO.output(GroveSavePin, False)
      

def restorePower(GroveSavePin):
  GPIO.setup(GroveSavePin, GPIO.OUT)
  GPIO.output(GroveSavePin, True)
   
def togglePower(GroveSavePin):
  print("Toggling Power to Pin ", GroveSavePin)
  removePower(GroveSavePin)
  time.sleep(4.5)
  restorePower(GroveSavePin)



################################################################################
# Fan Control

import SDL_Pi_GrovePowerDrive

TEMPFANTURNON = 37.0
TEMPFANTURNOFF = 34.0

myPowerDrive = SDL_Pi_GrovePowerDrive.SDL_Pi_GrovePowerDrive(config.GPIO_Pin_PowerDrive_Sig1, config.GPIO_Pin_PowerDrive_Sig2, False, False)

def turnFanOn():
  if not state.fanState:
    pclogging.log(pclogging.INFO, __name__, "Turning Fan On" )
    myPowerDrive.setPowerDrive(1, True) 
    myPowerDrive.setPowerDrive(2, True) 
    state.fanState = True

def turnFanOff():
  if state.fanState:
    pclogging.log(pclogging.INFO, __name__, "Turning Fan Off" )
    myPowerDrive.setPowerDrive(1, False) 
    myPowerDrive.setPowerDrive(2, False)
    state.fanState = False

turnFanOff()



################################################################################
# TSL2591 Light Sensor

try:
  print("Detecting TSL2591...")
  activate_bus(TCA9545_CONFIG_BUS3)
  tsl2591 = TSL2591.Tsl2591()
  int_time = TSL2591.INTEGRATIONTIME_100MS
  gain = TSL2591.GAIN_LOW
  tsl2591.set_gain(gain)
  tsl2591.set_timing(int_time)
  full, ir = tsl2591.get_full_luminosity()  # read raw values (full spectrum and ir spectrum)
  lux = tsl2591.calculate_lux(full, ir)  # convert raw values to lux
  print("TSL2591 Lux............: %d" % lux)
  print("TSL2591 Full Spectrum..: %d" % full)
  print("TSL2591 IR Spectrum....: %d" % ir)
  config.TSL2591_Present = True 
  print("TSL2591 Present")

except:
  config.TSL2591_Present = False 
  print("TSL2591 Not Present")



################################################################################
# SI1145 Light Sensor

try:
  print("Detecting SI1145...")
  activate_bus(TCA9545_CONFIG_BUS3)
  #restorePower(SI1145GSPIN)
  time.sleep(1.0)
  Sunlight_Sensor = SDL_Pi_SI1145.SDL_Pi_SI1145(indoor=0)
  time.sleep(1.0)

  visible = Sunlight_Sensor.readVisible()
  while visible == 0:
      #patTheDog()
      visible = Sunlight_Sensor.readVisible()
      time.sleep(1.0)
  IR = Sunlight_Sensor.readIR()
  UV = Sunlight_Sensor.readUV()
  IR_Lux = SI1145Lux.SI1145_IR_to_Lux(IR)
  vis_Lux = SI1145Lux.SI1145_VIS_to_Lux(visible)
  uvIndex = UV / 100.0
  config.SI1145_Present = True
  print("SI1145 Visible...: %d" % visible)
  print("SI1145 Lux.......: %d" % vis_Lux)
  print("SI1145 IR Lux....: %d" % IR_Lux)
  print("SI1145 UV Index..: %f" % uvIndex)
  print("SI1145 Present")

except:
  config.SI1145_Present = False
  print("SI1145 Not Present")



################################################################################
# GPS

def gpsAltitude():
  try:
    gps.read()
    [t, fix, sats, alt, lat, lat_ns, long, long_ew] = gps.vals()
    print("Time:",t," Satellites:",sats," Fix:",fix," Altitude:",alt," Lat:",lat,lat_ns," Long:",long,long_ew)
    if float(alt) > 0:
      config.BMP280_Altitude_Meters = float(alt)
      return float(alt)
    return 0
  except:
    print_exception()

try:
  print("Detecting GPS...")
  gps = GroveGPS.GPS()
  gps.read()
  gpsAltitude()
  config.GPS_Present = True
  print("GPS Present")
except:
  config.GPS_Present = False
  print("GPS Not Present")



################################################################################
# INA3221

LIPO_BATTERY_CHANNEL = 1
SOLAR_CELL_CHANNEL   = 2
OUTPUT_CHANNEL       = 3

try:
  print("Detecting INA3221...")
  activate_bus(TCA9545_CONFIG_BUS2)
  ina3221 = SDL_Pi_INA3221.SDL_Pi_INA3221(addr=0x40)
  busvoltage1 = ina3221.getBusVoltage_V(LIPO_BATTERY_CHANNEL)
  config.INA3221_Present = True
  print("INA3221 Present")
except:
  config.INA3221_Present = False
  print("INA3221 Not Present")



################################################################################
# HDC1080

try:
  print("Detecting HDC1080...")
  activate_bus(TCA9545_CONFIG_BUS0)
  hdc1080 = SDL_Pi_HDC1000.SDL_Pi_HDC1000() 
  deviceID = hdc1080.readDeviceID() 
  print "deviceID = 0x%X" % deviceID
  if (deviceID == 0x1050):
    config.HDC1080_Present = True
  else:
    config.HDC1080_Present = False
  print("HDC1080 Present")
except:
  config.HDC1080_Present = False
  print("HDC1080 Not Present")



################################################################################
# WeatherRack Weather Sensors
#
# GPIO Numbering Mode GPIO.BCM
#
# constants

SDL_MODE_INTERNAL_AD = 0
SDL_MODE_I2C_ADS1015 = 1    # internally, the library checks for ADS1115 or ADS1015 if found

# Sample mode means return immediately.  The wind speed is averaged at sampleTime or when you ask, whichever is longer
SDL_MODE_SAMPLE = 0
# Delay mode means to wait for sampleTime and the average after that time.
SDL_MODE_DELAY = 1

activate_bus(TCA9545_CONFIG_BUS0)
weatherStation = SDL_Pi_WeatherRack.SDL_Pi_WeatherRack(config.anemometerPin, config.rainPin, 0, 0, SDL_MODE_I2C_ADS1015)

#weatherStation.setWindMode(SDL_MODE_SAMPLE, 5.0)
weatherStation.setWindMode(SDL_MODE_DELAY, 5.0)



################################################################################
# DS3231/AT24C32 RTC

try:
  print("Detecting DS3231...")
  activate_bus(TCA9545_CONFIG_BUS0)

  filename = time.strftime("%Y-%m-%d%H:%M:%SRTCTest") + ".txt"
  starttime = datetime.utcnow()

  ds3231 = SDL_DS3231.SDL_DS3231(1, 0x68)
  ds3231.read_datetime()
  #print "DS3231=\t\t%s" % ds3231.read_datetime()
  config.DS3231_Present = True
  print("DS3231 Present")

except:
  config.DS3231_Present = False
  print("DS3231 Not Present")

def rtc_save():
  ds3231.write_now()

atexit.register(rtc_save)



################################################################################
# BMP280 Temperature/Humidity Sensor 

try:
  print("Detecting BMP280...")
  bmp280 = BMP280.BMP280()
  config.BMP280_Present = True
  print("BMP280 Present")
except: 
  config.BMP280_Present = False
  print("BMP280 Not Present")



################################################################################
# BME680 Temperature/Humidity Sensor 

try:
  print("Detecting BME680...")
  bme680 = BME680.BME680(BME680.I2C_ADDR_SECONDARY)
  BME680_Functions.setup_bme680(bme680)
  config.BME680_Present = True
  print("BME680 Present")
except:
  config.BME680_Present = False
  print("BME680 Not Present")



################################################################################
# SSD1306 OLED

try:
  print("Detecting SSD1306...")
  util.turnOLEDOn()
  RST = 27
  display = Adafruit_SSD1306.SSD1306_128_64(rst=RST, i2c_address=0x3C)
  # Initialize library.
  display.begin()
  display.clear()
  display.display()
  config.OLED_Present = True
  config.OLED_Originally_Present = True
  print("SSD1106 Present")
except:
  print_exception()
  config.OLED_Originally_Present = False
  config.OLED_Present = False
  print("SSD1106 Not Present")

try:
  util.turnOLEDOff()
except:
  print_exception()

def initializeOLED():
    util.turnOLEDOn()
    try:
        RST =27
        display = Adafruit_SSD1306.SSD1306_128_64(rst=RST, i2c_address=0x3C)
        # Initialize library.
        display.begin()
        display.clear()
        display.display()
        config.OLED_Present = True
        config.OLED_Originally_Present = True
    except:
        config.OLED_Originally_Present = False
        config.OLED_Present = False



################################################################################
# AS3935 Lightning Sensor

def handle_as3935_interrupt(channel=None):
  global as3935, as3935LastInterrupt, as3935LastDistance, as3935LastStatus, as3935LightningCount

  print("-" * 80)
  print("AS3935 Interrupt Handler")
  print("-" * 80)

  I2C_Lock.acquire()

  try:
    activate_bus(TCA9545_CONFIG_BUS1)

    reason = as3935.get_interrupt()
    timestamp = time.time()
    now = datetime.now().strftime('%H:%M:%S - %Y/%m/%d')

    as3935LastInterrupt = reason

    if reason == 0x00:
      as3935LastStatus = "Spurious Interrupt (%s)" % now
    elif reason == 0x01:
      as3935LastStatus = "Noise Floor Too Low; Adjusting (%s)" % now
      as3935.raise_noise_floor()
    elif reason == 0x04:
      as3935LastStatus = "Disturber Detected - Masking (%s)" % now
      as3935.set_mask_disturber(True)
    elif reason == 0x08:
      distance = as3935.get_distance()
      as3935LastDistance = distance
      as3935LightningCount += 1
      as3935LastStatus = "Lightning Detected "  + str(distance) + "km away. (%s)" % now

    pclogging.log(pclogging.INFO, __name__, as3935LastStatus)

    print("Last Interrupt = 0x%x:  %s" % (as3935LastInterrupt, as3935LastStatus))

    topic = 'skyweather/lightning'
    #now_utc = datetime.utcnow()
    #now_tz = now_utc.replace(tzinfo=pytz.utc).astimezone(timezone)
    stateMessage = {
      'as3935Timestamp': timestamp,
      'as3935LightningCount': as3935LightningCount,
      'as3935LastDistance': as3935LastDistance,
      'as3935LastStatus': as3935LastStatus,
    }
    mqtt_publish(topic, stateMessage)

    print("-" * 80)

  except:
    print_exception("AS3935 Interrupt")

  I2C_Lock.release()


# as3935 Set up Lightning Detector
as3935LastInterrupt = 0
as3935LightningCount = 0
as3935LastDistance = 0
as3935LastStatus = ""

#set values for lightning
# format: [NoiseFloor, Indoor, TuneCap, DisturberDetection, WatchDogThreshold, SpikeDetection]
# default: [2,1,7,0,3,3]
NoiseFloor = config.AS3935_Lightning_Config[0]
Indoor = config.AS3935_Lightning_Config[1]
TuneCap = config.AS3935_Lightning_Config[2]
DisturberDetection = config.AS3935_Lightning_Config[3]
WatchDogThreshold = config.AS3935_Lightning_Config[4]
SpikeDetection = config.AS3935_Lightning_Config[5]

try:
  print("Detecting AS3935 at 0x02...")
  activate_bus(TCA9545_CONFIG_BUS1)
  as3935 = RPi_AS3935(address=0x02, bus=1)
  as3935.set_noise_floor(NoiseFloor)
  as3935.set_indoors(Indoor)
  as3935.calibrate(tun_cap=TuneCap)
  as3935.set_mask_disturber(DisturberDetection)
  as3935.set_watchdog_threshold(WatchDogThreshold)
  as3935.set_spike_detection(SpikeDetection)

  config.AS3935_Present = True
  print("AS3935 Present at 0x02")
  handle_as3935_interrupt()

except:
  print("AS3935 Not Present at 0x02")

  try:
    print("Detecting AS3935 at 0x03...")
    activate_bus(TCA9545_CONFIG_BUS1)
    time.sleep(3)
    as3935 = RPi_AS3935(address=0x03, bus=1)

    as3935.set_noise_floor(NoiseFloor)
    as3935.set_indoors(Indoor)
    as3935.calibrate(tun_cap=TuneCap)
    as3935.set_mask_disturber(DisturberDetection)
    as3935.set_watchdog_threshold(WatchDogThreshold)
    as3935.set_spike_detection(SpikeDetection)

    config.AS3935_Present = True
    print("AS3935 Present at 0x03")
    handle_as3935_interrupt()

  except:
    config.AS3935_Present = False
    print("AS3935 Not Present at 0x03")

activate_bus(TCA9545_CONFIG_BUS0)
as3935pin = 16 
GPIO.setup(as3935pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.add_event_detect(as3935pin, GPIO.RISING, callback=handle_as3935_interrupt)



################################################################################
# SHT30 Temperature/Humidity Sensor

try:
  print("Detecting SHT30...")
  import SHT30
  activate_bus(TCA9545_CONFIG_BUS0)
  sht30 = SHT30.SHT30(powerpin=config.SHT30GSPIN)
  outsideHumidity, outsideTemperature, crc_checkH, crc_checkT = sht30.fast_read_humidity_temperature_crc() 

  if (crc_checkH == -1) or (crc_checkT == -1):
    config.SHT30_Present = False
    print("SHT30 Not Present")
  else:
    config.SHT30_Present = True
    state.currentOutsideTemperature = outsideTemperature
    state.currentOutsideHumidity = outsideHumidity
    print("SHT30 Temperature......: %0.1f C" % outsideTemperature)
    print("SHT30 Humidity.........: %0.1f %%" % outsideHumidity)
    print("SHT30 Temperature CRC..: 0x%02x" % crc_checkT)
    print("SHT30 Humidity CRC.....: 0x%02x" % crc_checkH)
    print("SHT30 Present")

except:
  config.SHT30_Present = False
  print("SHT30 Not Present")



################################################################################
# AM2315 Temperature/Humidity Sensor

activate_bus(TCA9545_CONFIG_BUS0)
if not config.SHT30_Present:
  try:
    print("Detecting AM2315...")
    import AM2315
    am2315 = AM2315.AM2315(powerpin=config.AM2315GSPIN)
    am2315.powerCycleAM2315()
    outsideHumidity, outsideTemperature, crc_check = am2315.read_humidity_temperature_crc() 
    #outsideHumidity, outsideTemperature, crc_check = am2315.fast_read_humidity_temperature_crc() 
    if (crc_check == -1):
      config.AM2315_Present = False
      print("AM2315 Not Present")
    else:
      config.AM2315_Present = True
      state.currentOutsideTemperature = outsideTemperature
      state.currentOutsideHumidity = outsideHumidity
      print("AM2315 Temperature..: %0.1f C" % outsideTemperature)
      print("AM2315 Humidity.....: %0.1f %%" % outsideHumidity)
      print("AM2315 CRC..........: 0x%02x" % crc_check)
      print("AM2315 Present")

  except:
    config.AM2315_Present = False
    print("AM2315 Not Present")


# Main Program


# sample weather 
totalRain = 0
def sampleWeather():
  global currentWindSpeed, currentWindGust, totalRain 
  global bmp180Temperature, bmp180Pressure, bmp180Altitude, bmp180SeaLevel 
  global outsideTemperature, outsideHumidity, crc_check 
  global currentWindDirection, currentWindDirectionVoltage
  global SunlightVisible, SunlightIR, SunlightUV, SunlightUVIndex 
  global HTUtemperature, HTUhumidity, rain60Minutes
  global am2315, Sunlight_Sensor

  print("-" * 80)
  print(" Weather Sampling")
  print("-" * 80)

  activate_bus(TCA9545_CONFIG_BUS0)
  SDL_INTERRUPT_CLICKS = 1

  currentWindSpeed = weatherStation.current_wind_speed()
  currentWindGust = weatherStation.get_wind_gust()
  totalRain = totalRain + weatherStation.get_current_rain_total()/SDL_INTERRUPT_CLICKS
  if config.ADS1015_Present or config.ADS1115_Present:
    currentWindDirection = weatherStation.current_wind_direction()
    currentWindDirectionVoltage = weatherStation.current_wind_direction_voltage()

  if config.BMP280_Present:
    try:
      bmp180Temperature = bmp280.read_temperature()
      bmp180Pressure = bmp280.read_pressure()/1000
      if (config.GPS_Present):
        bmp180Altitude = gpsAltitude()
      else:
        bmp180Altitude = bmp280.read_altitude()
      bmp180SeaLevel = bmp280.read_sealevel_pressure(config.BMP280_Altitude_Meters)/1000
    except:
      print_exception()

  if config.BME680_Present:	
    try:
      data = bme680.get_sensor_data()
      bmp180Temperature = bme680.data.temperature
      bmp180Humidity = bme680.data.humidity
      bmp180Pressure = bme680.data.pressure
      if (config.GPS_Present):
        bmp180Altitude = gpsAltitude()
      else:
        bmp180Altitude = config.BMP280_Altitude_Meters 
      bmp180SeaLevel = BME680_Functions.getSeaLevelPressure(config.BMP280_Altitude_Meters, bmp180Pressure)
      # reset read pressure to Sea Level
      #bmp180Pressure = bmp180SeaLevel 
    except:
      print_exception()

  HTUtemperature = 0.0
  HTUhumidity = 0.0

  if config.HDC1080_Present:
    activate_bus(TCA9545_CONFIG_BUS0)

    HTUtemperature = hdc1080.readTemperature() 
    HTUhumidity =  hdc1080.readHumidity()
  else:
    HTUtemperature = bmp180Temperature
    HTUhumidity =  bmp180Humidity

  # use TSL2591 first
  if config.TSL2591_Present:
    activate_bus(TCA9545_CONFIG_BUS3)

    full, ir = tsl2591.get_full_luminosity()  # read raw values (full spectrum and ir spectrum)
    lux = tsl2591.calculate_lux(full, ir)  # convert raw values to lux
    SunlightVisible = lux
    SunlightIR = ir 
    SunlightUV = 0
    SunlightUVIndex = 0.0
  else:
    if config.SI1145_Present:
      activate_bus(TCA9545_CONFIG_BUS3)

      visible = Sunlight_Sensor.readVisible()
      #while visible == 0:
      #  Sunlight_Sensor = SDL_Pi_SI1145.SDL_Pi_SI1145(indoor=0)
      #  time.sleep(5.0)
      #  visible = Sunlight_Sensor.readVisible()
      #  print "visible=", visible
      SunlightVisible = SI1145Lux.SI1145_VIS_to_Lux(visible)
      SunlightIR = SI1145Lux.SI1145_IR_to_Lux(Sunlight_Sensor.readIR())
      SunlightUV = Sunlight_Sensor.readUV()
      SunlightUVIndex = SunlightUV / 100.0
    else:
      SunlightVisible = 0
      SunlightIR = 0 
      SunlightUV = 0
      SunlightUVIndex = 0.0

  # if both AM2315 and SHT30 are present, SHT30 wins
  if config.AM2315_Present and not config.SHT30_Present:
    activate_bus(TCA9545_CONFIG_BUS0)

    try:
      ToutsideHumidity, ToutsideTemperature, crc_check = am2315.read_humidity_temperature_crc()
    except:
      if am2315 is None:
        am2315 = AM2315.AM2315(powerpin=config.AM2315GSPIN)
        print ("AM2315 None Error Detected")
      crc_check = -1

    if (crc_check != -1):
      outsideTemperature = ToutsideTemperature
      outsideHumidity = ToutsideHumidity
      state.currentOutsideTemperature = outsideTemperature
      state.currentOutsideHumidity = outsideHumidity
    print "AM2315 Stats: (g,br,bc,rt,pc)", am2315.read_status_info()

  # if both AM2315 and SHT30 are present, SHT30 wins
  if config.SHT30_Present:
    activate_bus(TCA9545_CONFIG_BUS0)

    ToutsideHumidity, ToutsideTemperature, crc_checkH, crc_checkT = sht30.read_humidity_temperature_crc()
            
    if (crc_checkH != -1) and (crc_checkT != -1):
      outsideTemperature = ToutsideTemperature
      outsideHumidity = ToutsideHumidity
      state.currentOutsideTemperature = outsideTemperature
      state.currentOutsideHumidity = outsideHumidity
    print "SHT30 Stats: (g,br,bc,rt,pc)", sht30.read_status_info()

  sampleINA3221()

  # Weather Variables
  state.currentOutsideTemperature = outsideTemperature 
  state.currentOutsideHumidity = outsideHumidity 

  state.currentInsideTemperature = bmp180Temperature
  state.currentInsideHumidity = bmp180Humidity 

  state.currentRain60Minutes =  rain60Minutes

  state.currentSunlightVisible = SunlightVisible
  state.currentSunlightIR = SunlightIR
  state.currentSunlightUV = SunlightUV
  state.currentSunlightUVIndex  = SunlightUVIndex

  state.ScurrentWindSpeed = currentWindSpeed
  state.ScurrentWindGust  = currentWindGust
  state.ScurrentWindDirection  = currentWindDirection
  state.currentTotalRain  = totalRain

  state.currentBarometricPressure = bmp180Pressure 

  state.currentAltitude = bmp180Altitude
  state.currentSeaLevel = bmp180SeaLevel


  # check for turn fan on
  if (state.currentInsideTemperature > TEMPFANTURNON):
    turnFanOn()
  # check for turn fan off
  if (state.currentInsideTemperature < TEMPFANTURNOFF):
    turnFanOff()

  # turn I2CBus 0 on
  activate_bus(TCA9545_CONFIG_BUS0)


def sampleINA3221():
  global batteryVoltage, batteryCurrent, solarVoltage, solarCurrent, loadVoltage, loadCurrent
  global batteryPower, solarPower, loadPower, batteryCharge

  if (config.INA3221_Present):
    activate_bus(TCA9545_CONFIG_BUS2)

    print("-" * 80)
    print(" INA3221 Sampling")
    print("-" * 80)
    
    busvoltage1 = ina3221.getBusVoltage_V(LIPO_BATTERY_CHANNEL)
    shuntvoltage1 = ina3221.getShuntVoltage_mV(LIPO_BATTERY_CHANNEL)
    # minus is to get the "sense" right.   - means the battery is charging, + that it is discharging
    batteryCurrent = ina3221.getCurrent_mA(LIPO_BATTERY_CHANNEL)
    batteryVoltage = busvoltage1 + (shuntvoltage1 / 1000)
    batteryPower = batteryVoltage * (batteryCurrent/1000)


    busvoltage2 = ina3221.getBusVoltage_V(SOLAR_CELL_CHANNEL)
    shuntvoltage2 = ina3221.getShuntVoltage_mV(SOLAR_CELL_CHANNEL)
    solarCurrent = ina3221.getCurrent_mA(SOLAR_CELL_CHANNEL)
    solarVoltage = busvoltage2 + (shuntvoltage2 / 1000)
    solarPower = solarVoltage * (solarCurrent/1000)

    busvoltage3 = ina3221.getBusVoltage_V(OUTPUT_CHANNEL)
    shuntvoltage3 = ina3221.getShuntVoltage_mV(OUTPUT_CHANNEL)
    loadCurrent = ina3221.getCurrent_mA(OUTPUT_CHANNEL)
    loadVoltage = busvoltage3 
    loadPower = loadVoltage * (loadCurrent/1000)

    batteryCharge = util.returnPercentLeftInBattery(batteryVoltage, 4.19)	

    state.batteryVoltage = batteryVoltage 
    state.batteryCurrent = batteryCurrent
    state.solarVoltage = solarVoltage
    state.solarCurrent = solarCurrent
    state.loadVoltage = loadVoltage
    state.loadCurrent = loadCurrent
    state.batteryPower = batteryPower
    state.solarPower = solarPower
    state.loadPower = loadPower
    state.batteryCharge = batteryCharge
  else:
    print("-" * 80)
    print(" INA3221 Not Present")
    print("-" * 80)


def sampleAndDisplay():
  global currentWindSpeed, currentWindGust, totalRain
  global bmp180Temperature, bmp180Pressure, bmp180Altitude, bmp180SeaLevel
  global outsideTemperature, outsideHumidity, crc_check
  global currentWindDirection, currentWindDirectionVoltage
  global HTUtemperature, HTUhumidity
  global SunlightVisible, SunlightIR, SunlightUV,  SunlightUVIndex 

  I2C_Lock.acquire()

  try:
    print("-" * 80)
    print(" Sample and Display")
    print("-" * 80)

    sampleWeather()

    state.pastBarometricReading = state.currentBarometricPressure

    if config.OLED_Present and state.runOLED:
      Scroll_SSD1306.addLineOLED(display,  ("Wind Speed=\t%0.2f MPH")%(currentWindSpeed/1.6))
      Scroll_SSD1306.addLineOLED(display,  ("Rain Total=\t%0.2f in")%(totalRain/25.4))
      if (config.ADS1015_Present or config.ADS1115_Present):
        Scroll_SSD1306.addLineOLED(display,  "Wind Dir=%0.2f Degrees" % weatherStation.current_wind_direction())

    if config.DS3231_Present:
      currenttime = datetime.utcnow()
      deltatime = currenttime - starttime

      print("Raspberry Pi........: %s" % time.strftime("%Y-%m-%d %H:%M:%S"))
      print("DS3231..............: %s" % ds3231.read_datetime())
      print("DS3231 Temperature..: %0.2f C" % ds3231.getTemp())
      if (config.OLED_Present) and (state.runOLED):
        Scroll_SSD1306.addLineOLED(display,"%s" % ds3231.read_datetime())

    if (config.HDC1080_Present):
      if (config.OLED_Present) and (state.runOLED):
        Scroll_SSD1306.addLineOLED(display,  "InTemp = \t%0.2f C" % HTUtemperature)

    if config.AS3935_Present:
      print("AS3935 Lightning Detector Present")
    else:
      print("AS3935 Lightning Detector Not Present")

    print("-" * 80)

    state.printState()

    f = open('/sys/class/thermal/thermal_zone0/temp', 'r')
    line = f.readline()
    f.close()
    cpu_temp = float(line) / 1000

    topic = 'skyweather/state'
    #strftime( '%Y-%m-%d %H:%M:%S'),
    now_utc = datetime.utcnow()
    now_tz = now_utc.replace(tzinfo=pytz.utc).astimezone(timezone)
    stateMessage = {
      'lastMainReading': now_tz.isoformat(),
      'cpuTemp': cpu_temp,
      'outsideTemperature': state.currentOutsideTemperature,
      'outsideHumidity': state.currentOutsideHumidity,

      'insideTemperature': state.currentInsideTemperature,
      'insideHumidity': state.currentInsideHumidity,

      'rain60Minutes': state.currentRain60Minutes,

      'sunlightVisible': state.currentSunlightVisible,
      'sunlightIR': state.currentSunlightIR,
      'sunlightUV': state.currentSunlightUV,
      'sunlightUVIndex': state.currentSunlightUVIndex,

      'currentWindSpeed': state.ScurrentWindSpeed,
      'currentWindGust': state.ScurrentWindGust,
      'currentWindDirection': state.ScurrentWindDirection,
      'totalRain': state.currentTotalRain,

      'currentPressure': state.currentBarometricPressure,
      'currentAltitude': state.currentAltitude,
      'currentPressureSeaLevel': state.currentSeaLevel,

      'AQI': state.Outdoor_AirQuality_Sensor_Value,

      'fanState': state.fanState,

      'batteryVoltage': state.batteryVoltage,
      'batteryCurrent': state.batteryCurrent,
      'solarVoltage': state.solarVoltage,
      'solarCurrent': state.solarCurrent,
      'loadVoltage': state.loadVoltage,
      'loadCurrent': state.loadCurrent,
      'batteryPower': state.batteryPower,
      'solarPower': state.solarPower,
      'loadPower': state.loadPower,
      'batteryCharge': state.batteryCharge,
    }
    mqtt_publish(topic, stateMessage)

    with open("state.json", "w") as f:
      json.dump(stateMessage, f, indent=4)

    print("-" * 80)
    print(" Sample and Display Done")
    print("-" * 80)

  except:
    print_exception()

  I2C_Lock.release()
  gc.collect()


def writeWeatherRecord():
  global as3935LightningCount
  global as3935, as3935LastInterrupt, as3935LastDistance, as3935LastStatus
  global currentWindSpeed, currentWindGust, totalRain 
  global bmp180Temperature, bmp180Pressure, bmp180Altitude, bmp180SeaLevel 
  global outsideTemperature, outsideHumidity, crc_check 
  global currentWindDirection, currentWindDirectionVoltage
  global SunlightVisible, SunlightIR, SunlightUV, SunlightUVIndex 
  global HTUtemperature, HTUhumidity

# now we have the data, stuff it in the database
  try:
    con = mdb.connect(config.MySQL_Host, config.MySQL_Username, config.MySQL_Password, 'SkyWeather');
    cur = con.cursor()
    #query = 'INSERT INTO WeatherData(TimeStamp,as3935LightningCount, as3935LastInterrupt, as3935LastDistance, as3935LastStatus, currentWindSpeed, currentWindGust, totalRain,  bmp180Temperature, bmp180Pressure, bmp180Altitude,  bmp180SeaLevel,  outsideTemperature, outsideHumidity, currentWindDirection, currentWindDirectionVoltage, insideTemperature, insideHumidity, AQI) VALUES(UTC_TIMESTAMP(), %.3f, %.3f, %.3f, "%s", %.3f, %.3f, %.3f, %i, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f)' % (as3935LightningCount, as3935LastInterrupt, as3935LastDistance, as3935LastStatus, currentWindSpeed, currentWindGust, totalRain,  bmp180Temperature, bmp180Pressure, bmp180Altitude,  bmp180SeaLevel,  outsideTemperature, outsideHumidity, currentWindDirection, currentWindDirectionVoltage, HTUtemperature, HTUhumidity, state.Outdoor_AirQuality_Sensor_Value)
    query = 'INSERT INTO WeatherData(TimeStamp,as3935LightningCount, as3935LastInterrupt, as3935LastDistance, as3935LastStatus, currentWindSpeed, currentWindGust, totalRain,  bmp180Temperature, bmp180Pressure, bmp180Altitude,  bmp180SeaLevel,  outsideTemperature, outsideHumidity, currentWindDirection, currentWindDirectionVoltage, insideTemperature, insideHumidity, AQI) VALUES(UTC_TIMESTAMP(), %.3f, %.3f, %.3f, "%s", %.3f, %.3f, %.3f, %i, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f)' % (as3935LightningCount, as3935LastInterrupt, as3935LastDistance, as3935LastStatus, state.ScurrentWindSpeed, state.ScurrentWindGust, state.currentTotalRain,  state.currentInsideTemperature, state.currentBarometricPressure, state.currentAltitude,  state.currentSeaLevel,  state.currentOutsideTemperature, state.currentOutsideHumidity, state.ScurrentWindDirection, currentWindDirectionVoltage, state.currentInsideTemperature, state.currentInsideHumidity, state.Outdoor_AirQuality_Sensor_Value)
    print("query = %s" % query)

    cur.execute(query)
    as3935LastDistance = 0

    # now check for TSL2591 Sensor
    if (config.TSL2591_Present):
      query = 'INSERT INTO Sunlight(TimeStamp, Visible, IR, UV, UVIndex) VALUES(UTC_TIMESTAMP(), %d, %d, %d, %.3f)' % (SunlightVisible, SunlightIR, SunlightUV, SunlightUVIndex)
      print("query = %s" % query)
      cur.execute(query)

    # now check for Sunlight Sensor
    if (config.SI1145_Present):
      query = 'INSERT INTO Sunlight(TimeStamp, Visible, IR, UV, UVIndex) VALUES(UTC_TIMESTAMP(), %d, %d, %d, %.3f)' % (SunlightVisible, SunlightIR, SunlightUV, SunlightUVIndex)
      print("query = %s" % query)
      cur.execute(query)

    con.commit()

  except mdb.Error, e:
    print_exception()
    con.rollback()
  
  finally:    
    cur.close() 
    con.close()
    del cur
    del con


def writePowerRecord():
  # now we have the data, stuff it in the database
  try:
    con = mdb.connect(config.MySQL_Host, config.MySQL_Username, config.MySQL_Password, 'SkyWeather');
    cur = con.cursor()
    query = 'INSERT INTO PowerSystem(TimeStamp, batteryVoltage, batteryCurrent, solarVoltage, solarCurrent, loadVoltage, loadCurrent, batteryPower, solarPower, loadPower, batteryCharge) VALUES (UTC_TIMESTAMP (), %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f)' % (state.batteryVoltage, state.batteryCurrent, state.solarVoltage, state.solarCurrent, state.loadVoltage, state.loadCurrent, state.batteryPower, state.solarPower, state.loadPower, state.batteryCharge) 
    print("query = %s" % query)

    cur.execute(query)
    con.commit()

  except mdb.Error, e:
    print_exception()
    con.rollback()
  
  finally:    
    cur.close() 
    con.close()
    del cur
    del con


def shutdownPi(why):
  pclogging.log(pclogging.INFO, __name__, "Pi Shutting Down: %s" % why)
  time.sleep(10.0)
  os.system("sudo shutdown -h now")


def rebootPi(why):
  pclogging.log(pclogging.INFO, __name__, "Pi Rebooting: %s" % why)
  pclogging.log(pclogging.INFO, __name__, "Pi Rebooting: %s" % why)
  os.system("sudo shutdown -r now")

# print out faults inside events
def ap_my_listener(event):
  if event.exception:
    print event.exception
    print event.traceback

# apscheduler events
def tick():
  print('Tick! The time is: %s' % datetime.now())
  gc.collect()


def killLogger():
  scheduler.shutdown()
  print "Scheduler Shutdown...."
  exit()


def checkForShutdown():
  if (batteryVoltage < 3.5):
    print "--->>>>Time to Shutdown<<<<---"
    shutdownPi("low voltage shutdown")


# Rain calculations
rainArray = []
for i in range(RAIN_ARRAY_SIZE):
  rainArray.append(0)

lastRainReading = 0.0


def addRainToArray(plusRain):
  global rainArray

  del rainArray[0]
  rainArray.append(plusRain)


def totalRainArray():
  global rainArray

  total = 0
  for i in range(RAIN_ARRAY_SIZE):
    total = total+rainArray[i]
  return total


def updateRain():
  global lastRainReading, rain60Minutes, totalRain

  addRainToArray(totalRain - lastRainReading)	
  rain60Minutes = totalRainArray()
  lastRainReading = totalRain


def barometricTrend():
  if (state.currentBarometricPressure >= state.pastBarometricReading):
    state.barometricTrend = True
  else:
    state.barometricTrend = False
  state.pastBarometricReading = state.currentBarometricPressure


print("=" * 80)
print("SkyWeather Weather Station Version "+config.SWVERSION+" - SwitchDoc Labs")
print("")
print("")
print("Program Started at:"+ time.strftime("%Y-%m-%d %H:%M:%S"))
print("")


def read_AQI():
  I2C_Lock.acquire()
  try:
    DustSensor.read_AQI()
  except:
    print_exception()
  I2C_Lock.release()

	
###############
#  Turn Dust Sensor Off
################
try:
  read_AQI()
  config.HM3301_Present = True
except:
  print_exception()
#patTheDog()



# Initialize Variables
bmp180Temperature =  0
bmp180Pressure = 0 
bmp180Altitude = 0
bmp180SeaLevel = 0 
bmp180Humidity = 0


print("=" * 80)
print(returnStatusLine("TCA9545", config.TCA9545_I2CMux_Present))
print(returnStatusLine("BME680", config.BME680_Present))
print(returnStatusLine("BMP280", config.BMP280_Present))
print(returnStatusLine("DS3231", config.DS3231_Present))
print(returnStatusLine("HDC1080", config.HDC1080_Present))
print(returnStatusLine("SHT30", config.SHT30_Present))
print(returnStatusLine("AM2315", config.AM2315_Present))
print(returnStatusLine("ADS1015", config.ADS1015_Present))
print(returnStatusLine("ADS1115", config.ADS1115_Present))
print(returnStatusLine("AS3935", config.AS3935_Present))
print(returnStatusLine("GPS", config.GPS_Present))
print(returnStatusLine("OLED", config.OLED_Present))
print(returnStatusLine("INA3221", config.INA3221_Present))
print(returnStatusLine("SI1145", config.SI1145_Present))
print(returnStatusLine("TSL2591", config.TSL2591_Present))
print(returnStatusLine("HM3301", config.HM3301_Present))
print("")
print(returnStatusLine("UseMySQL", config.enable_MySQL_Logging))
print("=" * 80)



# initialize appropriate weather variables
currentWindDirection = 0
currentWindDirectionVoltage = 0.0
rain60Minutes = 0.0

pclogging.log(pclogging.INFO, __name__, "SkyWeather Startup Version"+config.SWVERSION )


# Initial Sample And Display
sampleAndDisplay()

# Set up scheduler
scheduler = BackgroundScheduler()

# for debugging
scheduler.add_listener(ap_my_listener, apscheduler.events.EVENT_JOB_ERROR)

##############
# setup tasks
##############

# prints out the date and time to console
scheduler.add_job(tick, 'interval', seconds=60)

# sample and Watchdog jobs
#scheduler.add_job(patTheDog, 'interval', seconds=10)   # reset the WatchDog Timer

scheduler.add_job(sampleAndDisplay, 'interval', seconds=60)

# every 5 minutes, push data to mysql and check for shutdown
if config.enable_MySQL_Logging:
	scheduler.add_job(writeWeatherRecord, 'interval', seconds=5*60)
	scheduler.add_job(writePowerRecord, 'interval', seconds=5*60)

scheduler.add_job(updateRain, 'interval', seconds=int((60/RAIN_ARRAY_SIZE)*60))

#scheduler.add_job(checkForShutdown, 'interval', seconds=5*60)

# every 5 days at 00:04, reboot
#scheduler.add_job(rebootPi, 'cron', day='5-30/5', hour=0, minute=4, args=["5 day Reboot"]) 
	
#check for Barometric Trend (every 15 minutes)
scheduler.add_job(barometricTrend, 'interval', seconds=15*60)

if config.HM3301_Present:
  scheduler.add_job(read_AQI, 'interval', seconds=15*60)

# start scheduler
scheduler.start()
print("-" * 80)
print("Scheduled Jobs")
print("-" * 80)
scheduler.print_jobs()
print("-" * 80)


if not config.INA3221_Present:
  batteryCurrent = 0.0
  batteryVoltage = 4.00 
  batteryPower = batteryVoltage * (batteryCurrent/1000)

  solarCurrent = 0.0 
  solarVoltage = 0.0 
  solarPower = solarVoltage * (solarCurrent/1000)

  loadCurrent = 0.0
  loadVoltage = 0.0 
  loadPower = loadVoltage * (loadCurrent/1000)

  batteryCharge = 0 

#  Main Loop
try:
  while True:
    time.sleep(60)
except:
  scheduler.shutdown()
