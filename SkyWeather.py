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


import subprocess
import RPi.GPIO as GPIO
import smbus

import struct

import SDL_Pi_HDC1000


from apscheduler.schedulers.background import BackgroundScheduler

import apscheduler.events

if (config.enable_MySQL_Logging == True):
	import MySQLdb as mdb

import DustSensor

#import GroveGPS

import util

################
# Device Present State Variables
###############

config.TCA9545_I2CMux_Present = False
config.SunAirPlus_Present = False
config.AS3935_Present = False
config.DS3231_Present = False
config.BMP280_Present = False
config.BME680_Present = False
config.AM2315_Present = False
config.ADS1015_Present = False
config.ADS1115_Present = False
config.OLED_Present = False
config.Sunlight_Present = False
config.TSL2591_Present = False
config.SolarMax_Present = False
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


def patTheDog():
    # pat the dog
    print "------Patting The Dog------- "
    GPIO.setup(config.WATCHDOGTRIGGER, GPIO.OUT)
    GPIO.output(config.WATCHDOGTRIGGER, False)
    time.sleep(0.2)
    GPIO.output(config.WATCHDOGTRIGGER, True)
    GPIO.setup(config.WATCHDOGTRIGGER, GPIO.IN)

#patTheDog()

state.mqtt_client = mqtt.Client(client_id="SkyWeather")
state.mqtt_client.username_pw_set(config.MQTT_Username, config.MQTT_Password)
state.mqtt_client.connect(config.MQTT_ServerURL, port=config.MQTT_Port)
state.mqtt_client.loop_start()
	

################
# TCA9545 I2C Mux 

#/*=========================================================================
#    I2C ADDRESS/BITS
#    -----------------------------------------------------------------------*/
TCA9545_ADDRESS =                         (0x73)    # 1110011 (A0+A1=VDD)
#/*=========================================================================*/

#/*=========================================================================
#    CONFIG REGISTER (R/W)
#    -----------------------------------------------------------------------*/
TCA9545_REG_CONFIG            =          (0x00)
#    /*---------------------------------------------------------------------*/

TCA9545_CONFIG_BUS0  =                (0x01)  # 1 = enable, 0 = disable 
TCA9545_CONFIG_BUS1  =                (0x02)  # 1 = enable, 0 = disable 
TCA9545_CONFIG_BUS2  =                (0x04)  # 1 = enable, 0 = disable 
TCA9545_CONFIG_BUS3  =                (0x08)  # 1 = enable, 0 = disable 

#/*=========================================================================*/

BUS_MAP = {
    TCA9545_CONFIG_BUS0: 0,
    TCA9545_CONFIG_BUS1: 1,
    TCA9545_CONFIG_BUS2: 2,
    TCA9545_CONFIG_BUS3: 3
}

# I2C Mux TCA9545 Detection
try:
    tca9545 = SDL_Pi_TCA9545.SDL_Pi_TCA9545(addr=TCA9545_ADDRESS, bus_enable = TCA9545_CONFIG_BUS0)

    # turn I2CBus 1 on
    tca9545.write_control_register(TCA9545_CONFIG_BUS2)
    config.TCA9545_I2CMux_Present = True
except:
    print ">>>>>>>>>>>>>>>>>>><<<<<<<<<<<"
    print "TCA9545 I2C Mux Not Present" 
    print ">>>>>>>>>>>>>>>>>>><<<<<<<<<<<"
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
                time.sleep(1)
        except Exception as e:
            print(traceback.format_exc())
            print(e)
            print("TCA9545: Activating BUS%d" % BUS_MAP[requested_bus])
            try:
                tca9545.write_control_register(requested_bus)
                time.sleep(1)
            except Exception as e:
                print(traceback.format_exc())
                print(e)

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
                time.sleep(1)

        except Exception as e:
            print(traceback.format_exc())
            print(e)
            try:
                print("TCA9545: Restoring BUS%d" % BUS_MAP[previous_bus])
                tca9545.write_control_register(previous_bus)
                time.sleep(1)

            except Exception as e:
                print(traceback.format_exc())
                print(e)


def removePower(GroveSavePin):
    GPIO.setup(GroveSavePin, GPIO.OUT)
    GPIO.output(GroveSavePin, False)
        

def restorePower(GroveSavePin):
    GPIO.setup(GroveSavePin, GPIO.OUT)
    GPIO.output(GroveSavePin, True)
   
def togglePower(GroveSavePin):
    print("Toggling Power to Pin=", GroveSavePin)
    removePower(GroveSavePin)
    time.sleep(4.5)
    restorePower(GroveSavePin)


###############
# Fan Control
###############

import SDL_Pi_GrovePowerDrive

TEMPFANTURNON = 37.0
TEMPFANTURNOFF = 34.0

myPowerDrive = SDL_Pi_GrovePowerDrive.SDL_Pi_GrovePowerDrive(config.GPIO_Pin_PowerDrive_Sig1, config.GPIO_Pin_PowerDrive_Sig2, False, False)

def turnFanOn():
    if (state.fanState == False):
        pclogging.log(pclogging.INFO, __name__, "Turning Fan On" )
        myPowerDrive.setPowerDrive(1, True) 
        myPowerDrive.setPowerDrive(2, True) 
        state.fanState = True

def turnFanOff():
    if (state.fanState == True):
        pclogging.log(pclogging.INFO, __name__, "Turning Fan Off" )
        myPowerDrive.setPowerDrive(1, False) 
        myPowerDrive.setPowerDrive(2, False)
        state.fanState = False
 
turnFanOff()


###############
# TSL2591 Sunlight Sensor Setup
################

# turn I2CBus 3 on
activate_bus(TCA9545_CONFIG_BUS3)

try:
    tsl2591 = TSL2591.Tsl2591()
    int_time=TSL2591.INTEGRATIONTIME_100MS
    gain=TSL2591.GAIN_LOW
    tsl2591.set_gain(gain)
    tsl2591.set_timing(int_time)
    full, ir = tsl2591.get_full_luminosity()  # read raw values (full spectrum and ir spectrum)
    lux = tsl2591.calculate_lux(full, ir)  # convert raw values to lux
    print("TSL2591: Lux: %d, Full: %d, IR: %d" % (lux, full, ir))
    config.TSL2591_Present = True 

except:
    config.TSL2591_Present = False 


###############

# Sunlight SI1145 Sensor Setup

################
# turn I2CBus 3 on
activate_bus(TCA9545_CONFIG_BUS3)

try:
    #restorePower(SI1145GSPIN)
    time.sleep(1.0)
    Sunlight_Sensor = SDL_Pi_SI1145.SDL_Pi_SI1145(indoor=0)
    time.sleep(1.0)

    config.Sunlight_Present = True
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
    print("SI1145: Visible: %d, Lux: %d, IR Lux: %d, UV Index: %f" % (visible, vis_Lux, IR_Lux, uvIndex))



except:
    config.Sunlight_Present = False

def returnStatusLine(device, state):
    returnString = device
    if (state == True):
        returnString = returnString + ":\t\tPresent"
    else:
        returnString = returnString + ":\t\tNot Present"
    return returnString



################
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
    except Exception as e:
        print(traceback.format_exc())
        print(e)

#try:
#    gps = GroveGPS.GPS()
#    gps.read()
#    gpsAltitude()
#    config.GPS_Present = True
#except:
#    config.GPS_Present = False

config.GPS_Present = False
print("Past GPS")


# semaphore primitives for preventing I2C conflicts


I2C_Lock = threading.Lock()

################
# SunAirPlus Sensors


# the three channels of the INA3221 named for SunAirPlus Solar Power Controller channels (www.switchdoc.com)
LIPO_BATTERY_CHANNEL = 1
SOLAR_CELL_CHANNEL   = 2
OUTPUT_CHANNEL       = 3

try:
    activate_bus(TCA9545_CONFIG_BUS2)
    sunAirPlus = SDL_Pi_INA3221.SDL_Pi_INA3221(addr=0x40)
    busvoltage1 = sunAirPlus.getBusVoltage_V(LIPO_BATTERY_CHANNEL)
    config.SunAirPlus_Present = True
except:
    config.SunAirPlus_Present = False



################
# turn I2CBus 0 on
activate_bus(TCA9545_CONFIG_BUS0)

# Check for HDC1080 first (both are on 0x40)


###############

# HDC1080 Detection
try:
    hdc1080 = SDL_Pi_HDC1000.SDL_Pi_HDC1000() 
    deviceID = hdc1080.readDeviceID() 
    print "deviceID = 0x%X" % deviceID
    if (deviceID == 0x1050):
        config.HDC1080_Present = True
    else:
        config.HDC1080_Present = False
except:
    config.HDC1080_Present = False


###############

#WeatherRack Weather Sensors
#
# GPIO Numbering Mode GPIO.BCM
#


# constants

SDL_MODE_INTERNAL_AD = 0
SDL_MODE_I2C_ADS1015 = 1    # internally, the library checks for ADS1115 or ADS1015 if found

#sample mode means return immediately.  THe wind speed is averaged at sampleTime or when you ask, whichever is longer
SDL_MODE_SAMPLE = 0
#Delay mode means to wait for sampleTime and the average after that time.
SDL_MODE_DELAY = 1

# turn I2CBus 0 on
activate_bus(TCA9545_CONFIG_BUS0)
weatherStation = SDL_Pi_WeatherRack.SDL_Pi_WeatherRack(config.anemometerPin, config.rainPin, 0,0, SDL_MODE_I2C_ADS1015)

weatherStation.setWindMode(SDL_MODE_SAMPLE, 5.0)
#weatherStation.setWindMode(SDL_MODE_DELAY, 5.0)


################
# DS3231/AT24C32 Setup
# turn I2CBus 0 on
activate_bus(TCA9545_CONFIG_BUS0)

filename = time.strftime("%Y-%m-%d%H:%M:%SRTCTest") + ".txt"
starttime = datetime.utcnow()

ds3231 = SDL_DS3231.SDL_DS3231(1, 0x68)

try:
    ds3231.read_datetime()
    #print "DS3231=\t\t%s" % ds3231.read_datetime()
    config.DS3231_Present = True

except IOError as e:
    #print "I/O error({0}): {1}".format(e.errno, e.strerror)
    config.DS3231_Present = False

def rtc_save():
    ds3231.write_now()

atexit.register(rtc_save)



################
# BMP280 Setup 
try:
    bmp280 = BMP280.BMP280()
    config.BMP280_Present = True
except: 
    # print "I/O error({0}): {1}".format(e.errno, e.strerror)
    config.BMP280_Present = False


################
# BME680 Setup 
try:
    bme680 = BME680.BME680(BME680.I2C_ADDR_SECONDARY)
    config.BME680_Present = True
    BME680_Functions.setup_bme680(bme680)
except IOError as e:
    print "I/O error({0}): {1}".format(e.errno, e.strerror)
    config.BME680_Present = False

print ("after bme680", config.BME680_Present)


################
# OLED SSD_1306 Detection
try:
    util.turnOLEDOn()
    RST =27
    display = Adafruit_SSD1306.SSD1306_128_64(rst=RST, i2c_address=0x3C)
    # Initialize library.
    display.begin()
    display.clear()
    display.display()
    config.OLED_Present = True
    config.OLED_Originally_Present = True
except Exception as e:
    print(traceback.format_exc())
    print(e)
    config.OLED_Originally_Present = False
    config.OLED_Present = False

try:
    util.turnOLEDOff()
except Exception as e:
    print(traceback.format_exc())
    print(e)

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


################
def process_as3935_interrupt():
    global as3935, as3935LastInterrupt, as3935LastDistance, as3935LastStatus, as3935LightningCount

    print "Processing Interrupt from AS3935"

    try:
        I2C_Lock.acquire()
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

        print "Last Interrupt = 0x%x:  %s" % (as3935LastInterrupt, as3935LastStatus)

        topic = 'skyweather/lightning'
        #now_utc = datetime.utcnow()
        #now_tz = now_utc.replace(tzinfo=pytz.utc).astimezone(timezone)
        stateMessage = {
            'as3935Timestamp': timestamp,
            'as3935LightningCount': as3935LightningCount,
            'as3935LastDistance': as3935LastDistance,
            'as3935LastStatus': as3935LastStatus,
        }
        message = json.dumps(stateMessage)
        print("MQTT(",topic,"): ",message)
        state.mqtt_client.publish(topic, message, retain=True)

        print "----------------- "

    except Exception as e:
        print "Exception: AS3935 Process Interrupt"
        print(traceback.format_exc())
        print(e)

    I2C_Lock.release()




# as3935 Set up Lightning Detector
as3935LastInterrupt = 0
as3935LightningCount = 0
as3935LastDistance = 0
as3935LastStatus = ""

# switch to BUS1 - for low loading ib Base Bus
print "AS3935 Start (0x02)"
activate_bus(TCA9545_CONFIG_BUS1)
as3935 = RPi_AS3935(address=0x02, bus=1)

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
    as3935.set_noise_floor(NoiseFloor)
    as3935.set_indoors(Indoor)
    as3935.calibrate(tun_cap=TuneCap)
    as3935.set_mask_disturber(DisturberDetection)
    as3935.set_watchdog_threshold(WatchDogThreshold)
    as3935.set_spike_detection(SpikeDetection)

    config.AS3935_Present = True
    print "AS3935 Present at 0x02"
    process_as3935_interrupt()

except Exception as e:
    print(traceback.format_exc())
    print(e)

    try:
        print "AS3935 Start (0x03)"
        activate_bus(TCA9545_CONFIG_BUS1)
        as3935 = RPi_AS3935(address=0x03, bus=1)

        as3935.set_noise_floor(NoiseFloor)
        as3935.set_indoors(Indoor)
        as3935.calibrate(tun_cap=TuneCap)
        as3935.set_mask_disturber(DisturberDetection)
        as3935.set_watchdog_threshold(WatchDogThreshold)
        as3935.set_spike_detection(SpikeDetection)

        config.AS3935_Present = True
        print "AS3935 Present at 0x03"

    except Exception as e:
        print(traceback.format_exc())
        print(e)
        config.AS3935_Present = False

# back to BUS0
activate_bus(TCA9545_CONFIG_BUS0)


def handle_as3935_interrupt(channel):
    print "AS3935 Interrupt"
    try:
        process_as3935_interrupt()
    except Exception as e:
        print(traceback.format_exc())
        print(e)


# define Interrupt Pin for AS3935
as3935pin = 16 

#GPIO.setup(as3935pin, GPIO.IN)
GPIO.setup(as3935pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.add_event_detect(as3935pin, GPIO.RISING, callback=handle_as3935_interrupt)


##############
# Setup SHT30
# turn I2CBus 0 on
activate_bus(TCA9545_CONFIG_BUS0)

# Grove Power Save Pins for device reset
###############

# Detect SHT30
outsideHumidity = 0.0
outsideTemperature = 0.0
crc_check = -1
import SHT30
try:
    sht30 = SHT30.SHT30(powerpin=config.SHT30GSPIN)
    outsideHumidity, outsideTemperature, crc_checkH, crc_checkT = sht30.fast_read_humidity_temperature_crc() 
	
    print "outsideTemperature: %0.1f C" % outsideTemperature
    print "outsideHumidity: %0.1f %%" % outsideHumidity
    state.currentOutsideTemperature = outsideTemperature
    state.currentOutsideHumidity = outsideHumidity
    print "crcH: 0x%02x" % crc_checkH
    print "crcT 0x%02x" % crc_checkT
    config.SHT30_Present = True
    if (crc_checkH == -1) or (crc_checkT == -1):
        config.SHT30_Present = False

except Exception as e:
    config.SHT30_Present = False
    #print "exception in SHT30 Check"
    #print(traceback.format_exc())
    #print (e)

print "after SHT30"

##############
# Setup AM2315
# turn I2CBus 0 on
activate_bus(TCA9545_CONFIG_BUS0)

# Grove Power Save Pins for device reset

if (config.SHT30_Present == False):  # don't check for AM2315 if you find SHT30

    ###############
    # Detect AM2315
    outsideHumidity = 0.0
    outsideTemperature = 0.0
    crc_check = -1
    import AM2315
    try:
        am2315 = AM2315.AM2315(powerpin=config.AM2315GSPIN)
        am2315.powerCycleAM2315()
        outsideHumidity, outsideTemperature, crc_check = am2315.read_humidity_temperature_crc() 
        #outsideHumidity, outsideTemperature, crc_check = am2315.fast_read_humidity_temperature_crc() 
        print "outsideTemperature: %0.1f C" % outsideTemperature
        print "outsideHumidity: %0.1f %%" % outsideHumidity
        state.currentOutsideTemperature = outsideTemperature
        state.currentOutsideHumidity = outsideHumidity
        print "crc: 0x%02x" % crc_check
        config.AM2315_Present = True
        if (crc_check == -1):
            config.AM2315_Present = False

    except:
        config.AM2315_Present = False


# Main Program


# sample weather 
totalRain = 0
def sampleWeather():
    global currentWindSpeed, currentWindGust, totalRain 
    global bmp180Temperature, bmp180Pressure, bmp180Altitude,  bmp180SeaLevel 
    global outsideTemperature, outsideHumidity, crc_check 
    global currentWindDirection, currentWindDirectionVoltage
    global SunlightVisible, SunlightIR, SunlightUV,  SunlightUVIndex 
    global HTUtemperature, HTUhumidity, rain60Minutes
    global am2315, Sunlight_Sensor


    print "----------------- "
    print " Weather Sampling" 
    print "----------------- "
    #
    # turn I2CBus 0 on
    activate_bus(TCA9545_CONFIG_BUS0)
    SDL_INTERRUPT_CLICKS = 1

    currentWindSpeed = weatherStation.current_wind_speed()
    currentWindGust = weatherStation.get_wind_gust()
    totalRain = totalRain + weatherStation.get_current_rain_total()/SDL_INTERRUPT_CLICKS
    if ((config.ADS1015_Present == True) or (config.ADS1115_Present == True)):
        currentWindDirection = weatherStation.current_wind_direction()
        currentWindDirectionVoltage = weatherStation.current_wind_direction_voltage()

    print "----------------- "
  
    if (config.BMP280_Present):	
        try:
            bmp180Temperature = bmp280.read_temperature()
            bmp180Pressure = bmp280.read_pressure()/1000
            if (config.GPS_Present):
                bmp180Altitude = gpsAltitude()
            else:
                bmp180Altitude = bmp280.read_altitude()
            bmp180SeaLevel = bmp280.read_sealevel_pressure(config.BMP280_Altitude_Meters)/1000
        except:
            print("Unexpected error:", sys.exc_info()[0])


    if (config.BME680_Present):	
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
            print("Unexpected error:", sys.exc_info()[0])


	HTUtemperature = 0.0
	HTUhumidity = 0.0


    if (config.HDC1080_Present):
        HTUtemperature = hdc1080.readTemperature() 
        HTUhumidity =  hdc1080.readHumidity()
    else:
        HTUtemperature = bmp180Temperature
        HTUhumidity =  bmp180Humidity

    # use TSL2591 first

    if (config.TSL2591_Present):
        ################
        # turn I2CBus 3 on
        activate_bus(TCA9545_CONFIG_BUS3)

        full, ir = tsl2591.get_full_luminosity()  # read raw values (full spectrum and ir spectrum)
        lux = tsl2591.calculate_lux(full, ir)  # convert raw values to lux
        SunlightVisible = lux
        SunlightIR = ir 
        SunlightUV = 0
        SunlightUVIndex = 0.0
    else:
        if (config.Sunlight_Present):
            ################
            # turn I2CBus 3 on
            activate_bus(TCA9545_CONFIG_BUS3)

            visible = Sunlight_Sensor.readVisible()
            while visible == 0:
                #patTheDog()
                Sunlight_Sensor = SDL_Pi_SI1145.SDL_Pi_SI1145(indoor=0)
                time.sleep(5.0)
                visible = Sunlight_Sensor.readVisible()
                print "visible=", visible
            SunlightVisible = SI1145Lux.SI1145_VIS_to_Lux(visible)
            SunlightIR = SI1145Lux.SI1145_IR_to_Lux(Sunlight_Sensor.readIR())
            SunlightUV = Sunlight_Sensor.readUV()
            SunlightUVIndex = SunlightUV / 100.0

            ################
            # turn I2CBus 0 on
            activate_bus(TCA9545_CONFIG_BUS0)

        else:
        	SunlightVisible = 0
        	SunlightIR = 0 
        	SunlightUV = 0
        	SunlightUVIndex = 0.0

    # if both AM2315 and SHT30 are present, SHT30 wins
    if (config.AM2315_Present) and (config.SHT30_Present == False):
        # get AM2315 Outside Humidity and Outside Temperature
        # turn I2CBus 0 on
        activate_bus(TCA9545_CONFIG_BUS0)

        try:
            ToutsideHumidity, ToutsideTemperature, crc_check = am2315.read_humidity_temperature_crc()
        except:
            if am2315 is None:
                am2315 = AM2315.AM2315(powerpin=config.AM2315GSPIN )
                print ("am2315 None Error Detected")
            crc_check = -1

        if (crc_check !=  -1):
            outsideTemperature = ToutsideTemperature
            outsideHumidity = ToutsideHumidity
            state.currentOutsideTemperature = outsideTemperature
            state.currentOutsideHumidity = outsideHumidity
        print "AM2315 Stats: (g,br,bc,rt,pc)", am2315.read_status_info()

    # if both AM2315 and SHT30 are present, SHT30 wins
    if (config.SHT30_Present):
        # get SHT30 Outside Humidity and Outside Temperature
        # turn I2CBus 0 on
        activate_bus(TCA9545_CONFIG_BUS0)

        ToutsideHumidity, ToutsideTemperature, crc_checkH, crc_checkT = sht30.read_humidity_temperature_crc()
                
        if (crc_checkH !=  -1) and (crc_checkT != -1):
            outsideTemperature = ToutsideTemperature
            outsideHumidity = ToutsideHumidity
            state.currentOutsideTemperature = outsideTemperature
            state.currentOutsideHumidity = outsideHumidity
        print "SHT30 Stats: (g,br,bc,rt,pc)", sht30.read_status_info()


    sampleSunAirPlus()

    # set State Variables

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


def sampleSunAirPlus():
    global batteryVoltage, batteryCurrent, solarVoltage, solarCurrent, loadVoltage, loadCurrent
    global batteryPower, solarPower, loadPower, batteryCharge

    if (config.SunAirPlus_Present):
        # turn I2CBus 2 on
        activate_bus(TCA9545_CONFIG_BUS2)


        print "----------------- "
        print " SunAirPlus Sampling" 
        print "----------------- "
        

        
        busvoltage1 = sunAirPlus.getBusVoltage_V(LIPO_BATTERY_CHANNEL)
        shuntvoltage1 = sunAirPlus.getShuntVoltage_mV(LIPO_BATTERY_CHANNEL)
        # minus is to get the "sense" right.   - means the battery is charging, + that it is discharging
        batteryCurrent = sunAirPlus.getCurrent_mA(LIPO_BATTERY_CHANNEL)
        batteryVoltage = busvoltage1 + (shuntvoltage1 / 1000)
        batteryPower = batteryVoltage * (batteryCurrent/1000)


        busvoltage2 = sunAirPlus.getBusVoltage_V(SOLAR_CELL_CHANNEL)
        shuntvoltage2 = sunAirPlus.getShuntVoltage_mV(SOLAR_CELL_CHANNEL)
        solarCurrent = sunAirPlus.getCurrent_mA(SOLAR_CELL_CHANNEL)
        solarVoltage = busvoltage2 + (shuntvoltage2 / 1000)
        solarPower = solarVoltage * (solarCurrent/1000)

        busvoltage3 = sunAirPlus.getBusVoltage_V(OUTPUT_CHANNEL)
        shuntvoltage3 = sunAirPlus.getShuntVoltage_mV(OUTPUT_CHANNEL)
        loadCurrent = sunAirPlus.getCurrent_mA(OUTPUT_CHANNEL)
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
        print "----------------- "
        print " SunAirPlus Not Present" 
        print "----------------- "


def sampleAndDisplay():
    global currentWindSpeed, currentWindGust, totalRain
    global bmp180Temperature, bmp180Pressure, bmp180Altitude, bmp180SeaLevel
    global outsideTemperature, outsideHumidity, crc_check
    global currentWindDirection, currentWindDirectionVoltage
    global HTUtemperature, HTUhumidity
    global SunlightVisible, SunlightIR, SunlightUV,  SunlightUVIndex 
    global totalRain

    I2C_Lock.acquire()

    try:

        print "----------------- "
        print " Sample and Display "
        print "----------------- "

        sampleWeather()

        state.pastBarometricReading = state.currentBarometricPressure

        if (config.OLED_Present) and (state.runOLED):
            Scroll_SSD1306.addLineOLED(display,  ("Wind Speed=\t%0.2f MPH")%(currentWindSpeed/1.6))
            Scroll_SSD1306.addLineOLED(display,  ("Rain Total=\t%0.2f in")%(totalRain/25.4))
            if (config.ADS1015_Present or config.ADS1115_Present):
                Scroll_SSD1306.addLineOLED(display,  "Wind Dir=%0.2f Degrees" % weatherStation.current_wind_direction())
	
        print "----------------- "
        print "----------------- "
        print "----------------- "

        if (config.DS3231_Present == True):
            currenttime = datetime.utcnow()

            deltatime = currenttime - starttime

            print "Raspberry Pi=\t" + time.strftime("%Y-%m-%d %H:%M:%S")

            if (config.OLED_Present) and (state.runOLED):
                Scroll_SSD1306.addLineOLED(display,"%s" % ds3231.read_datetime())

            print "DS3231=\t\t%s" % ds3231.read_datetime()

            print "DS3231 Temperature= \t%0.2f C" % ds3231.getTemp()
            print "----------------- "

        if (config.HDC1080_Present):
            if (config.OLED_Present) and (state.runOLED):
                Scroll_SSD1306.addLineOLED(display,  "InTemp = \t%0.2f C" % HTUtemperature)

        if config.AS3935_Present:
            print("AS3935 Lightning Detector Present")
        else:
            print("AS3935 Lightning Detector Not Present")

        print "----------------- "

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
            'Hour24AQI': state.Hour24_Outdoor_AirQuality_Sensor_Value,

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
        message = json.dumps(stateMessage)
        print("MQTT(",topic,"): ",message)
        state.mqtt_client.publish(topic, message, retain=True)

        with open("state.json", "w") as f:
            json.dump(stateMessage, f, indent=4)

        print "----------------- "
        print " Sample and Display Done"
        print "----------------- "

    except IOError as e:
        print "I/O error({0}): {1}".format(e.errno, e.strerror)
        print "exception in Sample and Display Check"
        print(traceback.format_exc())

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
        print("trying database")
        con = mdb.connect(config.MySQL_Host, config.MySQL_Username, config.MySQL_Password, 'SkyWeather');
        cur = con.cursor()
        print "before query"
        #query = 'INSERT INTO WeatherData(TimeStamp,as3935LightningCount, as3935LastInterrupt, as3935LastDistance, as3935LastStatus, currentWindSpeed, currentWindGust, totalRain,  bmp180Temperature, bmp180Pressure, bmp180Altitude,  bmp180SeaLevel,  outsideTemperature, outsideHumidity, currentWindDirection, currentWindDirectionVoltage, insideTemperature, insideHumidity, AQI) VALUES(UTC_TIMESTAMP(), %.3f, %.3f, %.3f, "%s", %.3f, %.3f, %.3f, %i, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f)' % (as3935LightningCount, as3935LastInterrupt, as3935LastDistance, as3935LastStatus, currentWindSpeed, currentWindGust, totalRain,  bmp180Temperature, bmp180Pressure, bmp180Altitude,  bmp180SeaLevel,  outsideTemperature, outsideHumidity, currentWindDirection, currentWindDirectionVoltage, HTUtemperature, HTUhumidity, state.Outdoor_AirQuality_Sensor_Value)
        query = 'INSERT INTO WeatherData(TimeStamp,as3935LightningCount, as3935LastInterrupt, as3935LastDistance, as3935LastStatus, currentWindSpeed, currentWindGust, totalRain,  bmp180Temperature, bmp180Pressure, bmp180Altitude,  bmp180SeaLevel,  outsideTemperature, outsideHumidity, currentWindDirection, currentWindDirectionVoltage, insideTemperature, insideHumidity, AQI) VALUES(UTC_TIMESTAMP(), %.3f, %.3f, %.3f, "%s", %.3f, %.3f, %.3f, %i, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f)' % (as3935LightningCount, as3935LastInterrupt, as3935LastDistance, as3935LastStatus, state.ScurrentWindSpeed, state.ScurrentWindGust, state.currentTotalRain,  state.currentInsideTemperature, state.currentBarometricPressure, state.currentAltitude,  state.currentSeaLevel,  state.currentOutsideTemperature, state.currentOutsideHumidity, state.ScurrentWindDirection, currentWindDirectionVoltage, state.currentInsideTemperature, state.currentInsideHumidity, state.Outdoor_AirQuality_Sensor_Value)
        print("query=%s" % query)

        cur.execute(query)
        as3935LastDistance = 0

        # now check for TSL2591 Sensor
        if (config.TSL2591_Present):
            query = 'INSERT INTO Sunlight(TimeStamp, Visible, IR, UV, UVIndex) VALUES(UTC_TIMESTAMP(), %d, %d, %d, %.3f)' % (SunlightVisible, SunlightIR, SunlightUV, SunlightUVIndex)
            print("query=%s" % query)
            cur.execute(query)
	
        # now check for Sunlight Sensor
        if (config.Sunlight_Present):
            query = 'INSERT INTO Sunlight(TimeStamp, Visible, IR, UV, UVIndex) VALUES(UTC_TIMESTAMP(), %d, %d, %d, %.3f)' % (SunlightVisible, SunlightIR, SunlightUV, SunlightUVIndex)
            print("query=%s" % query)
            cur.execute(query)
	
        con.commit()
		
    except mdb.Error, e:
        print "Error %d: %s" % (e.args[0],e.args[1])
        con.rollback()
        #sys.exit(1)
    
    finally:    
        cur.close() 
        con.close()
        del cur
        del con


def writePowerRecord():
    # now we have the data, stuff it in the database
    try:
        print("trying database")
        con = mdb.connect(config.MySQL_Host, config.MySQL_Username, config.MySQL_Password, 'SkyWeather');
        cur = con.cursor()
        print "before query"
        query = 'INSERT INTO PowerSystem(TimeStamp, batteryVoltage, batteryCurrent, solarVoltage, solarCurrent, loadVoltage, loadCurrent, batteryPower, solarPower, loadPower, batteryCharge) VALUES (UTC_TIMESTAMP (), %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f)' % (state.batteryVoltage, state.batteryCurrent, state.solarVoltage, state.solarCurrent, state.loadVoltage, state.loadCurrent, state.batteryPower, state.solarPower, state.loadPower, state.batteryCharge) 
        print("query=%s" % query)

        cur.execute(query)
        con.commit()
		
    except mdb.Error, e:
        print "Error %d: %s" % (e.args[0],e.args[1])
        con.rollback()
        #sys.exit(1)
    
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


#Rain calculations

rainArray = []
for i in range(20):
    rainArray.append(0)

lastRainReading = 0.0

def addRainToArray(plusRain):
    global rainArray
    del rainArray[0]
    rainArray.append(plusRain)
    #print "rainArray=", rainArray

def totalRainArray():
    global rainArray
    total = 0
    for i in range(20):
        total = total+rainArray[i]
    return total


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

def updateRain():
	global lastRainReading, rain60Minutes
	addRainToArray(totalRain - lastRainReading)	
	rain60Minutes = totalRainArray()
	lastRainReading = totalRain

def checkForShutdown():
    if (batteryVoltage < 3.5):
        print "--->>>>Time to Shutdown<<<<---"
        shutdownPi("low voltage shutdown")


def barometricTrend():
    if (state.currentBarometricPressure >= state.pastBarometricReading):
        state.barometricTrend = True
    else:
        state.barometricTrend = False
    state.pastBarometricReading = state.currentBarometricPressure



print  ""
print "SkyWeather Weather Station Version "+config.SWVERSION+" - SwitchDoc Labs"
print ""
print ""
print "Program Started at:"+ time.strftime("%Y-%m-%d %H:%M:%S")
print ""

def read_AQI():
    I2C_Lock.acquire()
    DustSensor.read_AQI()
    I2C_Lock.release()
	
###############
#  Turn Dust Sensor Off
################
try:
    read_AQI()
    config.DustSensor_Present = True
except Exception as e:
    print(e)
#patTheDog()




# Initialize Variables
bmp180Temperature =  0
bmp180Pressure = 0 
bmp180Altitude = 0
bmp180SeaLevel = 0 
bmp180Humidity = 0


print "----------------------"
print returnStatusLine("I2C Mux - TCA9545",config.TCA9545_I2CMux_Present)
print returnStatusLine("BME680",config.BME680_Present)
print returnStatusLine("BMP280",config.BMP280_Present)
print returnStatusLine("DS3231",config.DS3231_Present)
print returnStatusLine("HDC1080",config.HDC1080_Present)
print returnStatusLine("SHT30",config.SHT30_Present)
print returnStatusLine("AM2315",config.AM2315_Present)
print returnStatusLine("ADS1015",config.ADS1015_Present)
print returnStatusLine("ADS1115",config.ADS1115_Present)
print returnStatusLine("AS3935",config.AS3935_Present)
print returnStatusLine("GPS",config.GPS_Present)
print returnStatusLine("OLED",config.OLED_Present)
print returnStatusLine("SunAirPlus/SunControl",config.SunAirPlus_Present)
print returnStatusLine("SolarMAX",config.SolarMAX_Present)
print returnStatusLine("SI1145 Sun Sensor",config.Sunlight_Present)
print returnStatusLine("TSL2591 Sun Sensor",config.TSL2591_Present)
print returnStatusLine("DustSensor",config.DustSensor_Present)
print
print returnStatusLine("UseMySQL",config.enable_MySQL_Logging)
print "----------------------"



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
if (config.enable_MySQL_Logging == True):
	scheduler.add_job(writeWeatherRecord, 'interval', seconds=5*60)
	scheduler.add_job(writePowerRecord, 'interval', seconds=5*60)

scheduler.add_job(updateRain, 'interval', seconds=5*60)

#scheduler.add_job(checkForShutdown, 'interval', seconds=5*60)

# every 5 days at 00:04, reboot
#scheduler.add_job(rebootPi, 'cron', day='5-30/5', hour=0, minute=4, args=["5 day Reboot"]) 
	
#check for Barometric Trend (every 15 minutes)
scheduler.add_job(barometricTrend, 'interval', seconds=15*60)

if (config.DustSensor_Present):
    scheduler.add_job(read_AQI, 'interval', seconds=15*60)

# start scheduler
scheduler.start()
print "-----------------"
print "Scheduled Jobs"
print "-----------------"
scheduler.print_jobs()
print "-----------------"


if (config.SunAirPlus_Present == False):
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
while True:
    time.sleep(1.0)
