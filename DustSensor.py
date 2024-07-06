#!/usr/bin/env python

#SwithchDoc Labs September 2018
# Public Domain


# tests SDL_Pi_DustSensor Driver
import sys
sys.path.append('./SDL_Pi_HM3301')
import time
import pigpio
import SDL_Pi_HM3301

import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
# Check for user imports
try:
            import conflocal as config
except ImportError:
            import config

import state

def resetPiGpio():
    pi = pigpio.pi()
    try:
        pi.bb_i2c_close(SDA=config.DustSensorSDA)
    finally:
        pi.stop()
    
def powerOnDustSensor():
    global dustSensor
    if (config.SWDEBUG):
        print ("Turning Dust Power On")
    #pi = pigpio.pi()
    #try:
    #    pi.bb_i2c_close(SDA=config.DustSensorSDA)
    #finally:
    #    pi.stop()
    GPIO.setup(config.DustSensorPowerPin, GPIO.OUT)
    GPIO.output(config.DustSensorPowerPin, True)
    time.sleep(30)
    pi = pigpio.pi()
    try:
        dustSensor = SDL_Pi_HM3301.SDL_Pi_HM3301(SDA=config.DustSensorSDA, SCL=config.DustSensorSCL, pi=pi)
    except:
        resetPiGpio()
        dustSensor = SDL_Pi_HM3301.SDL_Pi_HM3301(SDA=config.DustSensorSDA, SCL=config.DustSensorSCL, pi=pi)

    #time.sleep(1)
    #try:
    #pi = pigpio.pi()
    #dustSensor = SDL_Pi_HM3301.SDL_Pi_HM3301(SDA=config.DustSensorSDA, SCL=config.DustSensorSCL, pi=pi)
    #except:
    #    pi.bb_i2c_close()
    #    pi.stop()
    #    pi = pigpio.pi()
    #    dustSensor = SDL_Pi_HM3301.SDL_Pi_HM3301(SDA=config.DustSensorSDA, SCL=config.DustSensorSCL, pi=pi)
    #time.sleep(1)


def powerOffDustSensor():
    global dustSensor
    if (config.SWDEBUG):
        print ("Turning Dust Sensor Power Off")
    #try:
    #    pi.bb_i2c_close(SDA=config.DustSensorSDA)
    #except:
    #    pi.stop()
    #    pi = pigpio.pi()
    try:
        dustSensor.close()
    except:
        resetPiGpio()
    #try:
    #    pi.bb_i2c_close(SDA=config.DustSensorSDA)
    #finally:
    #    pi.stop()
    #pi.bb_i2c_close(SDA=config.DustSensorSDA)
    #pi.stop()
    #except:
    #    pi.bb_i2c_close()
    #    pi.stop()
    #time.sleep(1)
    #finally:
    GPIO.setup(config.DustSensorPowerPin, GPIO.OUT)
    GPIO.output(config.DustSensorPowerPin, False)
    #time.sleep(1)


def read_AQI():

    if (config.SWDEBUG):
        print ("###############")
        print ("Reading AQI")
        print ("###############")

    aqi = 0
    #while (int(aqi) == 0):
    powerOnDustSensor()

    # delay for 30 seconds for calibrated reading
    ##time.sleep(30)
      
    data = dustSensor.get_data()
    aqi = dustSensor.get_aqi()
    dustSensor.print_data()
    checksum_count = 0
    while ((dustSensor.checksum() != True or int(aqi) == 0) and checksum_count < 10):
        checksum_count += 1
        print("HM3301 Checksum Error!")
        #powerOffDustSensor()
        #powerOnDustSensor()
        data = dustSensor.get_data()
        aqi = dustSensor.get_aqi()
        dustSensor.print_data()

    print("  Current AQI (not 24 hour avg): " + str(int(aqi)))
    print("")

    if int(aqi) != 0:
        state.Outdoor_AirQuality_Sensor_Value = int(aqi)

    #if (dustSensor.checksum() != True):
    #    print("HM3301 Checksum Error!")
    #    dustSensor.print_data()
    #    data = dustSensor.get_data()
    #    if (dustSensor.checksum() != True):
    #        print("HM3301 2 Checksum Errors!")

    powerOffDustSensor()
    #time.sleep(3)

