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

  print ("HM3301 Power On")
  GPIO.setup(config.DustSensorPowerPin, GPIO.OUT)
  GPIO.output(config.DustSensorPowerPin, True)
  time.sleep(1)
  pi = pigpio.pi()
  try:
    dustSensor = SDL_Pi_HM3301.SDL_Pi_HM3301(SDA=config.DustSensorSDA, SCL=config.DustSensorSCL, pi=pi)
  except:
    resetPiGpio()
    dustSensor = SDL_Pi_HM3301.SDL_Pi_HM3301(SDA=config.DustSensorSDA, SCL=config.DustSensorSCL, pi=pi)


def powerOffDustSensor():
  global dustSensor

  print ("HM3301 Power Off")
  try:
    dustSensor.close()
  except:
    resetPiGpio()
  GPIO.setup(config.DustSensorPowerPin, GPIO.OUT)
  GPIO.output(config.DustSensorPowerPin, False)
  time.sleep(1)


def read_AQI():
  print ("###############")
  print ("Reading AQI")
  print ("###############")

  attempts = 0
  aqi = 0
  checksum = False

  powerOnDustSensor()
  print("HM3301 Calibrating for 30 seconds")
  time.sleep(30)

  while True:

    try:
      data = dustSensor.get_data()
      aqi = int(dustSensor.get_aqi())
      dustSensor.print_data()
      checksum = dustSensor.checksum()
    except:
      pass

    if checksum and aqi > 0:
      state.Outdoor_AirQuality_Sensor_Value = aqi
      print("HM3301 Checksum Valid")
      break
    else:
      attempts += 1
      print("HM3301 Checksum Error!")
      if attempts > 10:
        break
      time.sleep(3)

  print("HM3301 AQI: %d" % aqi)
  powerOffDustSensor()

