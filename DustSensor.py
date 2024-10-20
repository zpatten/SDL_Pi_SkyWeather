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

def startPiGPIO():
  pi = pigpio.pi()
  time.sleep(1)
  return pi

def stopPiGPIO(pi):
  #try:
  #  pi.bb_i2c_close(SDA=config.DustSensorSDA)
  #finally:
  pi.stop()
  time.sleep(1)
    
def powerOnDustSensor():
  global dustSensor

  print ("HM3301 Power On")
  GPIO.setup(config.DustSensorPowerPin, GPIO.OUT)
  GPIO.output(config.DustSensorPowerPin, True)

  try:
    pi = startPiGPIO()
    dustSensor = SDL_Pi_HM3301.SDL_Pi_HM3301(SDA=config.DustSensorSDA, SCL=config.DustSensorSCL, pi=pi)
  except:
    try:
      stopPiGPIO(pi)
      pi = startPiGPIO()
      dustSensor = SDL_Pi_HM3301.SDL_Pi_HM3301(SDA=config.DustSensorSDA, SCL=config.DustSensorSCL, pi=pi)
    except:
      pass

  return pi


def powerOffDustSensor(pi):
  global dustSensor

  print ("HM3301 Power Off")
  try:
    dustSensor.close()
  finally:
    stopPiGPIO(pi)

  GPIO.setup(config.DustSensorPowerPin, GPIO.OUT)
  GPIO.output(config.DustSensorPowerPin, False)


def read_AQI():
  print("#" * 80)
  print("Reading AQI")
  print("#" * 80)

  attempts = 0
  aqi = 0
  checksum = False

  pi = powerOnDustSensor()
  print("HM3301 Calibrating for 30 seconds")
  time.sleep(30)

  while True:

    try:
      data = dustSensor.get_data()
      value = int(dustSensor.get_aqi())
      if value > 0:
        aqi = value
      dustSensor.print_data()
      checksum = dustSensor.checksum()
    except:
      pass

    if checksum and aqi > 0:
      print("HM3301 Checksum Valid")
      break
    else:
      attempts += 1
      print("HM3301 Checksum Error")
      if attempts > 30:
        break
      time.sleep(1)

  if aqi > 0:
    state.Outdoor_AirQuality_Sensor_Value = aqi

  print("HM3301 AQI: %d" % aqi)
  powerOffDustSensor(pi)

