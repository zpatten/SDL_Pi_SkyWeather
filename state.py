# Weather Variables
currentOutsideTemperature = 0.0
currentOutsideHumidity = 1

currentInsideTemperature = 0.0
currentInsideHumidity = 1

currentRain60Minutes = 0.0

currentSunlightVisible = 0
currentSunlightIR = 0
currentSunlightUV = 0
currentSunlightUVIndex  = 0

ScurrentWindSpeed = 0
ScurrentWindGust  = 0
ScurrentWindDirection  = 0.2
currentTotalRain  = 0

currentBarometricPressure = 0
currentAltitude = 0 
currentSeaLevel = 0
pastBarometricReading = 0

Indoor_AirQuality_Sensor_Value = 0
Outdoor_AirQuality_Sensor_Value = 0

# Lightning Values
currentAs3935Interrupt = 0
currentAs3935LastInterrupt = 0
currentAs3935LastDistance = 0
currentAs3935LastStatus = 0
currentAs3935LastLightningTimeStamp = 0

# Button Variables
runOLED = False

# status Values
EnglishMetric = 0

# Solar Values
batteryVoltage = 0
batteryCurrent = 0
solarVoltage = 0
solarCurrent = 0
loadVoltage = 0
loadCurrent = 0
batteryPower = 0
solarPower = 0
loadPower = 0
batteryCharge = 0

# Fan State
fanState = False


def printState():
  print("--------------------------------------------------------------------------------")
  print("Current State")

  print("--------------------------------------------------------------------------------")
  print("currentOutsideTemperature..: %0.1f" % currentOutsideTemperature)
  print("currentOutsideHumidity.....: %0.1f" % currentOutsideHumidity)

  print("currentInsideTemperature...: %0.1f" % currentInsideTemperature)
  print("currentInsideHumidity......: %0.1f" % currentInsideHumidity)

  print("currentSunlightVisible.....: %0.1f" % currentSunlightVisible)
  print("currentSunlightIR..........: %0.1f" % currentSunlightIR)
  print("currentSunlightUV..........: %0.1f" % currentSunlightUV)
  print("currentSunlightUVIndex.....: %0.1f" % currentSunlightUVIndex)

  print("ScurrentWindSpeed..........: %0.1f" % ScurrentWindSpeed)
  print("ScurrentWindGust...........: %0.1f" % ScurrentWindGust)
  print("ScurrentWindDirection......: %0.1f" % ScurrentWindDirection)
  print("currentRain60Minutes.......: %0.1f" % currentRain60Minutes)
  print("currentTotalRain...........: %0.1f" % currentTotalRain)

  print("currentBarometricPressure..: %0.1f" % currentBarometricPressure)
  print("currentAltitude............: %0.1f" % currentAltitude)
  print("currentSeaLevel............: %0.1f" % currentSeaLevel)
  print("pastBarometricReading......: %0.1f" % pastBarometricReading)

  print("AirQuality_Sensor_Value....: %0.1f" % Outdoor_AirQuality_Sensor_Value)

  print("--------------------------------------------------------------------------------")
  print("runOLED.....: %s" % runOLED)

  print("--------------------------------------------------------------------------------")
  print("EnglishMetric..: %s" % EnglishMetric)

  print("--------------------------------------------------------------------------------")
  print("batteryVoltage..: %0.1f" % batteryVoltage)
  print("batteryCurrent..: %0.1f" % batteryCurrent)
  print("solarVoltage....: %0.1f" % solarVoltage)
  print("solarCurrent....: %0.1f" % solarCurrent)
  print("loadVoltage.....: %0.1f" % loadVoltage)
  print("loadCurrent.....: %0.1f" % loadCurrent)
  print("batteryPower....: %0.1f" % batteryPower)
  print("solarPower......: %0.1f" % solarPower)
  print("loadPower.......: %0.1f" % loadPower)
  print("batteryCharge...: %0.1f" % batteryCharge)

  print("--------------------------------------------------------------------------------")
  print("fanState: %s" % fanState)

  print("--------------------------------------------------------------------------------")
