
#
#
# configuration file - contains customization for SkyWeather system
#

# it is a good idea to copy this file into a file called "conflocal.py" and edit that instead of this one.  This file is wiped out if you update SkyWeather.

SWDEBUG = False

SWVERSION = "000" # set in SkyWeather.py
import uuid 
  
# printing the value of unique MAC 
# address using uuid and getnode() function  
MACADDRESS = hex(uuid.getnode()) 

mailUser = "yourusename"
mailPassword = "yourmailpassword"

notifyAddress ="you@example.com"

fromAddress = "yourfromaddress@example.com"

enableText = False
textnotifyAddress = "yourphonenumber@yourprovider"

#MySQL Logging and Password Information

enable_MySQL_Logging = False
MySQL_Password = "password"

# modify this IP to enable WLAN operating detection  - search for WLAN_check in SkyWeather.py
enable_WLAN_Detection = False
PingableRouterAddress = "192.168.1.1"

# LED configuration (on use on a Raspberry Pi 3B+)
runLEDs = False


############
# AS3935 Lightning Configuration
############
# format: [NoiseFLoor, Indoor, TuneCap, DisturberDetection, WatchDogThreshold, SpikeDetection]
AS3935_Lightning_Config = [2,1,3,0,3,3]



# for barometeric pressure - needed to calculate sealevel equivalent - set your weatherstation elevation here

BMP280_Altitude_Meters = 328.0

# device present global variables


Camera_Present = False
TCA9545_I2CMux_Present = False
INA3221_Present = False
AS3935_Present = False
DS3231_Present = False
BMP280_Present = False
BME680_Present = False
HDC1080_Present = False
SHT30_Present = False
AM2315_Present = False
ADS1015_Present = False
ADS1115_Present = False
OLED_Present = False
OLED_Originally_Present = False
SI1145_Present = False
TSL2591_Present = False
HM3301_Present = True

# set Sunlight High Gain (indoors - 1) or Low Gain (outdoors - 0)
Sunlight_Gain = 0


# Pin definitions
pixelPin = 21

DustSensorPin = 19
DustSensorPowerPin = 26

#WeatherRack 
anemometerPin = 20
rainPin = 13


SHT30GSPIN = 6
AM2315GSPIN = 6

# for fan
GPIO_Pin_PowerDrive_Sig1 = 5
GPIO_Pin_PowerDrive_Sig2 = 5     # To avoid stepping on GPIO 6 

WATCHDOGTRIGGER = 4

