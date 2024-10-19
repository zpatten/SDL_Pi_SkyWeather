
import requests
import time 

import hashlib

import os
import json


from PIL import ImageFont, ImageDraw, Image
import traceback
import util
import datetime as dt


# Check for user imports
try:
    import conflocal as config
except ImportError:
    import config

with open('state.json', 'r') as f:
    state = json.load(f)

def takeSkyPicture():
    print ("--------------------")
    print ("SkyCam Picture Taken")
    print ("--------------------")

    #os.system("/usr/bin/libcamera-jpeg --immediate --nopreview --width 1280 --height 720 -o static/skycamera.jpg")
    #os.system("/usr/bin/libcamera-jpeg --immediate --nopreview --width 1920 --height 1080 -o static/skycamera.jpg")
    #os.system("/usr/bin/libcamera-jpeg --immediate --nopreview --width 2328 --height 1748 -o static/skycamera.jpg")
    #os.system("/usr/bin/libcamera-jpeg --immediate --nopreview --width 3840 --height 2160 -o static/skycamera.jpg")
    #os.system("/usr/bin/libcamera-jpeg --immediate --nopreview --metering spot --width 4656 --height 3496 -o static/skycamera.jpg")
    os.system("/usr/bin/libcamera-still --immediate --nopreview --width 4656 --height 3496 -o static/skycamera.jpg")

    # now add timestamp to jpeg
    pil_im = Image.open('static/skycamera.jpg')
      
    draw = ImageDraw.Draw(pil_im)
        
    # Choose a font
    font = ImageFont.truetype("/usr/share/fonts/truetype/freefont/FreeSans.ttf", 72)

    # set up units
    #wind
    val = util.returnWindSpeed(state['currentWindSpeed'])
    WindStval = "{0:0.1f}".format(val) + util.returnWindSpeedUnit()
    val = util.returnWindSpeed(state['currentWindGust'])
    WindGtval = "{0:0.1f}".format(val) + util.returnWindSpeedUnit()
    val = util.returnTemperatureCF(state['outsideTemperature'])
    OTtval = "{0:0.1f}".format(val) + util.returnTemperatureCFUnit()

    AQItval = "{:.0f}".format(state['AQI'])
    CPUtval = "{0:0.1f}F".format(util.returnTemperatureCF(state['cpuTemp']))

    myText = "SkyWeather %s  Wind Speed: %s  Wind Gust: %s  Temp: %s (CPU: %s)  AQI: %s" % (dt.datetime.now().strftime('%d-%b-%Y %H:%M:%S'),WindStval, WindGtval, OTtval, CPUtval, AQItval)

    # Draw the text
    color = 'rgb(255,255,255)'
    #draw.text((0, 0), myText,fill = color, font=font)

    # get text size
    text_size = font.getsize(myText)

    # set button size + 10px margins
    button_size = (text_size[0]+20, text_size[1]+10)

    # create image with correct size and black background
    button_img = Image.new('RGBA', button_size, "black")

    # put text on button with 10px margins
    button_draw = ImageDraw.Draw(button_img)
    button_draw.text((10, 5), myText, fill = color, font=font)

    # put button on source image in position (0, 0)

    pil_im.paste(button_img, (0, 0))
    bg_w, bg_h = pil_im.size 
    # WeatherSTEM logo in lower left
    size = 64
    WSLimg = Image.open("static/WeatherSTEMLogoSkyBackground.png")
    WSLimg.thumbnail((size,size),Image.ANTIALIAS)
    pil_im.paste(WSLimg, (0, bg_h-size))

    # SkyWeather log in lower right
    SWLimg = Image.open("static/SkyWeatherLogoSymbol.png")
    SWLimg.thumbnail((size,size),Image.ANTIALIAS)
    pil_im.paste(SWLimg, (bg_w-size, bg_h-size))

    # Save the image
    pil_im.save('static/skycamera.jpg', format= 'JPEG')

    WSLimg.close()
    SWLimg.close()
    pil_im.close()

    os.system("/usr/bin/cp -v static/skycamera.jpg /home/pi/images/skyweather/$(date --iso-8601=seconds).jpg")



takeSkyPicture()
