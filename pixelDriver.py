# NeoPixel library strandtest example
# Author: Tony DiCola (tony@tonydicola.com)
#
# Direct port of the Arduino NeoPixel library strandtest example.  Showcases
# various animations on a strip of NeoPixels.
import time
import state
import updateBlynk
from neopixel import *

# Check for user imports
try:
            import conflocal as config
except ImportError:
            import config



# LED strip configuration:
LED_COUNT      = 16      # Number of LED pixels.
LED_PIN        = config.pixelPin      # GPIO pin connected to the pixels (must support PWM!).
LED_FREQ_HZ    = 800000  # LED signal frequency in hertz (usually 800khz)
#LED_DMA        = 10      # DMA channel to use for generating signal (try 10)
LED_DMA        = 5      # DMA channel to use for generating signal (try 10)
LED_BRIGHTNESS = 16     # Set to 0 for darkest and 255 for brightest
LED_INVERT     = False   # True to invert the signal (when using NPN transistor level shift)
LED_CHANNEL    = 0
LED_STRIP      = ws.SK6812_STRIP_RGBW
#LED_STRIP      = ws.SK6812W_STRIP

strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL, LED_STRIP)
# Intialize the library (must be called once before other functions).
strip.begin()

# Define functions which animate LEDs in various ways.
def colorWipe(strip, color, wait_ms=50):
	"""Wipe color across display a pixel at a time."""
	for i in range(LED_COUNT):
		strip.setPixelColor(i, color)
		strip.show()
		time.sleep(wait_ms/1000.0)

def theaterChase(strip, color, wait_ms=50, iterations=10):
	"""Movie theater light style chaser animation."""
	for j in range(iterations):
		for q in range(3):
			for i in range(0, LED_COUNT, 3):
				strip.setPixelColor(i+q, color)
			strip.show()
			time.sleep(wait_ms/1000.0)
			for i in range(0, LED_COUNT, 3):
				strip.setPixelColor(i+q, 0)

PALETTE = []
PALETTE.append(Color(0, 255, 0))
PALETTE.append(Color(255, 0, 0))
PALETTE.append(Color(0, 0, 255))
#print('PALETTE=', PALETTE)

RGB_PALETTE = []
for x in range(256):
  RGB_PALETTE.append(Color(255 - x, x, 0))
for x in range(256):
  RGB_PALETTE.append(Color(0, 255 - x, x))
for x in range(256):
  RGB_PALETTE.append(Color(x, 0, 255 - x))
#print('RGB_PALETTE=', RGB_PALETTE)

RGB_PALETTE_FADE = []
for x in range(256):
  RGB_PALETTE_FADE.append(Color(x, 0, 0))
for x in range(256):
  RGB_PALETTE_FADE.append(Color(255 - x, x, 0))
for x in range(256):
  RGB_PALETTE_FADE.append(Color(0, 255 - x, x))
for x in range(256):
  RGB_PALETTE_FADE.append(Color(0, 0, 255 - x))
#print('RGB_PALETTE_FADE=', RGB_PALETTE_FADE)

def wheel(pos, palette=RGB_PALETTE):
  """Generate rainbow colors across 0-255 positions."""
  max_pos = len(palette)
  rgb_pos = pos % max_pos
  return palette[rgb_pos]

def rainbow(strip, wait_ms=500, iterations=1):
  """Draw rainbow that fades across all pixels at once."""
  wait_ms = wait_ms / 1000.0
  for j in range(len(PALETTE) * iterations):
    for i in range(LED_COUNT):
      strip.setPixelColor(i, wheel(j+i, PALETTE))
    strip.show()
    time.sleep(wait_ms)

def rainbowCycle(strip, wait_ms=500, iterations=1):
  """Draw rainbow that uniformly distributes itself across all pixels."""
  wait_ms = wait_ms / 1000.0
  for j in range(0, len(RGB_PALETTE) * iterations, 64):
    for i in range(LED_COUNT):
      strip.setPixelColor(i, wheel(j, RGB_PALETTE))
    strip.show()
    time.sleep(wait_ms)

def theaterChaseRainbow(strip, wait_ms=50):
  """Rainbow movie theater light style chaser animation."""
  wait_ms = 10.0 / float((len(RGB_PALETTE) * iterations))
  print("wait_ms", wait_ms)
  for j in range(len(RGB_PALETTE)*iterations):
    for i in range(LED_COUNT):
      strip.setPixelColor(i, wheel(j+i))
    strip.show()
    time.sleep(wait_ms)


###############
# Flash LED
###############

def blinkLED(PixelLock, pixel, color, times, length):
  global strip

  if (config.runLEDs == True):
    if (PixelLock.acquire(False) == False):
      # We are locked so return - Don't wait
      if (config.SWDEBUG):
        print ("N--->Blink LED:Thread Locked")
      return

    try:
      #strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL, LED_STRIP)
      # Intialize the library (must be called once before other functions).
      #strip.begin()
        
      if (config.SWDEBUG):
        print "N--->Blink LED:%i/%i/%i/%6.2f" % (pixel, color, times, length)

      for x in range(LED_COUNT):
        strip.setPixelColor(x, Color(0, 0, 0))
      strip.show()
    
      strip.setPixelColor(0, Color(0, 255, 0))
      strip.show()
      time.sleep(length)
	
      for x in range(LED_COUNT):
        strip.setPixelColor(x, Color(0, 0, 0))
      strip.show()

    except Exception as e:
      print(e)
    PixelLock.release()


    
def statusLEDs(PixelLock):
  global strip
  '''
  if (PixelLock.acquire(False) == False):
    # We are locked so return - Don't wait
    if (config.SWDEBUG):
      print ("N--->status LEDs :Thread Locked")
    return
  '''
  if (PixelLock.acquire(False) == False):
    # We are locked so return - Don't wait
    if (config.SWDEBUG):
      print ("N--->statusLEDs:Thread Locked")
    return
    
  try:
    #strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL, LED_STRIP)
    # Intialize the library (must be called once before other functions).
    #strip.begin()
    
    if (config.SWDEBUG):
      print ("N ---->statusLEDs Running")
      print ("state.runRainbow =", state.runRainbow)
      print ("state.flashStrip =", state.flashStrip)
         
    if (config.runLEDs == True):
      if (state.flashStrip == True):
        updateBlynk.stopFlash()
        state.flashStrip = False
             
        updateBlynk.blynkStatusTerminalUpdate("Flashing Strips")
        # Now do the flash
    
        for i in range(LED_COUNT):
          strip.setPixelColor(i,Color(255,255,255))
        strip.show()
        time.sleep(0.5)
        for i in range(LED_COUNT):
          strip.setPixelColor(i,Color(0,0,0))
        strip.show()

      while (state.runRainbow == True):
        if (config.SWDEBUG):
          print "rainbow start"
        rainbow(strip)
        #rainbowCycle(strip)
        #theaterChaseRainbow(strip)
        if (config.SWDEBUG):
          print "rainbow end"

        for i in range(LED_COUNT):
          strip.setPixelColor(i,Color(0,0,0))
        strip.show()

      else:
        for i in range(LED_COUNT):
          strip.setPixelColor(i,Color(0,0,0))
          strip.show()

  except Exception as e:
    print(e)

  PixelLock.release()



"""

# Main program logic follows:
if __name__ == '__main__':
	# Create NeoPixel object with appropriate configuration.
	strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL, LED_STRIP)
	# Intialize the library (must be called once before other functions).
	strip.begin()

	print ('Press Ctrl-C to quit.')
/bin/bash: tx: command not found
		# Color wipe animations.
		colorWipe(strip, Color(255, 0, 0))  # Red wipe
		colorWipe(strip, Color(0, 255, 0))  # Blue wipe
		colorWipe(strip, Color(0, 0, 255))  # Green wipe
		colorWipe(strip, Color(0, 0, 0, 255))  # White wipe
		colorWipe(strip, Color(255, 255, 255))  # Composite White wipe
		colorWipe(strip, Color(255, 255, 255, 255))  # Composite White + White LED wipe
		# Theater chase animations.
		theaterChase(strip, Color(127, 0, 0))  # Red theater chase
		theaterChase(strip, Color(0, 127, 0))  # Green theater chase
		theaterChase(strip, Color(0, 0, 127))  # Blue theater chase
		theaterChase(strip, Color(0, 0, 0, 127))  # White theater chase
		theaterChase(strip, Color(127, 127, 127, 0))  # Composite White theater chase
		theaterChase(strip, Color(127, 127, 127, 127))  # Composite White + White theater chase
		# Rainbow animations.
		rainbow(strip)
		rainbowCycle(strip)
		theaterChaseRainbow(strip)


"""
