import time

from rpi_ws281x import Color
from rpi_ws281x import PixelStrip


# LED strip configuration:
# Number of LED pixels.
LED_COUNT = 6
# GPIO pin connected to the pixels (10 uses SPI /dev/spidev0.0).
LED_PIN = 10
# LED signal frequency in hertz (usually 800khz)
LED_FREQ_HZ = 800000
# DMA channel to use for generating signal (try 10)
LED_DMA = 10
# Set to 0 for darkest and 255 for brightest
LED_BRIGHTNESS = 200
# True to invert the signal (when using NPN transistor level shift)
LED_INVERT = False
# set to '1' for GPIOs 13, 19, 41, 45 or 53
LED_CHANNEL = 0


def colorWipe(strip, color, wait_second):
    """Wipe color across display a pixel at a time."""
    for i in range(strip.numPixels()):
        strip.setPixelColor(i, color)
        strip.show()
    time.sleep(wait_second)


def main():
    strip = PixelStrip(
        LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA,
        LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
    strip.begin()

    try:
        while True:
            colorWipe(strip, Color(100, 0, 0), 0.5)
            colorWipe(strip, Color(200, 0, 0), 0.5)
            colorWipe(strip, Color(255, 0, 0), 0.5)  # Red wipe
            colorWipe(strip, Color(0, 100, 0), 0.5)
            colorWipe(strip, Color(0, 200, 0), 0.5)
            colorWipe(strip, Color(0, 255, 0), 0.5)  # Blue wipe
            colorWipe(strip, Color(0, 0, 100), 0.5)
            colorWipe(strip, Color(0, 0, 200), 0.5)
            colorWipe(strip, Color(0, 0, 255), 0.5)  # Green wipe
            colorWipe(strip, Color(100, 100, 0), 0.5)
            colorWipe(strip, Color(200, 200, 0), 0.5)
            colorWipe(strip, Color(255, 255, 0), 0.5)  # yellow wipe
            colorWipe(strip, Color(0, 100, 100), 0.5)
            colorWipe(strip, Color(0, 200, 200), 0.5)
            colorWipe(strip, Color(0, 255, 255), 0.5)  # sky wipe
            colorWipe(strip, Color(100, 0, 100), 0.5)
            colorWipe(strip, Color(200, 0, 200), 0.5)
            colorWipe(strip, Color(255, 0, 255), 0.5)  # pink wipe
            colorWipe(strip, Color(100, 100, 100), 0.5)
            colorWipe(strip, Color(200, 200, 200), 0.5)
            colorWipe(strip, Color(255, 255, 255), 0.5)  # white wipe
    except KeyboardInterrupt:
        colorWipe(strip, Color(0, 0, 0), 0.01)


if __name__ == '__main__':
    main()
