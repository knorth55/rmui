import time

from rpi_ws281x import Color
from rpi_ws281x import PixelStrip


# LED strip configuration:
LED_COUNT = 1        # Number of LED pixels.
# LED_PIN = 12          # GPIO pin connected to the pixels (12 uses PWM!).
LED_PIN = 10        # GPIO pin connected to the pixels (10 uses SPI /dev/spidev0.0).
LED_FREQ_HZ = 800000  # LED signal frequency in hertz (usually 800khz)
LED_DMA = 10          # DMA channel to use for generating signal (try 10)
LED_BRIGHTNESS = 200  # Set to 0 for darkest and 255 for brightest
LED_INVERT = False    # True to invert the signal (when using NPN transistor level shift)
LED_CHANNEL = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53



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
            colorWipe(strip, Color(255, 0, 0), 0.5)  # Red wipe
            colorWipe(strip, Color(0, 255, 0), 0.5)  # Blue wipe
            colorWipe(strip, Color(0, 0, 255), 0.5)  # Green wipe
    except KeyboardInterrupt:
        colorWipe(strip, Color(0, 0, 0), 0.01)


if __name__ == '__main__':
    main()