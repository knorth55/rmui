import time

from rpi_ws281x import Color
from rpi_ws281x import PixelStrip


class WS281x(object):
    hz = 800000
    dma = 10

    def __init__(self, pin=10, n_led=1, brightness=255):
        self.strip = PixelStrip(
            n_led, pin, self.hz, self.dma, False, brightness, 0)
        self.strip.begin()

    def set_color(self, R, G, B):
        color = Color(R, G, B)
        for i in range(self.strip.numPixels()):
            self.strip.setPixelColor(i, color)
            self.strip.show()

    def turn_on(self, R, G, B, wait=0.01):
        self.set_color(R, G, B)
        time.sleep(wait)

    def turn_off(self):
        self.set_color(0, 0, 0)
        time.sleep(0.01)
