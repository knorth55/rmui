import time

from rpi_ws281x import Color
from rpi_ws281x import PixelStrip


class WS281x(object):
    hz = 800000
    dma = 10

    def __init__(self, pin=10, n_led=1, brightness=200):
        super(WS281x, self).__init__()
        self.strip = PixelStrip(
            n_led, pin, self.hz, self.dma, False, brightness, 0)
        self.n_led = n_led
        self.strip.begin()

    def set_color_all(self, R, G, B):
        for i in range(self.strip.numPixels()):
            self.set_color(i, R, G, B)

    def set_color(self, i, R, G, B):
        color = Color(R, G, B)
        self.strip.setPixelColor(i, color)
        self.strip.show()

    def turn_on(self, R, G, B, wait=0.01):
        self.set_color_all(R, G, B)
        time.sleep(wait)

    def turn_off(self, wait=0.01):
        self.set_color_all(0, 0, 0)
        time.sleep(wait)
