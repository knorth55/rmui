#!/usr/bin/env python

import time

import rospy

from rmui_drivers import WS281x
from rmui_msgs.msg import LED


class WS281xNode(object):
    def __init__(self, pin=10):
        n_led = rospy.get_param('~n_led', 1)
        brightness = rospy.get_param('~brightness', 200)
        self.led = WS281x(pin, n_led, brightness)
        self.led.turn_off()
        self.sub = rospy.Subscriber(
            '~input', LED, self._cb, queue_size=1)
        self.led.turn_on()
        rospy.loginfo('ws281x node initialized')

    def _cb(self, msg):
        self.led.set_color_all(msg.r, msg.g, msg.b)
        if msg.time >= 0:
            time.sleep(msg.time)
            self.led.turn_off()


if __name__ == '__main__':
    rospy.init_node('ws281x_node')
    app = WS281xNode()

    def shutdown_hook():
        app.led.turn_off(wait=1.0)

    rospy.on_shutdown(shutdown_hook)
    rospy.spin()
