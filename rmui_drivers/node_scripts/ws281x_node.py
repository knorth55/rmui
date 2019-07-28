#!/usr/bin/env python

import time

import rospy

from rmui_drivers import WS281x 
from rmui_msgs.msg import LED


class WS281xNode(object):
    def __init__(self, pin=10):
        n_led = rospy.get_param('~n_led', 1)
        self.led = WS281x(pin, n_led)
        self.led.turn_off()
        rospy.loginfo('ws28x1 node initialized')
        self.sub = rospy.Subscriber(
            '~input', LED, self._cb, queue_size=1)

    def _cb(self, msg):
        self.led.turn_on(msg.r, msg.g, msg.b)
        if msg.time >= 0:
            time.sleep(msg.time)
            self.led.turn_off()


if __name__ == '__main__':
    rospy.init_node('ws281x_node')
    app = WS281xNode()
    def shutdown_hook():
        app.led.turn_off()

    rospy.on_shutdown(shutdown_hook)
    rospy.spin()
