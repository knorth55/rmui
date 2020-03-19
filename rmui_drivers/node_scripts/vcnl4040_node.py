#!/usr/bin/env python

import rospy
from std_msgs.msg import Header

from force_proximity_ros.msg import Proximity
from force_proximity_ros.msg import ProximityStamped

from rmui_drivers import VCNL4040


class VCNL4040Node(object):
    def __init__(self, bus=1, address=0x60):
        super(VCNL4040Node, self).__init__()
        self.sensor = VCNL4040(bus, address)
        self.sensor.init_sensor()
        self.average = None
        self.fa2 = 0

        duration = rospy.get_param('~duration', 0.1)
        self.ea = rospy.get_param('~ea', 0.3)
        self.sensitivity = rospy.get_param('~sensitivity', 50)
        self.pub = rospy.Publisher(
            '~output', ProximityStamped, queue_size=1)
        self.timer = rospy.Timer(
            rospy.Duration(duration), self._timer_cb)
        rospy.loginfo('vcnl4040 node initialized')

    def _timer_cb(self, event):
        self.sensor.start_blink()
        prx_d = self.sensor.read_proximity()
        self.sensor.stop_blink()

        prx_msg = self._process(prx_d)
        self.pub.publish(prx_msg)

    def _process(self, prx_d):
        prx_msg = ProximityStamped()
        msg = Proximity()
        if prx_d is not False:
            if self.average is None:
                self.average = prx_d
            average = self.ea * prx_d + (1 - self.ea) * self.average
            fa2 = self.average - prx_d
            fa2derivative = self.average - prx_d - self.fa2
            self.average = average
            self.fa2 = fa2

            msg.proximity = prx_d
            msg.average = average
            msg.fa2 = fa2
            msg.fa2derivative = fa2derivative
            if self.fa2 < -self.sensitivity:
                msg.mode = "T"
            elif self.fa2 > self.sensitivity:
                msg.mode = "R"
            else:
                msg.mode = "0"
        else:
            msg.mode = "X"
        prx_msg.proximity = msg
        prx_msg.header.stamp = rospy.Time.now()
        return prx_msg


if __name__ == '__main__':
    rospy.init_node('vcnl4040_node')
    app = VCNL4040Node()
    rospy.spin()
