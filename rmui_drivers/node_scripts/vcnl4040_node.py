#!/usr/bin/env python

import rospy

from force_proximity_ros.msg import Proximity
from force_proximity_ros.msg import ProximityStamped
from std_msgs.msg import Header

from rmui_drivers.vcnl4040 import VCNL4040


class VCNL4040Node(object):
    def __init__(self, bus=1, address=0x60):
        super(VCNL4040Node, self).__init__()
        self.sensor = VCNL4040(bus, address)
        self.average = self.sensor.read_proximity()
        self.fa2 = 0

        duration = rospy.get_param('~duration', 0.1)
        self.ea = rospy.get_param('~ea', 0.3)
        self.sensitivity = rospy.get_param('~sensitivity', 50)
        self.timer = rospy.Timer(
            rospy.Duration(duration), self._timer_cb)
        self.pub = rospy.Publisher(
            '~output', ProximityStamped, queue_size=1)
        rospy.loginfo('vcnl4040 node initialized')


    def _timer_cb(self, event):
        self.sensor.start_blink()
        prx_data = self.sensor.read_proximity()
        self.sensor.stop_blink()

        fa2derivative = self.average - prx_data - self.fa2
        self.fa2 = self.average - prx_data
        prx_msg = ProximityStamped(
            header=Header(stamp=rospy.Time.now()),
            proximity=Proximity(
                proximity=prx_data,
                average=self.average,
                fa2=self.fa2,
                fa2derivative=fa2derivative))
        if self.fa2 < -self.sensitivity:
            prx_msg.proximity.mode = "T"
        elif self.fa2 > self.sensitivity:
            prx_msg.proximity.mode = "R"
        else:
            prx_msg.proximity.mode = "0"
        self.pub.publish(prx_msg)
        self.average = self.ea * prx_data + (1 - self.ea) * self.average


if __name__ == '__main__':
    rospy.init_node('vcnl4040_node')
    app = VCNL4040Node()
    rospy.spin()
