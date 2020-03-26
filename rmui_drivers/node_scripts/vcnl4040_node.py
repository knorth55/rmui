#!/usr/bin/env python

import rospy

from force_proximity_ros.msg import ProximityStamped

from rmui_drivers import VCNL4040


class VCNL4040Node(object):
    def __init__(self, bus=1, address=0x60):
        super(VCNL4040Node, self).__init__()
        duration = rospy.get_param('~duration', 0.1)
        ea = rospy.get_param('~ea', 0.3)
        sensitivity = rospy.get_param('~sensitivity', 50)

        self.sensor = VCNL4040(bus, address, ea, sensitivity)
        self.sensor.init_sensor()

        self.pub = rospy.Publisher(
            '~output', ProximityStamped, queue_size=1)
        self.timer = rospy.Timer(
            rospy.Duration(duration), self._timer_cb)
        rospy.loginfo('vcnl4040 node initialized')

    def _timer_cb(self, event):
        self.sensor.start_sensor()
        prx_d = self.sensor.read_proximity()
        self.sensor.stop_sensor()
        prx_msg = self._process(prx_d)
        self.pub.publish(prx_msg)

    def _process(self, prx_d):
        prx_msg = ProximityStamped()
        prx_msg.proximity = self.sensor.get_proximity_msg(prx_d)
        prx_msg.header.stamp = rospy.Time.now()
        return prx_msg


if __name__ == '__main__':
    rospy.init_node('vcnl4040_node')
    app = VCNL4040Node()
    rospy.spin()
