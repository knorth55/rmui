#!/usr/bin/env python

import rospy

from force_proximity_ros.msg import ProximityArray

from rmui_drivers import PCA9547
from rmui_drivers import VCNL4040


class VCNL4040MultiPlexaNode(object):
    def __init__(
            self, bus=1, n_sensor=5,
            multiplexa_address=0x70, slave_address=0x60
    ):
        super(VCNL4040MultiPlexaNode, self).__init__()
        duration = rospy.get_param('~duration', 0.1)
        ea = rospy.get_param('~ea', 0.3)
        sensitivity = rospy.get_param('~sensitivity', 50)

        self.multiplexa = PCA9547(bus, multiplexa_address)
        self.sensor = VCNL4040(bus, slave_address, ea, sensitivity)
        self.n_sensor = n_sensor
        for channel in range(self.n_sensor):
            self.multiplexa.start(channel)
            self.sensor.init_sensor()
        self.multiplexa.stop()

        self.pub = rospy.Publisher(
            '~output', ProximityArray, queue_size=1)
        self.timer = rospy.Timer(
            rospy.Duration(duration), self._timer_cb)
        rospy.loginfo('vcnl4040 multiplexa node initialized')

    def _timer_cb(self, event):
        # start sensors
        for channel in range(self.n_sensor):
            self.multiplexa.start(channel)
            self.sensor.start_blink()
        self.multiplexa.stop()

        # stop sensors
        prx_data = []
        for channel in range(self.n_sensor):
            self.multiplexa.start(channel)
            prx_d = self.sensor.read_proximity()
            prx_data.append(prx_d)
            self.sensor.stop_blink()
        self.multiplexa.stop()

        prx_msg = self._process(prx_data)
        self.pub.publish(prx_msg)

    def _process(self, prx_data):
        prx_msg = ProximityArray()
        for i, prx_d in enumerate(prx_data):
            msg = self.sensor.get_proximity_msg(prx_d)
            prx_msg.proximities.append(msg)
        prx_msg.header.stamp = rospy.Time.now()
        return prx_msg


if __name__ == '__main__':
    rospy.init_node('vcnl4040_multiplexa_node')
    app = VCNL4040MultiPlexaNode()
    rospy.spin()
