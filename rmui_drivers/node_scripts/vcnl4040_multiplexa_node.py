#!/usr/bin/env python

import rospy

from force_proximity_ros.msg import Proximity
from force_proximity_ros.msg import ProximityArray

from rmui_drivers import PCA9547
from rmui_drivers import VCNL4040


class VCNL4040MultiPlexaNode(object):
    def __init__(
            self, bus=1, n_sensors=5,
            multiplexa_address=0x70, slave_address=0x60
    ):
        super(VCNL4040MultiPlexaNode, self).__init__()
        self.multiplexa = PCA9547(bus, multiplexa_address)
        for channel in range(self.n_sensors):
            self.multiplexa.start(channel)
            self.sensor = VCNL4040(bus, slave_address)
        self.multiplexa.stop()
        self.fa2 = [0] * self.n_sensors
        self.average = [None] * self.n_sensors

        duration = rospy.get_param('~duration', 0.1)
        self.ea = rospy.get_param('~ea', 0.3)
        self.sensitivity = rospy.get_param('~sensitivity', 50)
        self.pub = rospy.Publisher(
            '~output', ProximityArray, queue_size=1)
        self.timer = rospy.Timer(
            rospy.Duration(duration), self._timer_cb)
        rospy.loginfo('vcnl4040 multiplexa node initialized')

    def _timer_cb(self, event):
        # start sensors
        for channel in range(self.n_sensors):
            self.multiplexa.start(channel)
            self.sensor.start_blink()
        self.multiplexa.stop()

        # stop sensors
        prx_data = []
        for channel in range(self.n_sensors):
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
            if self.average[i] is None:
                self.average[i] = prx_d
            average = (1 - self.ea) * self.average[i] + self.ea * prx_d
            fa2derivative = self.average[i] - prx_d - self.fa2[i]
            fa2 = self.average[i] - prx_d
            self.average[i] = average
            self.fa2[i] = fa2

            msg = Proximity(
                proximity=prx_d,
                average=average,
                fa2=fa2,
                fa2derivative=fa2derivative)
            if fa2 < -self.sensitivity:
                msg.mode = "T"
            elif fa2 > self.sensitivity:
                msg.mode = "R"
            else:
                msg.mode = "0"
            prx_msg.proximities.append(msg)
        prx_msg.header.stamp = rospy.Time.now()
        return prx_msg


if __name__ == '__main__':
    rospy.init_node('vcnl4040_multiplexa_node')
    app = VCNL4040MultiPlexaNode()
    rospy.spin()
