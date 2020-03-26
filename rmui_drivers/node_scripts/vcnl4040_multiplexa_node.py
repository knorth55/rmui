#!/usr/bin/env python

import rospy

from force_proximity_ros.msg import ProximityArray

from rmui_drivers import PCA9547
from rmui_drivers import VCNL4040
from rmui_drivers import VCNL4040Multiplexa


class VCNL4040MultiplexaNode(object):
    def __init__(
            self, bus=1, n_sensor=5,
            multiplexa_address=0x70, slave_address=0x60
    ):
        super(VCNL4040MultiplexaNode, self).__init__()
        duration = rospy.get_param('~duration', 0.1)
        ea = rospy.get_param('~ea', 0.3)
        sensitivity = rospy.get_param('~sensitivity', 50)

        multiplexa = PCA9547(bus, multiplexa_address)
        sensors = []
        for channel in range(n_sensor):
            sensor = VCNL4040(bus, slave_address, ea, sensitivity)
            sensors.append(sensor)

        self.sensor_board = VCNL4040Multiplexa(multiplexa, sensors)
        self.sensor_board.init_sensors()

        self.pub = rospy.Publisher(
            '~output', ProximityArray, queue_size=1)
        self.timer = rospy.Timer(
            rospy.Duration(duration), self._timer_cb)
        rospy.loginfo('vcnl4040 multiplexa node initialized')

    def _timer_cb(self, event):
        # start sensors
        self.sensor_board.start_sensors()
        # read proximities
        prx_data = self.sensor_board.read_proximities()
        # stop sensors
        self.sensor_board.stop_sensors()

        prx_msg = self._process(prx_data)
        self.pub.publish(prx_msg)

    def _process(self, prx_data):
        prx_msg = ProximityArray()
        prx_msg.proximities = self.sensor_board.get_proximity_msgs(prx_data)
        prx_msg.header.stamp = rospy.Time.now()
        return prx_msg


if __name__ == '__main__':
    rospy.init_node('vcnl4040_multiplexa_node')
    app = VCNL4040MultiplexaNode()
    rospy.spin()
