#!/usr/bin/env python

import rospy

from force_proximity_ros.msg import ProximityArray
from sensor_msgs.msg import Imu

from rmui_drivers import BNO055
from rmui_drivers import PCA9547
from rmui_drivers import RMUI
from rmui_drivers import VCNL4040
from rmui_drivers import VCNL4040Multiplexa


class RMUINode(object):
    def __init__(
            self, bus=1, n_board_sensor=5,
            multiplexa_addresses=[0x70, 0x71, 0x72],
            vcnl_slave_address=0x60,
            imu_slave_address=0x28,
    ):
        super(RMUI, self).__init__()
        frame_id = rospy.get_param('~frame_id', 'rmui')
        duration = rospy.get_param('~duration', 0.1)
        ea = rospy.get_param('~ea', 0.3)
        sensitivity = rospy.get_param('~sensitivity', 50)

        imu = BNO055(bus, imu_slave_address)
        sensor_boards = []
        for multiplexa_address in multiplexa_addresses:
            multiplexa = PCA9547(bus, multiplexa_address)
            sensors = []
            for channel in range(n_board_sensor):
                sensor = VCNL4040(
                    bus, vcnl_slave_address, ea, sensitivity)
                sensors.append(sensor)
            sensor_board = VCNL4040Multiplexa(multiplexa, sensors)
            sensor_boards.append(sensor_board)

        self.device = RMUI(imu, sensor_boards, frame_id=frame_id)
        self.device.init_device()

        self.pub_imu = rospy.Publisher(
            '~output/imu', Imu, queue_size=1)
        self.pub_prx = rospy.Publisher(
            '~output/proximities', ProximityArray, queue_size=1)
        self.timer = rospy.Timer(
            rospy.Duration(duration), self._timer_cb)
        rospy.loginfo('rmui node initialized')

    def _timer_cb(self, event):
        imu_msg = self.device.get_imu_msg()
        prx_msg = self.device.get_proximity_array_msg()
        self.imu_msg.publish(imu_msg)
        self.pub_prx.publish(prx_msg)


if __name__ == '__main__':
    rospy.init_node('rmui_node')
    app = RMUINode()
    rospy.spin()
