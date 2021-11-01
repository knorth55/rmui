#!/usr/bin/env python

import rospy

from rmui_drivers import BNO055
from rmui_drivers import PCA9547
from rmui_drivers import RMUI
from rmui_drivers import VCNL4040
from rmui_drivers import VCNL4040Multiplexa
from rmui_drivers import WS281x

from force_proximity_ros.msg import ProximityArray
from sensor_msgs.msg import Imu

from rmui_msgs.msg import ImuCalibStatus


class RMUINode(object):
    def __init__(
            self, bus=1, n_board_sensor=5,
            multiplexa_addresses=[0x70, 0x71, 0x72, 0x73, 0x74, 0x75],
            vcnl_slave_address=0x60,
            imu_slave_address=0x28,
            led_pin=10, led_brightness=200,
    ):
        super(RMUINode, self).__init__()
        frame_id = rospy.get_param('~frame_id', 'rmui_link')
        duration = rospy.get_param('~duration', 0.1)
        ea = rospy.get_param('~ea', 0.3)
        sensitivity = rospy.get_param('~sensitivity', 50)
        touch_prx_threshold = rospy.get_param(
            '~touch_prx_threshold', 1000)
        n_board = rospy.get_param('~n_board', None)
        if n_board is not None:
            multiplexa_addresses = multiplexa_addresses[:n_board]

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
        led = WS281x(led_pin, len(multiplexa_addresses), led_brightness)

        self.device = RMUI(
            imu, sensor_boards, led,
            frame_id=frame_id,
            touch_prx_threshold=touch_prx_threshold)
        self.device.init_device()

        self.pub_imu = rospy.Publisher(
            '~output/imu', Imu, queue_size=1)
        self.pub_imu_calib = rospy.Publisher(
            '~output/imu/calib_status', ImuCalibStatus, queue_size=1)
        self.pub_prx = rospy.Publisher(
            '~output/proximities', ProximityArray, queue_size=1)
        self.timer = rospy.Timer(
            rospy.Duration(duration), self._timer_cb)
        rospy.loginfo('rmui node initialized')

    def _timer_cb(self, event):
        imu_msg = self.device.get_imu_msg()
        prx_msg = self.device.get_proximity_array_msg()
        self.device.turn_on_touch_led(prx_msg)
        calib_msg = self.device.get_imu_calib_msg()
        self.pub_imu.publish(imu_msg)
        self.pub_prx.publish(prx_msg)
        self.pub_imu_calib.publish(calib_msg)


if __name__ == '__main__':
    rospy.init_node('rmui_node')
    app = RMUINode()

    def shutdown_hook():
        app.device.led.turn_off()

    rospy.on_shutdown(shutdown_hook)
    rospy.spin()
