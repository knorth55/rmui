#!/usr/bin/env python

import rospy

from rmui_drivers import DummyRMUI

from force_proximity_ros.msg import ProximityArray
from sensor_msgs.msg import Imu

from rmui_msgs.msg import ImuCalibStatus


class DummyRMUINode(object):

    def __init__(self):
        frame_id = rospy.get_param('~frame_id', 'rmui_link')
        duration = rospy.get_param('~duration', 0.1)
        n_board = rospy.get_param('~n_board', 6)
        n_sensor = rospy.get_param('~n_sensor', 5)
        ea = rospy.get_param('~ea', 0.3)
        prx_threshold = rospy.get_param('~prx_threshold', 500)

        self.device = DummyRMUI(n_board, n_sensor, ea, prx_threshold, frame_id)

        self.pub_imu = rospy.Publisher(
            '~output/imu', Imu, queue_size=1)
        self.pub_imu_calib = rospy.Publisher(
            '~output/imu/calib_status', ImuCalibStatus, queue_size=1)
        self.pub_prx = rospy.Publisher(
            '~output/proximities', ProximityArray, queue_size=1)
        self.timer = rospy.Timer(
            rospy.Duration(duration), self._timer_cb)
        rospy.loginfo('dummy rmui node initialized')

    def _timer_cb(self, event):
        imu_msg = self.device.get_imu_msg()
        prx_msg = self.device.get_proximity_array_msg()
        calib_msg = self.device.get_imu_calib_msg()
        self.pub_imu.publish(imu_msg)
        self.pub_prx.publish(prx_msg)
        self.pub_imu_calib.publish(calib_msg)


if __name__ == '__main__':
    rospy.init_node('dummy_rmui_node')
    app = DummyRMUINode()
    rospy.spin()
