#!/usr/bin/env python

import rospy

from rmui_drivers import DummyRMUI

from force_proximity_ros.msg import ProximityArray
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse

from rmui_msgs.msg import ImuCalibStatus


class DummyRMUINode(object):

    def __init__(self):
        frame_id = rospy.get_param('~frame_id', 'rmui_link')
        duration = rospy.get_param('~duration', 0.1)
        self.n_board = rospy.get_param('~n_board', 6)
        n_sensor = rospy.get_param('~n_sensor', 5)
        ea = rospy.get_param('~ea', 0.3)
        prx_threshold = rospy.get_param('~prx_threshold', 500)

        self.device = DummyRMUI(
            self.n_board, n_sensor, ea, prx_threshold, frame_id)

        self.pub_imu = rospy.Publisher(
            '~output/imu', Imu, queue_size=1)
        self.pub_imu_calib = rospy.Publisher(
            '~output/imu/calib_status', ImuCalibStatus, queue_size=1)
        self.pub_prx = rospy.Publisher(
            '~output/proximities', ProximityArray, queue_size=1)
        self.timer = rospy.Timer(
            rospy.Duration(duration), self._timer_cb)
        self.contact_services = []
        self.release_services = []
        for i in range(self.n_board):
            self.contact_services.append(
                rospy.Service(
                    '~board{}/contact'.format(i), Empty,
                    self._get_contact_service_cb(i)))
            self.release_services.append(
                rospy.Service(
                    '~board{}/release'.format(i), Empty,
                    self._get_release_service_cb(i)))
        self.contact_reset_service = rospy.Service(
            '~contact_reset', Empty, self._contact_reset_service_cb)
        rospy.loginfo('dummy rmui node initialized')

    def _timer_cb(self, event):
        imu_msg = self.device.get_imu_msg()
        prx_msg = self.device.get_proximity_array_msg()
        calib_msg = self.device.get_imu_calib_msg()
        self.pub_imu.publish(imu_msg)
        self.pub_prx.publish(prx_msg)
        self.pub_imu_calib.publish(calib_msg)

    def _get_contact_service_cb(self, i):
        def _contact_cb(req):
            return self._contact_service_cb(req, i)
        return _contact_cb

    def _get_release_service_cb(self, i):
        def _release_cb(req):
            return self._release_service_cb(req, i)
        return _release_cb

    def _contact_service_cb(self, req, i):
        rospy.loginfo('board {} is contacted'.format(i))
        self.device.contact_board(i)
        return EmptyResponse()

    def _release_service_cb(self, req, i):
        rospy.loginfo('board {} is released'.format(i))
        self.device.release_board(i)
        return EmptyResponse()

    def _contact_reset_service_cb(self, req):
        rospy.loginfo('all board contact is resetted')
        for i in range(self.n_board):
            self.device.release_board(i)
        return EmptyResponse()


if __name__ == '__main__':
    rospy.init_node('dummy_rmui_node')
    app = DummyRMUINode()
    rospy.spin()
