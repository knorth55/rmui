#!/usr/bin/env python

import rospy
import sys
import tf2_ros

from rmui_drivers.dummy_rmui import DummyRMUI

from force_proximity_ros.msg import ProximityArray
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse
from std_srvs.srv import SetBool
from std_srvs.srv import SetBoolResponse

from rmui_msgs.msg import ImuCalibStatus


class DummyRMUINode(object):

    def __init__(self):
        frame_id = rospy.get_param('~frame_id', 'rmui_link')
        duration = rospy.get_param('~duration', 0.1)
        self.n_board = rospy.get_param('~n_board', 6)
        self.n_sensor = rospy.get_param('~n_sensor', 5)
        ea = rospy.get_param('~ea', 0.3)
        prx_threshold = rospy.get_param('~prx_threshold', 500)

        self.device = DummyRMUI(
            self.n_board, self.n_sensor, ea, prx_threshold, frame_id)

        self.pub_imu = rospy.Publisher(
            '~output/imu', Imu, queue_size=1)
        self.pub_imu_calib = rospy.Publisher(
            '~output/imu/calib_status', ImuCalibStatus, queue_size=1)
        self.pub_prx = rospy.Publisher(
            '~output/proximities', ProximityArray, queue_size=1)
        self.broadcaster = tf2_ros.TransformBroadcaster()

        # contact
        self.contact_board_services = []
        self.contact_sensor_services = []
        for i in range(self.n_board):
            self.contact_board_services.append(
                rospy.Service(
                    '~board{}/contact'.format(i), SetBool,
                    self._get_contact_board_service_cb(i)))
            for j in range(self.n_sensor):
                self.contact_sensor_services.append(
                    rospy.Service(
                        '~board{}/sensor{}/contact'.format(i, j), SetBool,
                        self._get_contact_sensor_service_cb(i, j)))

        self.contact_reset_service = rospy.Service(
            '~contact_reset', Empty, self._contact_reset_service_cb)
        # rotation
        self.rotate_services = []
        for axis in ['X', 'Y', 'Z']:
            for direction in ['cw', 'ccw']:
                for angle in [45, 90]:
                    self.rotate_services.append(
                        rospy.Service(
                            '~{}_axis/rotate_{}{}'.format(
                                axis.lower(), direction, angle),
                            Empty,
                            self._get_rotate_service_cb(
                                axis, direction, angle)))
        self.rotate_reset_service = rospy.Service(
            '~rotate_reset', Empty, self._rotate_reset_service_cb)

        # quit
        self.quit_service = rospy.Service(
            '~quit', Empty, self._quit_service_cb)

        rospy.loginfo('dummy rmui node initialized')
        # contact to ground
        self.device.contact_board(0)

        self.timer = rospy.Timer(
            rospy.Duration(duration), self._timer_cb)

    def _timer_cb(self, event):
        imu_msg = self.device.get_imu_msg()
        prx_msg = self.device.get_proximity_array_msg()
        calib_msg = self.device.get_imu_calib_msg()
        transform_stamped_msg = self.device.get_transform_stamped_msg()
        self.pub_imu.publish(imu_msg)
        self.pub_prx.publish(prx_msg)
        self.pub_imu_calib.publish(calib_msg)
        self.broadcaster.sendTransform(transform_stamped_msg)

    def _get_contact_board_service_cb(self, i):
        def _contact_board_cb(req):
            return self._contact_board_service_cb(req, i)
        return _contact_board_cb

    def _contact_board_service_cb(self, req, i):
        if req.data:
            rospy.loginfo('board {} is contacted'.format(i))
            self.device.contact_board(i)
        else:
            rospy.loginfo('board {} is released'.format(i))
            self.device.release_board(i)
        return SetBoolResponse(success=True)

    def _get_contact_sensor_service_cb(self, i, j):
        def _contact_sensor_cb(req):
            return self._contact_sensor_service_cb(req, i, j)
        return _contact_sensor_cb

    def _contact_sensor_service_cb(self, req, i, j):
        if req.data:
            rospy.loginfo('board {}, sensor {} is contacted'.format(i, j))
            self.device.contact_sensor(i, j)
        else:
            rospy.loginfo('board {}, sensor {} is released'.format(i, j))
            self.device.release_sensor(i, j)
        return SetBoolResponse(success=True)

    def _contact_reset_service_cb(self, req):
        rospy.loginfo('all board contact is resetted')
        for i in range(self.n_board):
            self.device.release_board(i)
        return EmptyResponse()

    def _get_rotate_service_cb(self, axis, direction, angle):
        def _rotate_cb(req):
            return self._rotate_service_cb(req, axis, direction, angle)
        return _rotate_cb

    def _rotate_service_cb(self, req, axis, direction, angle):
        if direction == 'ccw':
            angle = angle
        elif direction == 'cw':
            angle = -1 * angle
        else:
            rospy.logerr('Unsupported direction: {}'.format(direction))
        rospy.loginfo(
            'device rotates {} degree in {} axis'.format(angle, axis))
        self.device.rotate(axis, angle)
        return EmptyResponse()

    def _rotate_reset_service_cb(self, req):
        rospy.loginfo('all board rotation is resetted')
        self.device.reset_rotation()
        return EmptyResponse()

    def _quit_service_cb(self, req):
        rospy.loginfo('quit dummy node')
        sys.exit(0)


if __name__ == '__main__':
    rospy.init_node('dummy_rmui_node')
    app = DummyRMUINode()
    rospy.spin()
