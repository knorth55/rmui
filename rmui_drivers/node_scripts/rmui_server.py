#!/usr/bin/env python

import re
import sys

import message_filters
import rospy

from force_proximity_ros.msg import ProximityArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from rmui_msgs.msg import ImuCalibStatus


class RMUIServer(object):
    def __init__(self):
        super(RMUIServer, self).__init__()
        self.node_names = []
        self.node_ids = []
        queue_size = rospy.get_param('~queue_size')
        approximate_sync = rospy.get_param('~approximate_sync')
        sensitivity = rospy.get_param('~sensitivity', 500)

        params = rospy.get_param(rospy.get_name(), [])
        for name, value in params.items():
            if not re.match(r'^node_\d$', name):
                continue
            self.node_names.append(value)
            self.node_ids.append(int(name.replace('node_', '')))
        if not self.node_names:
            rospy.logerr('No topic is specified.')
            sys.exit(1)

        self.subs = []
        self.nodes = []
        for node_id, node_name in zip(self.node_ids, self.node_names):
            node = RMUIClient(node_id, node_name, sensitivity)
            sub_prx = message_filters.Subscriber(
                '{}/output/proximities'.format(node_name),
                ProximityArray)
            sub_imu = message_filters.Subscriber(
                '{}/output/imu'.format(node_name), Imu)
            sub_imu_calib = message_filters.Subscriber(
                '{}/output/imu/calib_status'.format(node_name), ImuCalibStatus)
            if approximate_sync:
                sub = message_filters.ApproximateTimeSynchronizer(
                    [sub_prx, sub_imu, sub_imu_calib], queue_size, 0.1,
                    allow_headerless=True)
            else:
                sub = message_filters.TimeSynchronizer(
                    [sub_prx, sub_imu, sub_imu_calib], queue_size)
            sub.registerCallback(node._cb)

            self.subs.append(sub)
            self.nodes.append(node)


class RMUIClient(object):
    sensor_positions = [
        # board 0
        (0.05,  0.0,   -0.1),
        (0.0,   0.05,  -0.1),
        (-0.05, 0.0,   -0.1),
        (0.0,   -0.05, -0.1),
        (0.0,   0.0,   -0.1),
        # board 1
        (0.1,   0.05,  0.0),
        (0.1,   0.0,   -0.05),
        (0.1,   -0.05, 0.0),
        (0.1,   0.0,   0.05),
        (0.1,   0.0,   0.0),
        # board 2
        (-0.05, 0.1,   0.0),
        (0.0,   0.1,   -0.05),
        (0.05,  0.1,   0.0),
        (0.0,   0.1,   0.05),
        (0.0,   0.1,   0.0),
        # board 3
        (-0.1,  -0.05, 0.0),
        (-0.1,  0.0,   -0.05),
        (-0.1,  0.05,  0.0),
        (-0.1,  0.0,   0.05),
        (-0.1,  0.0,   0.0),
        # board 4
        (0.05,  -0.1,  0.0),
        (0.0,   -0.1,  -0.05),
        (-0.05, -0.1,  0.0),
        (0.0,   -0.1,  0.05),
        (0.0,   -0.1,  0.0),
        # board 5
        (0.05,  0.0,   0.1),
        (0.0,   -0.05, 0.1),
        (-0.05, 0.0,   0.1),
        (0.0,   0.05,  0.1),
        (0.0,   0.0,   0.1),
    ]
    sensor_directions = (
        '-z', 'x', 'y', '-x', '-y', 'z',
    )

    def __init__(self, node_id, node_name, sensitivity=500):
        super(RMUIClient, self).__init__()
        self.node_id = node_id
        self.node_name = node_name
        self.sensitivity = sensitivity
        self.sys_calib = False
        self.gyr_calib = False
        self.acc_calib = False
        self.mag_calib = False
        self.pub_prx_markers = rospy.Publisher(
            '{}/output/proximities/markers'.format(self.node_name),
            MarkerArray, queue_size=1)

    def _cb(self, prx_msg, imu_msg, imu_calib_msg):
        header = prx_msg.header
        prx_markers = MarkerArray()
        for prx_id, (proximity, sensor_pos) in enumerate(
                zip(prx_msg.proximities, self.sensor_positions)):
            if proximity.proximity < self.sensitivity:
                continue
            prx_marker = Marker()
            prx_marker.header = header
            prx_marker.ns = self.node_name
            prx_marker.id = prx_id
            prx_marker.type = Marker.ARROW
            prx_marker.scale.x = 0.02
            prx_marker.scale.y = 0.03
            prx_marker.color.r = 1.0
            prx_marker.color.g = 0.0
            prx_marker.color.b = 0.0
            prx_marker.color.a = 1.0
            prx_marker.pose.orientation.w = 1.0
            prx_marker.lifetime = rospy.Duration(0.5)
            prx_marker.action = Marker.ADD
            start_point = Point(
                x=sensor_pos[0],
                y=sensor_pos[1],
                z=sensor_pos[2])
            end_point = Point(
                x=sensor_pos[0],
                y=sensor_pos[1],
                z=sensor_pos[2])
            sensor_direction = self.sensor_directions[prx_id // 5]
            arrow_scale = min(
                3000.0, proximity.proximity - self.sensitivity) / 3000.0
            if sensor_direction[0] == '-':
                arrow_scale = arrow_scale * -1
            if sensor_direction[-1] == 'x':
                start_point.x = start_point.x + 0.2 * arrow_scale
            elif sensor_direction[-1] == 'y':
                start_point.y = start_point.y + 0.2 * arrow_scale
            else:
                start_point.z = start_point.z + 0.2 * arrow_scale
            prx_marker.points = [start_point, end_point]
            prx_markers.markers.append(prx_marker)
        self.pub_prx_markers.publish(prx_markers)

        self.sys_calib = (imu_calib_msg.system >= 3)
        self.gyr_calib = (imu_calib_msg.gyroscope >= 3)
        self.acc_calib = (imu_calib_msg.accelerometer >= 3)
        self.mag_calib = (imu_calib_msg.magnetometer >= 3)
        if not self.sys_calib:
            rospy.loginfo_throttle(
                1.0,
                'Device {} is not calibrated: system status {}'.format(
                    self.node_id, imu_calib_msg.system))
            return
        if not self.gyr_calib:
            rospy.loginfo_throttle(
                1.0,
                'Device {} is not calibrated: gyroscope status {}'.format(
                    self.node_id, imu_calib_msg.gyroscope))
            return


if __name__ == '__main__':
    rospy.init_node('rmui_server')
    app = RMUIServer()
    rospy.spin()
