#!/usr/bin/env python

import re
import sys

import message_filters
import rospy

from rmui_drivers.rmui_client import RMUIClient

from force_proximity_ros.msg import ProximityArray
from sensor_msgs.msg import Imu

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


if __name__ == '__main__':
    rospy.init_node('rmui_server')
    app = RMUIServer()
    rospy.spin()
