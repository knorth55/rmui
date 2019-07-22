#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

from rmui_drivers import BNO055


class BNO055Node(object):
    def __init__(self, bus=1, address=0x28):
        super(BNO055Node, self).__init__()
        self.sensor = BNO055(bus, address)

        self.frame_id = rospy.get_param('~frame_id', 'imu')
        duration = rospy.get_param('~duration', 0.1)
        self.pub = rospy.Publisher('~output', Imu, queue_size=1)
        self.timer = rospy.Timer(
            rospy.Duration(duration), self._timer_cb)
        rospy.loginfo('bno055 node initialized')


    def _timer_cb(self, event):
        q = self.sensor.read_quaternion()
        v = self.sensor.read_angular_velocity()
        a = self.sensor.read_linear_acceleration()
        orientation = Quaternion(
            x=q[1], y=q[2], z=q[3], w=q[0])
        angular_velocity = Vector3(
            x=v[0], y=v[1], z=v[2])
        linear_acceleration = Vector3(
            x=a[0], y=a[1], z=a[2])
        imu_msg = Imu(
            header=Header(
                stamp=rospy.Time.now(),
                frame_id=self.frame_id),
            orientation=orientation,
            angular_velocity=angular_velocity,
            linear_acceleration=linear_acceleration,
        )
        self.pub.publish(imu_msg)


if __name__ == '__main__':
    rospy.init_node('bno055_node')
    app = BNO055Node()
    rospy.spin()
