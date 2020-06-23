#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Imu

from rmui_drivers import BNO055

from rmui_msgs.msg import ImuCalibStatus


class BNO055Node(object):
    def __init__(self, bus=1, address=0x28):
        super(BNO055Node, self).__init__()
        self.sensor = BNO055(bus, address)
        self.sensor.init_sensor()

        self.frame_id = rospy.get_param('~frame_id', 'imu')
        duration = rospy.get_param('~duration', 0.1)
        self.pub = rospy.Publisher('~output/imu', Imu, queue_size=1)
        self.pub_calib = rospy.Publisher(
            '~output/imu/calib_status', ImuCalibStatus, queue_size=1)
        self.timer = rospy.Timer(
            rospy.Duration(duration), self._timer_cb)
        rospy.loginfo('bno055 node initialized')

    def _timer_cb(self, event):
        q = self.sensor.read_quaternion()
        v = self.sensor.read_angular_velocity()
        a = self.sensor.read_linear_acceleration()
        imu_msg = self.sensor.get_imu_msg(q, v, a)
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = self.frame_id
        self.pub.publish(imu_msg)

        calib_msg = ImuCalibStatus()
        self.sensor.read_calib_status()
        calib_msg.system = self.sensor.sys_calib_status
        calib_msg.gyroscope = self.sensor.gyr_calib_status
        calib_msg.accelerometer = self.sensor.acc_calib_status
        calib_msg.magnetometer = self.sensor.mag_calib_status
        calib_msg.header.stamp = rospy.Time.now()
        calib_msg.header.frame_id = self.frame_id
        self.pub_calib.publish(calib_msg)


if __name__ == '__main__':
    rospy.init_node('bno055_node')
    app = BNO055Node()
    rospy.spin()
