import numpy as np
try:
    from scipy.spatial.transform import Rotation
except Exception:
    print('please install scipy for dummy rmui')

import rospy

from rmui_drivers import imu_utils
from rmui_drivers import prx_utils

from force_proximity_ros.msg import ProximityArray
from geometry_msgs.msg import TransformStamped

from rmui_msgs.msg import ImuCalibStatus


class DummyRMUI(object):
    initial_q = np.array([0.0, 0.0, 0.0, 1.0])
    initial_v = np.array([0.0, 0.0, 0.0])
    initial_a = np.array([0.0, 0.0, 0.0])
    orientation_covariance = [
        0.002, 0, 0,
        0, 0.002, 0,
        0, 0, 0.002,
    ]
    angular_velocity_covariance = [
        0.003, 0, 0,
        0, 0.003, 0,
        0, 0, 0.003,
    ]
    linear_acceleration_covariance = [
        0.6, 0, 0,
        0, 0.6, 0,
        0, 0, 0.6,
    ]

    def __init__(
            self, n_board=6, n_sensor=5,
            ea=0.3, prx_threshold=500, frame_id='rmui'
    ):
        super(DummyRMUI, self).__init__()
        self.n_board = n_board
        self.n_sensor = n_sensor
        self.frame_id = frame_id
        self.ea = ea
        self.prx_threshold = prx_threshold
        self.fa2s = [0] * (self.n_board * self.n_sensor)
        self.averages = [None] * (self.n_board * self.n_sensor)
        self.contact = [False] * (self.n_board * self.n_sensor)
        self.reset_rotation()

    def get_imu_msg(self):
        imu_msg = imu_utils.get_imu_msg(
            self.q.tolist(), self.v.tolist(), self.a.tolist(),
            self.orientation_covariance,
            self.angular_velocity_covariance,
            self.linear_acceleration_covariance)
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = self.frame_id
        return imu_msg

    def get_proximity_array_msg(self):
        prx_msg = ProximityArray()
        msgs = []
        for i in range(self.n_board):
            for j in range(self.n_sensor):
                if self.contact[5 * i + j]:
                    prx_d = 2000
                else:
                    prx_d = 0
                average = self.averages[5 * i + j]
                fa2 = self.fa2s[5 * i + j]
                if average is None:
                    average = prx_d
                msg, average, fa2 = prx_utils.get_proximity_msg(
                    prx_d, average, fa2, self.ea, self.prx_threshold)
                self.averages[self.n_sensor * i + j] = average
                self.fa2s[self.n_sensor * i + j] = fa2
                msgs.append(msg)
        prx_msg.proximities = msgs
        prx_msg.header.stamp = rospy.Time.now()
        prx_msg.header.frame_id = self.frame_id
        return prx_msg

    def get_imu_calib_msg(self):
        calib_msg = ImuCalibStatus()
        calib_msg.system = 3
        calib_msg.gyroscope = 3
        calib_msg.accelerometer = 3
        calib_msg.magnetometer = 3
        calib_msg.header.stamp = rospy.Time.now()
        calib_msg.header.frame_id = self.frame_id
        return calib_msg

    def get_transform_stamped_msg(self):
        msg = TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'world'
        msg.child_frame_id = self.frame_id
        msg.transform.rotation.x = self.q[0]
        msg.transform.rotation.y = self.q[1]
        msg.transform.rotation.z = self.q[2]
        msg.transform.rotation.w = self.q[3]
        return msg

    def contact_board(self, i):
        for j in range(self.n_sensor):
            self.contact[5 * i + j] = True

    def contact_sensor(self, i, j):
        self.contact[5 * i + j] = True

    def release_board(self, i):
        for j in range(self.n_sensor):
            self.contact[5 * i + j] = False

    def release_sensor(self, i, j):
        self.contact[5 * i + j] = False

    def rotate(self, axis, angle):
        r0 = Rotation.from_quat(self.q)
        r1 = Rotation.from_euler(axis, angle, degrees=True)
        self.q = (r1 * r0).as_quat()
        rospy.loginfo('self.q: {}'.format(self.q))

    def reset_rotation(self):
        self.q = self.initial_q.copy()
        self.v = self.initial_v.copy()
        self.a = self.initial_a.copy()
