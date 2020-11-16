import rospy

from rmui_drivers import imu_utils
from rmui_drivers import prx_utils

from force_proximity_ros.msg import ProximityArray

from rmui_msgs.msg import ImuCalibStatus


class DummyRMUI(object):
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
            ea=0.3, sensitivity=500, frame_id='rmui'
    ):
        super(DummyRMUI, self).__init__()
        self.n_board = n_board
        self.n_sensor = n_sensor
        self.frame_id = frame_id
        self.ea = ea
        self.sensitivity = sensitivity
        self.fa2s = [0] * (self.n_board * self.n_sensor)
        self.averages = [None] * (self.n_board * self.n_sensor)

    def init_device(self):
        rospy.loginfo('DummyRMUI is initialized.')

    def get_imu_msg(self):
        q = [0.0, 0.0, 0.0, 1.0]
        v = [0.0, 0.0, 0.0]
        a = [0.0, 0.0, 0.0]
        imu_msg = imu_utils.get_imu_msg(
            q, v, a,
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
            prx_data = [self.sensitivity + 100] * self.n_sensor
            for j, prx_d in enumerate(prx_data):
                average = self.averages[5*i+j]
                msg, average, fa2 = prx_utils.get_proximity_msg(
                    prx_d, average, self.fa2, self.ea, self.sensitivity)
                self.averages[self.n_sensor*i+j] = average
                self.fa2s[self.n_sensor*i+j] = fa2
                msgs = msgs.append(msg)
        prx_msg.proximities = msgs
        prx_msg.header.stamp = rospy.Time.now()
        prx_msg.header.frame_id = self.frame_id
        return prx_msg

    def get_imu_calib_msg(self):
        self.imu.read_calib_status()
        calib_msg = ImuCalibStatus()
        calib_msg.system = 3
        calib_msg.gyroscope = 3
        calib_msg.accelerometer = 3
        calib_msg.magnetometer = 3
        calib_msg.header.stamp = rospy.Time.now()
        calib_msg.header.frame_id = self.frame_id
        return calib_msg
