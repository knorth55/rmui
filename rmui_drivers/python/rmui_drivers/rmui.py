import rospy

from force_proximity_ros.msg import ProximityArray


class RMUI(object):
    def __init__(self, imu, sensor_boards, frame_id='rmui'):
        super(RMUI, self).__init__()
        self.imu = imu
        self.sensor_boards = sensor_boards
        for sensor_board in self.sensor_boards:
            sensor_board.multiplexa.stop()
        self.frame_id = frame_id

    def init_device(self):
        self.imu.init_sensor()
        for sensor_board in self.sensor_boards:
            sensor_board.init_sensors()

    def get_imu_msg(self):
        q = self.imu.read_quaternion()
        v = self.imu.read_angular_velocity()
        a = self.imu.read_linear_acceleration()
        imu_msg = self.imu.get_imu_msg(q, v, a)
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = self.frame_id
        return imu_msg

    def get_proximity_array_msg(self):
        prx_msg = ProximityArray()
        msgs = []
        for sensor_board in self.sensor_boards:
            sensor_board.start_sensors()
            prx_data = sensor_board.read_proximities()
            sensor_board.stop_sensors()
            msgs = msgs + sensor_board.get_proximity_msgs(prx_data)
        prx_msg.proximities = msgs
        prx_msg.header.stamp = rospy.Time.now()
        prx_msg.header.frame_id = self.frame_id
        return prx_msg
