import rospy

from force_proximity_ros.msg import ProximityArray

from rmui_msgs.msg import ImuCalibStatus


class RMUI(object):
    def __init__(self, imu, sensor_boards, led, frame_id='rmui'):
        super(RMUI, self).__init__()
        self.imu = imu
        self.sensor_boards = sensor_boards
        for sensor_board in self.sensor_boards:
            sensor_board.multiplexa.stop()
        self.led = led
        self.frame_id = frame_id

    def init_device(self):
        imu_calibrated = self.imu.init_sensor()
        if not imu_calibrated:
            rospy.logerr('IMU is not calibrated')
        for sensor_board in self.sensor_boards:
            sensor_board.init_sensors()
        self.led.turn_on(255, 0, 0, 0.1)
        self.led.turn_on(0, 255, 0, 0.1)
        self.led.turn_on(0, 0, 255, 0.1)
        self.led.turn_off()

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

    def get_imu_calib_msg(self):
        calib_msg = ImuCalibStatus()
        calib_status = self.imu.get_calib_status()
        calib_msg.system = calib_status[0]
        calib_msg.gyroscope = calib_status[1]
        calib_msg.accelerometer = calib_status[2]
        calib_msg.magnetometer = calib_status[3]
        calib_msg.header.stamp = rospy.Time.now()
        calib_msg.header.frame_id = self.frame_id
        return calib_msg

    def turn_on_touch_led(self, prx_msg):
        touch_led_ids = []
        for sensor_id, prx_data in enumerate(prx_msg.proximities):
            led_id = sensor_id // len(self.sensor_boards[0].sensors)
            if prx_data.proximity > 2000 and led_id not in touch_led_ids:
                touch_led_ids.append(led_id)

        for led_id in range(self.led.n_led):
            if led_id in touch_led_ids:
                self.led.set_color(led_id, 255, 0, 0)
            else:
                self.led.set_color(led_id, 0, 0, 0)
