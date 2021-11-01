import rospy

from force_proximity_ros.msg import ProximityArray

from rmui_drivers import led_utils

from rmui_msgs.msg import ImuCalibStatus


class RMUI(object):
    def __init__(
            self, imu, sensor_boards, led, frame_id='rmui',
            touch_prx_threshold=500
    ):
        super(RMUI, self).__init__()
        self.imu = imu
        self.sensor_boards = sensor_boards
        for sensor_board in self.sensor_boards:
            sensor_board.multiplexa.stop()
        self.led = led
        self.frame_id = frame_id
        self.touch_prx_threshold = touch_prx_threshold

    def init_device(self):
        imu_calibrated = self.imu.init_sensor()
        if not imu_calibrated:
            rospy.logerr('IMU is not initially calibrated.')
        for sensor_board in self.sensor_boards:
            sensor_board.init_sensors()
        self.led.turn_off()
        self.led.turn_on()
        self.led.set_color_all(255, 0, 0, 0.1)
        self.led.set_color_all(0, 255, 0, 0.1)
        self.led.set_color_all(0, 0, 255, 0.1)
        self.led.set_color_all(0, 0, 0, 0.1)

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
        self.imu.read_calib_status()
        calib_msg = ImuCalibStatus()
        calib_msg.system = self.imu.sys_calib_status
        calib_msg.gyroscope = self.imu.gyr_calib_status
        calib_msg.accelerometer = self.imu.acc_calib_status
        calib_msg.magnetometer = self.imu.mag_calib_status
        calib_msg.header.stamp = rospy.Time.now()
        calib_msg.header.frame_id = self.frame_id
        return calib_msg

    def turn_on_touch_led(self, prx_msg):
        touch_prx_dict = {}
        for sensor_id, prx_data in enumerate(prx_msg.proximities):
            led_id = sensor_id // len(self.sensor_boards[0].sensors)
            if prx_data.proximity >= self.touch_prx_threshold:
                if led_id not in touch_prx_dict:
                    touch_prx_dict[led_id] = [prx_data.proximity]
                else:
                    touch_prx_dict[led_id].append(prx_data.proximity)

        for led_id in range(self.led.n_led):
            if led_id in touch_prx_dict:
                prx_data = touch_prx_dict[led_id]
                prx_average = sum(prx_data) / float(len(prx_data))
                r, g, b = led_utils.prx_to_rgb(
                    prx_average, min_prx=self.touch_prx_threshold,
                    max_prx=2000, max_rgb=200)
                self.led.set_color(led_id, r, g, b)
            else:
                self.led.set_color(led_id, 0, 0, 0)
