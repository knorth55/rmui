import struct
import sys
import time

import smbus

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu


class BNO055(object):
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

    def __init__(self, bus=1, address=0x28):
        super(BNO055, self).__init__()
        self.bus = smbus.SMBus(bus)
        self.address = address
        if not self.read_chip_id() == 0xA0:
            sys.exit(1)
        self.sys_calib_status = 0
        self.gyr_calib_status = 0
        self.acc_calib_status = 0
        self.mag_calib_status = 0

    def init_sensor(self):
        # configuration
        self.set_config_mode()
        self.reset_system()
        time.sleep(1.0)
        while not self.read_chip_id() == 0xA0:
            time.sleep(0.01)
        time.sleep(0.05)

        # set normal power mode
        self.set_normal_power_mode()
        time.sleep(0.01)

        # set page to 0
        self.set_page(0x00)
        time.sleep(0.01)

        # use external oscillator
        self.use_external_oscillator()
        time.sleep(0.01)

        # unit select: radian
        self.set_radian_unit()

        # run
        self.set_ndof_mode()
        time.sleep(0.02)

        # calibration
        self.read_calib_status()
        calibrated = (
            self.sys_calib_status == 3 and
            self.gyr_calib_status == 3 and
            self.acc_calib_status == 3 and
            self.mag_calib_status == 3)

        return calibrated

    def set_config_mode(self):
        self._set_mode(0x00)

    def set_ndof_no_fmc_mode(self):
        self._set_mode(0x0B)

    def set_ndof_mode(self):
        self._set_mode(0x0C)

    def _set_mode(self, cmd):
        self.bus.write_byte_data(self.address, 0x3D, cmd)

    def set_normal_power_mode(self):
        self._set_power_mode(0x00)

    def _set_power_mode(self, cmd):
        self.bus.write_byte_data(self.address, 0x3E, cmd)

    def set_page(self, cmd):
        self.bus.write_byte_data(self.address, 0x07, cmd)

    def reset_system(self):
        self._set_sys_trigger(0x20)

    def reset_trigger(self):
        self._set_sys_trigger(0x00)

    def use_external_oscillator(self):
        self._set_sys_trigger(0x80)

    def set_radian_unit(self):
        self.bus.write_byte_data(self.address, 0x3B, 0x06)

    def _set_sys_trigger(self, cmd):
        self.bus.write_byte_data(self.address, 0x3F, cmd)

    def read_chip_id(self):
        return self.bus.read_byte_data(self.address, 0x00)

    def read_quaternion(self):
        return self._read_data(0x20, 8, (1.0 / (1 << 14)))

    def read_euler(self):
        return self._read_data(0x1A, 6, (1.0 / 900.0))

    def read_angular_velocity(self):
        return self._read_data(0x14, 6, (1.0 / 900.0))

    def read_linear_acceleration(self):
        return self._read_data(0x28, 6, (1.0 / 100.0))

    def read_gravity_acceleration(self):
        return self._read_data(0x2E, 6, (1.0 / 100.0))

    def _read_data(self, cmd, n_byte, scale):
        buf = self.bus.read_i2c_block_data(self.address, cmd, n_byte)
        data = []
        for i in range(n_byte // 2):
            d = struct.unpack('h', struct.pack('BB', buf[2*i], buf[2*i+1]))[0]
            data.append(d * scale)
        return data

    def read_calib_status(self):
        status = self.bus.read_byte_data(self.address, 0x35)
        self.sys_calib_status = int((status >> 6) & 0x03)
        self.gyr_calib_status = int((status >> 4) & 0x03)
        self.acc_calib_status = int((status >> 2) & 0x03)
        self.mag_calib_status = int(status & 0x03)

    def get_imu_msg(self, q, v, a):
        orientation = Quaternion(
            x=q[1], y=q[2], z=q[3], w=q[0])
        angular_velocity = Vector3(
            x=v[0], y=v[1], z=v[2])
        linear_acceleration = Vector3(
            x=a[0], y=a[1], z=a[2])
        imu_msg = Imu(
            orientation=orientation,
            orientation_covariance=self.orientation_covariance,
            angular_velocity=angular_velocity,
            angular_velocity_covariance=self.angular_velocity_covariance,
            linear_acceleration=linear_acceleration,
            linear_acceleration_covariance=self.linear_acceleration_covariance,
        )
        return imu_msg
