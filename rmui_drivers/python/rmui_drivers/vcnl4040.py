import warnings

from rmui_drivers import prx_utils

from force_proximity_ros.msg import Proximity

try:
    import smbus
except ImportError:
    warnings.warn('Please install smbus for IMU: apt-get install python-smbus')


class VCNL4040(object):

    def __init__(self, bus=1, address=0x60, ea=0.3, sensitivity=50):
        super(VCNL4040, self).__init__()
        self.bus = smbus.SMBus(bus)
        self.address = address
        self.average = None
        self.fa2 = 0
        self.ea = ea
        self.sensitivity = sensitivity

    def init_sensor(self):
        try:
            self.bus.write_word_data(self.address, 0x00, 0x0001)
            return True
        except IOError:
            return False

    def start_sensor(self):
        try:
            self.bus.write_word_data(self.address, 0x03, 0x080e)
            return True
        except IOError:
            return False

    def stop_sensor(self):
        try:
            self.bus.write_word_data(self.address, 0x03, 0x0100)
            return True
        except IOError:
            return False

    def read_proximity(self):
        try:
            prx_d = self.bus.read_word_data(self.address, 0x0008)
            return prx_d
        except IOError:
            return None

    def get_proximity_msg(self, prx_d):
        if prx_d is None:
            prx_msg = Proximity()
        else:
            if self.average is None:
                self.average = prx_d
            prx_msg, average, fa2 = prx_utils.get_proximity_msg(
                prx_d, self.average, self.fa2, self.ea, self.sensitivity)
            self.average = average
            self.fa2 = fa2
        return prx_msg
