import smbus

from force_proximity_ros.msg import Proximity


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

    def start_blink(self):
        try:
            self.bus.write_word_data(self.address, 0x03, 0x080e)
            return True
        except IOError:
            return False

    def stop_blink(self):
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
            return False

    def get_proximity_msg(self, prx_d):
        msg = Proximity()
        if prx_d is not False:
            if self.average is None:
                self.average = prx_d
            average = self.ea * prx_d + (1 - self.ea) * self.average
            fa2 = self.average - prx_d
            fa2derivative = self.average - prx_d - self.fa2
            self.average = average
            self.fa2 = fa2

            msg.proximity = prx_d
            msg.average = average
            msg.fa2 = fa2
            msg.fa2derivative = fa2derivative
            if self.fa2 < -self.sensitivity:
                msg.mode = "T"
            elif self.fa2 > self.sensitivity:
                msg.mode = "R"
            else:
                msg.mode = "0"
        else:
            msg.mode = "X"
        return msg
