import smbus


class VCNL4040(object):

    def __init__(self, bus=1, address=0x60):
        super(VCNL4040, self).__init__()
        self.bus = smbus.SMBus(bus)
        self.address = address

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
            data = self.bus.read_word_data(self.address, 0x0008)
            return data
        except IOError:
            return False
