import smbus


class VCNL4040(object):

    def __init__(self, bus=1, address=0x60):
        super(VCNL4040, self).__init__()
        self.bus = smbus.SMBus(bus)
        self.address = address
        self.bus.write_word_data(self.address, 0x00, 0x0001)

    def start_blink(self):
        self.bus.write_word_data(self.address, 0x03, 0x080e)

    def stop_blink(self):
        self.bus.write_word_data(self.address, 0x03, 0x0100)

    def read_proximity(self):
        data = self.bus.read_word_data(self.address, 0x0008)
        return data
