import smbus2


class VCNL4040(object):

    def __init__(bus=1, address=0x60):
        super(VNCL4040, self).__init__()
        self.bus = smbus2.SMBus(bus)
        self.address = address

    def start_blink():
        self.bus.write_word_data(self.address, 0x03, 0x00)

    def stop_blink():
        self.bus.write_word_data(self.address, 0x03, 0x01)

    def read_proximity():
        data = self.bus.read_word_data(self.address, 0x08)
        return data
