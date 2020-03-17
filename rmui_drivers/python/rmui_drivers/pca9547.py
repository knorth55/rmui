import smbus


class PCA9547(object):

    def __init__(self, bus=1, address=0x70):
        super(PCA9547, self).__init__()
        self.bus = smbus.SMBus(bus)
        self.address = address

    def start(self, ch):
        ch = ch & 0x07
        ch = ch | 0x08
        self.bus.write_byte(self.address, ch)

    def stop(self):
        self.bus.write_byte(self.address, 0x00)
