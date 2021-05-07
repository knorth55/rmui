import smbus
import time


def main():
    bus = smbus.SMBus(1) 
    addr = 0x60

    # coefficients
    a0 = (bus.read_byte_data(addr, 0x04) << 8) | bus.read_byte_data(addr, 0x05)
    if (a0 & 0x8000):
        a0 = -((~a0 & 0xffff) + 1)
    a0 = float(a0) / 8.0
    print('a0: {}'.format(a0))

    b1 = (bus.read_byte_data(addr, 0x06) << 8) | bus.read_byte_data(addr, 0x07) 
    if (b1 & 0x8000):
        b1 = -((~b1 & 0xffff) + 1)
    b1 = float(b1) / 8192.0
    print('b1: {}'.format(b1))

    b2 = (bus.read_byte_data(addr, 0x08) << 8) | bus.read_byte_data(addr, 0x09)
    if (b2 & 0x8000):
        b2 = -((~b2 & 0xffff) + 1)
    b2 = float(b2) / 16384.0
    print('b2: {}'.format(b2))

    c12 = (bus.read_byte_data(addr, 0x0a) << 8) | bus.read_byte_data(addr, 0x0b)
    c12 = c12 >> 2
    if (c12 & 0x8000):
        c12 = -((~c12  &  (0xffff >> 2)) + 1)
    c12 = float(c12) / 4194304
    print('c12: {}'.format(c12))

    while True:
        # start conversion
        bus.write_byte_data(addr, 0x12, 0x00)
        # wait for 3ms
        time.sleep(0.003)

        praw = (bus.read_byte_data(addr, 0x00) << 2) | (bus.read_byte_data(addr, 0x01) >> 6)
        traw = (bus.read_byte_data(addr, 0x02) << 2) | (bus.read_byte_data(addr, 0x03) >> 6)

        pcomp = a0 + (b1 + c12 * traw) * praw + (b2 * traw)
        p = (pcomp * 65.0 / 1023.0) + 50.0
        t = 25.0 - ((traw -  498.0) / 5.35)
        print('pressure [kpa]: {}'.format(p))
        print('temperature [c]: {}'.format(t))
        time.sleep(0.5)


if __name__ == '__main__':
    main()
