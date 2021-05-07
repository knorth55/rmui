import smbus
import time


def main():
    bus = smbus.SMBus(1)
    addr = 0x60

    # coefficients
    data = bus.read_i2c_block_data(addr, 0x04, 8)
    a0 = (data[0] << 8 | data[1])
    if (a0 & 0x8000):
        a0 = -((~a0 & 0xffff) + 1)
    a0 = float(a0) / 8.0
    print('a0: {}'.format(a0))

    b1 = (data[2] << 8 | data[3])
    if (b1 & 0x8000):
        b1 = -((~b1 & 0xffff) + 1)
    b1 = float(b1) / 8192.0
    print('b1: {}'.format(b1))

    b2 = (data[4] << 8 | data[5])
    if (b2 & 0x8000):
        b2 = -((~b2 & 0xffff) + 1)
    b2 = float(b2) / 16384.0
    print('b2: {}'.format(b2))

    c12 = (data[6] << 8 | data[7])
    if (c12 & 0x8000):
        c12 = -((~c12 & 0xffff) >> 2 + 1)
    else:
        c12 = c12 >> 2
    c12 = float(c12) / 4194304.0
    print('c12: {}'.format(c12))

    while True:
        # start conversion
        bus.write_byte_data(addr, 0x12, 0x00)
        # wait for 3ms
        time.sleep(0.003)

        data = bus.read_i2c_block_data(addr, 0x00, 4)
        praw = (data[0] << 2 | data[1] >> 6)
        traw = (data[2] << 2 | data[3] >> 6)
        print('pressure   : {}'.format(praw))
        print('temperature: {}'.format(traw))

        if praw > 511:
            praw = praw - 1024
        pcomp = a0 + (b1 + c12 * traw) * praw + (b2 * traw)
        p = (pcomp * 65.0 / 1023.0) + 50.0
        t = 25.0 - ((traw - 498.0) / 5.35)
        print('pressure [kpa] : {}'.format(p))
        print('temperature [c]: {}'.format(t))
        time.sleep(0.5)


if __name__ == '__main__':
    main()
