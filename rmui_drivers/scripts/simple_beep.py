import RPi.GPIO as GPIO
import time


SOUNDER = 23
HZ = [262, 294, 330, 349, 392, 440, 494, 523]
MELODY = [0, 0, 4, 4, 5, 5, 4, -1, 3, 3, 2, 2, 1, 1, 0]


def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SOUNDER, GPIO.OUT, initial=GPIO.LOW)
    p = GPIO.PWM(SOUNDER, 1)

    p.start(50)

    for melody in MELODY:
        hz = HZ[melody]
        if hz < 0:
            time.sleep(0.5)
            continue
        p.ChangeFrequency(hz)
        time.sleep(0.5)

    p.stop()
    GPIO.cleanup()


if __name__ == '__main__':
    main()
