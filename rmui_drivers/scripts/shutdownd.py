#!/usr/bin/python


import os
import RPi.GPIO as GPIO
import time


def shutdown(channel):
    print("shutdown")
    os.system("shutdown -h now")


def main():
    GPIO_ID = 26
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(GPIO_ID, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(
        GPIO_ID, GPIO.FALLING, callback=shutdown, bouncetime=2000)
    while True:
        time.sleep(100)


if __name__ == '__main__':
    main()
