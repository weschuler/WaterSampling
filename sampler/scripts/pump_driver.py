import RPi.GPIO as GPIO
from time import sleep


class Pump:
    # init function takes in values for the arguments.
    def __init__(self, pin):
        self.pin = pin
        self.__isrunning = False
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.pin, GPIO.OUT)

    def start(self):
        GPIO.output(self.pin, True)
        self.__isrunning = True

    def stop(self):
        GPIO.output(self.pin, False)
        self.__isrunning = False

    @property
    def isrunning(self,):
        return self.__isrunning
