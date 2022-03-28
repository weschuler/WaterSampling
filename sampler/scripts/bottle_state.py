import RPi.GPIO as GPIO
from time import sleep


class Bottle:
    # init function takes in values for the arguments.
    def __init__(self, pin):
        self.pin = pin
        self.__isfull = False
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    @property
    def is_full(self):
        if GPIO.input(self.pin) == False:
            self.__isfull = True
        #print("fetching data")
        return self.__isfull
        
    @is_full.setter
    def is_full(self, value):
        if value==True:
            self.__isfull = True
        else:
            self.__isfull = False
        #print("setting data")
    

