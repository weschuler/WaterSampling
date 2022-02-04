import RPi.GPIO as GPIO
from time import sleep

class Pump:
    def __init__(self, pin):           # init function takes in values for the arguments.
        self.pin = pin                 # putting values in variables
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(pin,GPIO.OUT)
        
        
    def start(self):
        GPIO.output(self.pin, True)
        self.isrunning = 1
        
    def stop(self):
        GPIO.output(self.pin, False)
        self.isrunning = 0
