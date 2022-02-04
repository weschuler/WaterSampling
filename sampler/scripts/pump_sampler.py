from gpiozero import PWMLED
from time import sleep

class Pump:
    def __init__(self, pin):              # init function takes in values for the arguments.
        self.pin = pin                          # putting values in variables
        
    def start(self):
        
        p = PWMLED(self.pin)
        p.value = 1.0
        self.isrunning = 1
        
    def stop(self):
        
        p = PWMLED(self.pin)
        p.value = 0
        self.isrunning = 0

mastersampler = Pump(19)                          # pin 19, 16, 26, 20 are being used 
sampler_A = Pump(16)
sampler_B = Pump(26)
sampler_C = Pump(20)
        
if trigger == HIGH:
    mastersampler.start()
    if 