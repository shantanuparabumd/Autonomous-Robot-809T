import RPi.GPIO as gpio
import time

class Localization:
    
    def __init__(self) -> None:
        self.X=0
        self.Y=0
        self.Theta=0
        # PINS
        self.TRIG=16
        self.ECHO=18

    def calibrate(self):
        pass

    def distance(self):
        gpio.setmode(gpio.BOARD)
        gpio.setup(self.TRIG,gpio.OUT)
        gpio.setup(self.ECHO,gpio.IN)

        #Ensure output has no value
        gpio.output(self.TRIG, False)
        time.sleep(0.01)

        #Genereate trigger pulse
        gpio.output(self.TRIG,True)
        time.sleep(0.00001)
        gpio.output(self.TRIG, False)

        #Generate self.ECHO time signal
        while gpio.input(self.ECHO) == 0:
            pulse_start = time.time()

        while gpio.input(self.ECHO)== 1:
            pulse_end = time.time()

        pulse_duration = pulse_end-pulse_start

        #Convert time to distance
        distance = pulse_duration*17150
        distance = round(distance, 2)

        #Cleanup gpio 7 return distance
        gpio.cleanup()
        return distance
    
    
