import RPi.GPIO as gpio
import numpy as np


ENCODER=7

def inti():
    gpio.setmode(gpio.BOARD)
    gpio.setup(ENCODER, gpio.IN, pull_up_down = gpio.PUD_UP)

def gameover():
    gpio.cleanup()


inti()


counter = np.uint64(0)
button = int(0)

while True:

    if int(gpio.input(ENCODER)) != int(button):
         button = int(gpio.input(ENCODER))
         counter+=1
         print(counter)

    if counter>=960:
        gameover()

        print("Thanks")
        break