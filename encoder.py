import RPi.GPIO as gpio
import numpy as np

def inti():
    gpio.setmode(gpio.BOARD)
    gpio.setup(12, gpio.IN, pull_up_down = gpio.PUD_UP)

def gameover():
    gpio.cleanup()


inti()


counter = np.uint64(0)
button = int(0)

while True:

    if int(gpio.input(12)) != int(button):
         button = int(gpio.input(12))
         counter+=1
         print(counter)

    if counter>=960:
        gameover()

        print("Thanks")
        break