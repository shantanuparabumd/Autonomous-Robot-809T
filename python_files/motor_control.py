import RPi.GPIO as gpio
import time

def init():
    gpio.setmode(gpio.BOARD)
    # gpio.cleanup()
    gpio.setup(31, gpio.OUT)
    gpio.setup(33, gpio.OUT)
    gpio.setup(35, gpio.OUT)
    gpio.setup(37, gpio.OUT)

def gameover():
    gpio.output(31, False)
    gpio.output(33, False)
    gpio.output(35, False)
    gpio.output(37, False)

def forward(tf):
    init()
    # Left Wheels
    gpio.output(31, True)
    gpio.output(33, False)

    # Right Wheels
    gpio.output(35, False)
    gpio.output(37, True)

    #Wait 
    time.sleep(tf)

    # Send all pins low & cleanup
    gameover()
    gpio.cleanup()

forward(5)