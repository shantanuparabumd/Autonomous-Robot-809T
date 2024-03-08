import RPi.GPIO as gpio
import numpy as np
import time
import datetime


fl = open('hw5data.txt','a') 

def init():
    gpio.setmode(gpio.BOARD)
    # gpio.cleanup()
    gpio.setup(31, gpio.OUT)
    gpio.setup(33, gpio.OUT)
    gpio.setup(35, gpio.OUT)
    gpio.setup(37, gpio.OUT)
    gpio.setup(12, gpio.IN, pull_up_down = gpio.PUD_UP)

def gameover():
    gpio.cleanup()

init()
pwm=gpio.PWM(37,50)
val=14
pwm.start(val)
time.sleep(0.1)

counter = np.uint64(0)
button = int(0)
# Get the current time in seconds since the epoch
start_time = datetime.datetime.now()
while True:

    if int(gpio.input(12)) != int(button):
         button = int(gpio.input(12))
         print(button)
         #Calculating and saving time to file
         end_time = datetime.datetime.now()
         now=end_time-start_time
         outstring = str(button)+' '+str(now.total_seconds())+'\n'

        # Writing to file
         fl.write(outstring)
         counter+=1
         print(counter)

    if counter>=960:
        gameover()
        pwm.stop()
        print("Thanks")
        break