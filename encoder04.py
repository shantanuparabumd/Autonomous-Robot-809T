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
    gpio.setup(7, gpio.IN, pull_up_down = gpio.PUD_UP)

def gameover():
    gpio.output(31,False)
    gpio.output(33,False)
    gpio.output(35,False)
    gpio.output(37,False)
    gpio.cleanup()

init()

counterBR = np.uint64(0)
counterFL = np.uint64(0)

buttonBR = int(0)
buttonFL = int(0)


val=22
pwm1=gpio.PWM(31,50)
pwm1.start(val)
pwm2=gpio.PWM(37,50)
pwm2.start(val)
time.sleep(0.1)

counter = np.uint64(0)
button = int(0)
# Get the current time in seconds since the epoch
start_time = datetime.datetime.now()

for i in range(0,200000):

    print("countBR: ", counterBR, "counterFL: ", counterFL, "BR state: ", gpio.input(12), "FL state: ", gpio.input(7))
    if int(gpio.input(12)) != int(buttonBR):
         buttonBR = int(gpio.input(12))
         counterBR+=1

    if int(gpio.input(7)) != int(buttonFL):
         buttonFL = int(gpio.input(7))
         counterFL+=1
         #Calculating and saving time to file
        #  end_time = datetime.datetime.now()
        #  now=end_time-start_time
        #  outstring = str(button)+' '+str(now.total_seconds())+'\n'

        # # Writing to file
        #  fl.write(outstring)
        #  counter+=1
        #  print(counter)
    if counterFL>=960:

        pwm1.stop()

        
    if counterBR>=960 :
        pwm2.stop()


    if counterBR>=960 and counterFL>=960:
        gameover()
        pwm1.stop()
        pwm2.stop()
        print("Thanks")
        break