import RPi.GPIO as gpio
import numpy as np
import time
import datetime

# CONSTANTS
WHEEL_DIA=0.065
PI=3.142
GEAR_RATIO=120
TICKS_PER_MREV=8
DIST_PER_REV=PI*WHEEL_DIA
BUFFER=5

fl = open('hw7encoderdata.txt','a') 

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


BASE_DUTYCYCLE=25
pwm1=gpio.PWM(31,50)
pwm1.start(0)
pwm2=gpio.PWM(37,50)
pwm2.start(0)
time.sleep(0.1)

counter = np.uint64(0)
button = int(0)
# Get the current time in seconds since the epoch
start_time = datetime.datetime.now()
DISTANCE=float(input("Enter the distance to be travelled: "))
WHEEL_REV=DISTANCE/DIST_PER_REV
MOTOR_REV=WHEEL_REV*GEAR_RATIO
TOTAL_TICKS=(TICKS_PER_MREV*MOTOR_REV)
print("TOTAL TICKS: ",TOTAL_TICKS)
TOTAL_TICKS=int(TOTAL_TICKS-MOTOR_REV*0.78)
print("TOTAL TICKS INT: ",TOTAL_TICKS)

KP=3
pwm2.ChangeDutyCycle(BASE_DUTYCYCLE)
pwm1.ChangeDutyCycle(BASE_DUTYCYCLE)
while counterBR<=TOTAL_TICKS or counterFL<TOTAL_TICKS:
    line=str(buttonBR)+" "+str(buttonFL)+"\n"
    fl.write(line)
    # print("countBR: ", counterBR, "counterFL: ", counterFL, "BR state: ", gpio.input(12), "FL state: ", gpio.input(7))
    if int(gpio.input(12)) != int(buttonBR):
         buttonBR = int(gpio.input(12))
         counterBR+=1

    if int(gpio.input(7)) != int(buttonFL):
         buttonFL = int(gpio.input(7))
         counterFL+=1
    
    # print("Value: ",val)
    # val=int(input("Enter PWM value: "))
    error=counterBR-counterFL
    # print(error)
    if error>0:
        # print("Duty Cycle: Left: ",BASE_DUTYCYCLE+error*KP," Right: ",BASE_DUTYCYCLE)
        pwm1.ChangeDutyCycle(BASE_DUTYCYCLE+error*KP)
        pwm2.ChangeDutyCycle(BASE_DUTYCYCLE)
    elif error<0:
        # print("Duty Cycle: Left: ",BASE_DUTYCYCLE," Right: ",BASE_DUTYCYCLE-error*KP)
        pwm2.ChangeDutyCycle(BASE_DUTYCYCLE-error*KP)
        pwm1.ChangeDutyCycle(BASE_DUTYCYCLE)

if int(gpio.input(12)) != int(buttonBR):
    buttonBR = int(gpio.input(12))
    counterBR+=1

if int(gpio.input(7)) != int(buttonFL):
    buttonFL = int(gpio.input(7))
    counterFL+=1
print("countBR: ", counterBR, "counterFL: ", counterFL, "BR state: ", gpio.input(12), "FL state: ", gpio.input(7))
gameover()
pwm1.stop()
pwm2.stop()
print("Thanks Distance Reached")
fl.close()
