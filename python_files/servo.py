import RPi.GPIO as gpio
import time

gpio.setmode(gpio.BOARD)
# gpio.cleanup()
gpio.setup(36, gpio.OUT)
pwm=gpio.PWM(36,50)
pwm.start(5)


def move(t):
    pwm.ChangeDutyCycle(t)

def key_input(event):
    print("Duty Cycle: ",event)
    tf=int(event)

    move(tf)
   
    

while True:
    key_press=input("Select  Duty Cycle: ")
    if key_press == 'p':
        break
    key_input(key_press)

pwm.stop()
gpio.cleanup()