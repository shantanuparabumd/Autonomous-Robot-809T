import RPi.GPIO as gpio
import time

#Define pin allocations
trig=16
echo=18


pwm=gpio.PWM(36,50)
pwm.start(8)

def distance():
    gpio.setmode(gpio.BOARD)
    gpio.setup(trig,gpio.OUT)
    gpio.setup(echo,gpio.IN)

    #Ensure output has no value
    gpio.output(trig, False)
    time.sleep(0.01)

    #Genereate trigger pulse
    gpio.output(trig,True)
    time.sleep(0.00001)
    gpio.output(trig, False)

    #Generate echo time signal
    while gpio.input(echo) == 0:
        pulse_start = time.time()

    while gpio.input(echo)== 1:
        pulse_end = time.time()

    pulse_duration = pulse_end-pulse_start

    #Convert time to distance
    distance = pulse_duration*17150
    distance = round(distance, 2)

    #Cleanup gpio 7 return distance
    gpio.cleanup()
    return distance

def init():
    gpio.setmode(gpio.BOARD)
    # gpio.cleanup()
    gpio.setup(31, gpio.OUT)
    gpio.setup(33, gpio.OUT)
    gpio.setup(35, gpio.OUT)
    gpio.setup(37, gpio.OUT)

    gpio.setup(36, gpio.OUT)
    

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
    time.sleep(tf/5)

    # Send all pins low & cleanup
    gameover()
    gpio.cleanup()

def pivotleft(tf):
    init()
    # Left Wheels
    gpio.output(31, False)
    gpio.output(33, True)

    # Right Wheels
    gpio.output(35, False)
    gpio.output(37, True)

    #Wait 
    time.sleep(tf)

    # Send all pins low & cleanup
    gameover()
    gpio.cleanup()

def reverse(tf):
    init()
    # Left Wheels
    gpio.output(31, False)
    gpio.output(33, True)

    # Right Wheels
    gpio.output(35, True)
    gpio.output(37, False)

    #Wait 
    time.sleep(tf/5)

    # Send all pins low & cleanup
    gameover()
    gpio.cleanup()

def pivotright(tf):
    init()
    # Left Wheels
    gpio.output(31, True)
    gpio.output(33, False)

    # Right Wheels
    gpio.output(35, True)
    gpio.output(37, False)

    #Wait 
    time.sleep(tf)

    # Send all pins low & cleanup
    gameover()
    gpio.cleanup()

def move(t):
    pwm.ChangeDutyCycle(t)



def key_input(event):
    init()
    print("Key: ",event)
    print("Distance: ",distance())
    key_press =event
    tf=2

    if key_press.lower() == 'w':
        forward(tf)
    elif key_press.lower() == 'z':
        reverse(tf)
    elif key_press.lower() == 'a':
        pivotleft(tf)
    elif key_press.lower() == 's':
        pivotright(tf)
    elif key_press.lower() == 'o':
        move(13)
    elif key_press.lower() == 'c':
        move(7)

while True:
    key_press=input("Select driving mode: ")
    if key_press == 'p':
        break
    key_input(key_press)