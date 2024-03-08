import RPi.GPIO as gpio
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np

# Set up ultrasonic sensor
TRIG = 16
ECHO = 18

# Set up motor driver pins
IN1 = 31
IN2 = 33
IN3 = 35
IN4 = 37



# Initialize the Raspberry Pi camera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 25
rawCapture = PiRGBArray(camera, size=(640,480))
# Allow the camera to warmup
time.sleep(0.1)

# Define function to calculate distance
def distance():
    pulse_end=0
    pulse_start=0
    gpio.setmode(gpio.BOARD)
    gpio.setup(TRIG,gpio.OUT)
    gpio.setup(ECHO,gpio.IN)

    #Ensure output has no value
    gpio.output(TRIG, False)
    time.sleep(0.01)

    #Genereate TRIGger pulse
    gpio.output(TRIG,True)
    time.sleep(0.00001)
    gpio.output(TRIG, False)

    #Generate ECHO time signal
    while gpio.input(ECHO) == 0:
        pulse_start = time.time()

    while gpio.input(ECHO)== 1:
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
    gpio.setup(IN1, gpio.OUT)
    gpio.setup(IN2, gpio.OUT)
    gpio.setup(IN3, gpio.OUT)
    gpio.setup(IN4, gpio.OUT)
    

    

def gameover():
    init()
    gpio.output(IN1, False)
    gpio.output(IN2, False)
    gpio.output(IN3, False)
    gpio.output(IN4, False)

def forward(tf):
    init()
    # Left Wheels
    gpio.output(IN1, True)
    gpio.output(IN2, False)

    # Right Wheels
    gpio.output(IN3, False)
    gpio.output(IN4, True)

    #Wait 
    time.sleep(tf)

    # Send all pins low & cleanup
    gameover()
    gpio.cleanup()

def pivotleft(tf):
    init()
    # Left Wheels
    gpio.output(IN1, False)
    gpio.output(IN2, True)

    # Right Wheels
    gpio.output(IN3, False)
    gpio.output(IN4, True)

    #Wait 
    time.sleep(tf)

    # Send all pins low & cleanup
    gameover()
    gpio.cleanup()

def reverse(tf):
    init()
    # Left Wheels
    gpio.output(IN1, False)
    gpio.output(IN2, True)

    # Right Wheels
    gpio.output(IN3, True)
    gpio.output(IN4, False)

    #Wait 
    time.sleep(tf)

    # Send all pins low & cleanup
    gameover()
    gpio.cleanup()

def pivotright(tf):
    init()
    # Left Wheels
    gpio.output(IN1, True)
    gpio.output(IN2, False)

    # Right Wheels
    gpio.output(IN3, True)
    gpio.output(IN4, False)

    #Wait 
    time.sleep(tf)

    # Send all pins low & cleanup
    gameover()
    gpio.cleanup()

def move(t):
    t=int(t)
    gpio.setmode(gpio.BOARD)
    gpio.setup(36, gpio.OUT)
    pwm=gpio.PWM(36,50)
    pwm.start(8)
    pwm.ChangeDutyCycle(t)
    time.sleep(1)


font = cv2.FONT_HERSHEY_SIMPLEX #Setting the Font
textcolorf = (0,255,0)
textcolor = (100,200,0)

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('hw6.avi', fourcc, 10, (640, 480))

# Main loop
delay=0.1
w,s,a,d,x,o,c=0,0,0,0,0,0,0
# Keep looping
i=0
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=False):
# Grab the current frame
    image = frame.array
    image=cv2.flip(image,0)
    image=cv2.flip(image,1)
    # Show the frame to our screen
    print("Distance: ",distance())
    cv2.putText(image,'Distance: '+str(distance()),(20,30), font,0.5,textcolorf,1,cv2.LINE_AA)
    cv2.putText(image,f'Image {i} Captured',(20,50), font,0.5,textcolorf,1,cv2.LINE_AA)
    # Write frame to video file
    cv2.imshow("Frame", image)
   
    
    out.write(image)
    
    key = cv2.waitKey(1) & 0xFF
    # Clear the stream in preparation for the next frame
    rawCapture.truncate(0)
    
    if key == ord("q"):    
        break
    if key == ord("e"):
        cv2.imwrite(f'image_{i}.jpg',image)
        i+=1  
    
    if key == ord('w') or w==1:   # Move forward
        w=1
        s,a,d,x,o,c=0,0,0,0,0,0
        forward(delay)
    if key == ord('s') or s==1: # Move backward
        s=1
        w,a,d,x,o,c=0,0,0,0,0,0
        reverse(delay)
    if key == ord('a') or a==1: # Turn left
        a=1
        s,w,d,x,o,c=0,0,0,0,0,0
        pivotleft(delay)
    if key == ord('d') or d==1: # Turn right
        d=1
        s,a,w,x,o,c=0,0,0,0,0,0
        pivotright(delay)

    if key == ord('x') or x==1: # Stop
        x=1
        s,a,d,w,o,c=0,0,0,0,0,0
        gameover()
        gpio.cleanup()

    if key == ord('o') :
        print("Servo Open")
        move(13)
       
    if key== ord('c') :
        print("Servo Close")
        move(7)



gpio.cleanup()
