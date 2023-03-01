# import the necessary packages
import RPi.GPIO as gpio
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2

#Define pin allocations
trig=16
echo=18

# Define function to calculate distance
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

# Initialize the Raspberry Pi camera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 25
rawCapture = PiRGBArray(camera, size=(640,480))
# Allow the camera to warmup
time.sleep(0.1)

font = cv2.FONT_HERSHEY_SIMPLEX #Setting the Font
textcolorf = (0,255,0)
textcolor = (100,200,0)

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('distance.avi', fourcc, 10, (640, 480))


# Keep looping
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=False):
# Grab the current frame
    image = frame.array
    image=cv2.flip(image,0)
    image=cv2.flip(image,1)
    # Show the frame to our screen

    cv2.putText(image,'Distance: '+str(distance()),(20,30), font,0.5,textcolorf,1,cv2.LINE_AA)
    # Write frame to video file
    out.write(image)
    cv2.imshow("Frame", image)
    key = cv2.waitKey(1) & 0xFF
    # Clear the stream in preparation for the next frame
    rawCapture.truncate(0)
    
    if key == ord("q"): 
        cv2.imwrite('capture_distance.jpg',image)     
        break