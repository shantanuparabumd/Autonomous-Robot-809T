# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import datetime
f = open('hw3data.txt','a') 

# initialize the Raspberry Pi camera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 25
rawCapture = PiRGBArray(camera, size=(640,480))
# allow the camera to warmup
time.sleep(0.1)

# Define the color range for the ball (in HSV format)
lower_color = np.array([73, 120, 199])
upper_color = np.array([76, 163, 233])

# Define the kernel size for the morphological operations
kernel_size = 5

# Initialize the list of ball centers
centers = []
# define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('final_video_signal.avi', fourcc, 10, (640*3, 480))


font = cv2.FONT_HERSHEY_SIMPLEX #Setting the Font
textcolorf = (0,255,0)
textcolor = (100,200,0)

# Loop through each frame of the video
for image in camera.capture_continuous(rawCapture, format="bgr", use_video_port=False):
    # Start time
    start_time = datetime.datetime.now()
    # grab the current frame
    frame = image.array
    frame=cv2.flip(frame,0)
    frame=cv2.flip(frame,1)


    # Convert the frame to the HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Create a blank mask
    mask = np.zeros((hsv.shape[0], hsv.shape[1]), dtype=np.uint8)
    # Create a mask for the ball color using cv2.inRange()
    mask = cv2.inRange(hsv, lower_color, upper_color)

    # Apply  operations to remove noise and fill in gaps
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    mask = cv2.erode(mask, kernel)
    mask = cv2.dilate(mask, kernel)

    # Find contours in the mask using cv2.findContours()
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Loop through each contour
    for contour in contours:
        # Compute the center and radius of the contour using cv2.moments()
        M = cv2.moments(contour)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        radius = int(np.sqrt(M["m00"] / np.pi))
        # image=cv2.putText(frame,'GREEN',(20,30), font,0.5,textcolorf,1,cv2.LINE_AA)
        cv2.putText(frame,'x: '+str(center[0])+' y: '+str(center[1]),(20,30), font,0.5,textcolorf,1,cv2.LINE_AA)

        # Draw a circle around the ball using cv2.circle()
        cv2.circle(frame, center, radius, (0, 255, 0), 2)
        cv2.circle(frame, center, 1, (0, 0, 255), 2)


        # Add the center to the list of centers
        centers.append(center)
    # Display the RGB, HSV, and masked images using NumPy's hstack() function
    rgb_hsv_masked = np.hstack((frame, hsv, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)))
    cv2.imshow('RGB, HSV, and Masked Images', rgb_hsv_masked)
    # Write the frame to video
    out.write(rgb_hsv_masked)

    key = cv2.waitKey(1) & 0xFF
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
    # End time
    end_time = datetime.datetime.now()
    #Calculating and saving time to file
    now=end_time-start_time
    outstring = str(now.total_seconds())+'\n'
    f.write(outstring)
    print(now.total_seconds())

    # press the 'q' key to stop the video stream
    if key == ord("q"):       
        break


