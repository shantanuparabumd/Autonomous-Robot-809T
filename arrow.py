# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import datetime
import math


fl = open('hw4data.txt','a') 
# initialize the Raspberry Pi camera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 25
rawCapture = PiRGBArray(camera, size=(640,480))
# allow the camera to warmup
time.sleep(0.1)

#Detection Parameters
angle_tolerance=30
confidence_threshold=80
# define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('arrow_final.avi', fourcc, 10, (640*3, 480))


font = cv2.FONT_HERSHEY_SIMPLEX #Setting the Font
textcolorf = (0,255,0)
textcolor = (100,200,0)

# Define the lower and upper bounds of the green color range in HSV
lower_green = np.array([72, 103, 196])
upper_green = np.array([104, 202, 255])
kernel_size = 5

# Define variables for frame rate and status display
fps = 0
status = "Detecting..."
confidence=0
direction=str()
# Get the current time in seconds since the epoch
start_time = time.time()
# Loop through each frame of the video
f=0
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=False):
    # Start time
    start_time = datetime.datetime.now()
    
    # grab the current frame
    img = frame.array
    img=cv2.flip(img,0)
    img=cv2.flip(img,1)

    # Convert the image to the HSV color space
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Define the lower and upper bounds of the green color range in HSV
    lower_green = np.array([48, 46, 213])
    upper_green = np.array([85, 168, 248])

    # Create a mask to isolate green pixels
    mask = cv2.inRange(hsv_img, lower_green, upper_green)

    # Apply erosion and dilation to remove noise and make corners smooth
    kernel = np.ones((5,5),np.uint8)
    erosion = cv2.erode(mask, kernel, iterations = 1)
    dilation = cv2.dilate(erosion, kernel, iterations = 1)

    # Apply gaussian blur to he mask
    mask_blur = cv2.GaussianBlur(dilation, (5, 5), 0)

    # Find the contours in the binary image
    contours, hierarchy = cv2.findContours(mask_blur, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours)==0:
        # Default the readings
        status = "Detecting..."
        confidence=0
        direction=""
    else:
        # Update status
        status = "Green Object Detected"

        # Find the maximum contour
        max_contour = max(contours, key=cv2.contourArea)

        # Draw a bounding box around the max contour and mask everything else
        x,y,w,h = cv2.boundingRect(max_contour)
        tolerance=0
        mask = np.zeros(mask_blur.shape, np.uint8)
        mask[y:y+h+tolerance, x:x+w+tolerance] = mask_blur[y:y+h+tolerance, x:x+w+tolerance]    
          
        # Use Shi-Thomas feature detection
        corners = cv2.goodFeaturesToTrack(mask,5,0.35,10,15,blockSize=15)

        # Converting to integers
        corners = np.int0(corners)

        # Converting to numpy array for easy operation
        corners=np.array(corners)

        # Drawing the corners
        for corner in corners:
            x,y = corner.ravel()
            cv2.circle(img,(x,y),3,255,-1)
            
        # Checking for enough corners to make prediction
        if len(corners)>=5:
            # Updating the status
            status = "Arrow Detected"

            # Getting the center of the arrow
            M = cv2.moments(max_contour)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
 
            # Comparing the width and height of the contour to predict orientation
            if w>h:
                # Count elements greater than and less than center x co ordinate
                greater = np.sum(corners[:, 0, 0] > cX)
                less = np.sum(corners[:, 0, 0] < cX)
                if greater>less:
                    direction="Right"
                else:
                    direction="Left"
            else:
                # Count elements greater than and less than center y co ordinate
                greater = np.sum(corners[:, 0, 1] > cY)
                less = np.sum(corners[:, 0, 1] < cY)
                if greater>less:
                    direction="Down"
                else:
                    direction="Up"
    
    # Showing the direction on image
    cv2.putText(img, direction, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    
    # Display the frame,  and status text
    text = "Status: {} | Frames: {:.2f}".format( status,f)
    cv2.putText(img, text, (10, 460), cv2.FONT_HERSHEY_SIMPLEX,  0.5, (255, 255, 255), 1)

    # Increment frame count
    f=f+1

    # Stack the original image, HSV image, and mask horizontally
    output = np.hstack((img, hsv_img, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)))

    # Display the output
    cv2.imshow("Arrow Image", output)

    # Write the frame to video
    out.write(output)
    

    key = cv2.waitKey(1) & 0xFF
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)

    # End time
    end_time = datetime.datetime.now()

    #Calculating and saving time to file
    now=end_time-start_time
    outstring = str(now.total_seconds())+'\n'

    # Writing to file
    fl.write(outstring)
    # press the 'q' key to stop the video stream
    if key == ord("q"):       
        break
