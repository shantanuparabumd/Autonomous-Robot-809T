# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import datetime
import math


# fl = open('hw4data.txt','a') 
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
# fourcc = cv2.VideoWriter_fourcc(*'XVID')
# out = cv2.VideoWriter('arrow_final.avi', fourcc, 10, (640*3, 480))


font = cv2.FONT_HERSHEY_SIMPLEX #Setting the Font
textcolorf = (0,255,0)
textcolor = (100,200,0)

# Define the lower and upper bounds of the green color range in HSV
lower_green = np.array([72, 103, 196])
upper_green = np.array([104, 202, 255])
kernel_size = 5


def get_position(px,py,height):
    block_height=57.15 #in mm
    h=block_height/2 # centre of bolck
#     print(h)
    H=135 #height of camera in mm
    # FOV in degree
    hfov=62.2
    vfov=48.8
    dppx=hfov/640
    dppy=vfov/480
#     print(dppx)
#     print(dppy)
    if px>=640/2:
        px=px-(640/2)
    else:
        px=((640/2)-px)*-1
    if py>480/2:
        py=(py-(480/2))*-1
    else:
        py=(480/2)-py
    azimuth=dppx*px
    polar=dppy*py
    r=(h-H)/math.sin(np.deg2rad(polar))
    rprime=r*math.cos(np.deg2rad(polar))
    x=rprime*math.cos(np.deg2rad(azimuth))
    y=rprime*math.sin(np.deg2rad(azimuth))
    z=h-H
    frame_height=480
    focal_length = 3.04
    sensor_height=2.76
    x = (focal_length*block_height*frame_height)/(height*sensor_height)
    # print(distance)
    return x,y,z

# Define variables for frame rate and status display
fps = 0
status = "Detecting..."
confidence=0
distance=0
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
    lower_green = np.array([91, 147, 0])
    upper_green = np.array([179, 255, 255])

    # Create a mask to isolate green pixels
    mask = cv2.inRange(hsv_img, lower_green, upper_green)

    # Apply erosion and dilation to remove noise and make corners smooth
    kernel = np.ones((3,3),np.uint8)
    erosion = cv2.erode(mask, kernel, iterations = 1)
    dilation = cv2.dilate(erosion, kernel, iterations = 1)

    # Apply gaussian blur to he mask
    mask_blur = cv2.GaussianBlur(dilation, (3, 3), 0)

    # Find the contours in the binary image
    contours, hierarchy = cv2.findContours(mask_blur, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)[:3]

    X,Y,Z=0,0,0
    if len(contours)==0:
        # Default the readings
        status = "Detecting..."
        confidence=0
        distance=1
    else:
        # Update status
        status = "Red Block Detected"

        # Find the maximum contour
        # max_contour = max(contours, key=cv2.contourArea)
        for max_contour in contours:
            # Draw a bounding box around the max contour and mask everything else
            x,y,w,h = cv2.boundingRect(max_contour)
            tolerance=0
            mask = np.zeros(mask_blur.shape, np.uint8)
            mask[y:y+h+tolerance, x:x+w+tolerance] = mask_blur[y:y+h+tolerance, x:x+w+tolerance]    
            cv2.rectangle(img, (x, y), (x+w, y+h), (0,0,255), 2)
            X,Y,Z=get_position(x+(w/2),y+(h/2),h)
            cv2.circle(img, (int(x+(w/2)),int(y+(h/2))), 2, (255,0,0), -1)
            # co=str(int(x+(w/2)))+" "+str(int(y+(h/2)))+str(y)+" "+str(h)
            cv2.putText(img, str(X), (int(x+(w/2)),int(y+(h/2))), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

          
       
    
    # Showing the direction on image
    # cv2.putText(img, str(distance), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    
    # Display the frame,  and status text
    # text = "Status: {} | Frames: {:.2f}".format( status,f)
    # cv2.putText(img, text, (10, 460), cv2.FONT_HERSHEY_SIMPLEX,  0.5, (255, 255, 255), 1)
    # pose= "X: {} | Y: {} | Z: {}".format( X,Y,Z)
    # cv2.putText(img, pose, (10, 460), cv2.FONT_HERSHEY_SIMPLEX,  0.5, (255, 255, 255), 1)
    # Increment frame count
    f=f+1

    # Stack the original image, HSV image, and mask horizontally
    output = np.hstack((img, hsv_img, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)))

    # Display the output
    cv2.imshow("Arrow Image", img)

    # Write the frame to video
    # out.write(output)
    

    key = cv2.waitKey(1) & 0xFF
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)

    # End time
    end_time = datetime.datetime.now()

    #Calculating and saving time to file
    now=end_time-start_time
    outstring = str(now.total_seconds())+'\n'

    # Writing to file
    # fl.write(outstring)
    # press the 'q' key to stop the video stream
    if key == ord("q"):       
        break
