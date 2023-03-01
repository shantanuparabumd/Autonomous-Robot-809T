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
    lower_green = np.array([70, 105, 196])
    upper_green = np.array([145, 222, 255])

    # Create a mask to isolate green pixels
    mask = cv2.inRange(hsv_img, lower_green, upper_green)

    
    # Apply erosion and dilation to remove noise and make corners smooth
    kernel = np.ones((3,3),np.uint8)
    erosion = cv2.erode(mask, kernel, iterations = 1)
    dilation = cv2.dilate(erosion, kernel, iterations = 1)

    # Apply gaussian blur to he mask
    mask_blur = cv2.GaussianBlur(dilation, (3, 3), 0)
    # mask_blur = cv2.addWeighted(mask, 7, mask_blur, -1.5, 0)
    # Find the contours in the binary image
    contours, hierarchy = cv2.findContours(mask_blur, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # # Draw a line on the image
    # start_point = (x_min, y_min)
    # end_point = (x_max, y_max)
    color = (0, 255, 0) # green
    thickness = 2
    # print(contour)
    # # Find the contour with the max area
    if len(contours)==0:
        status = "Detecting..."
        confidence=0
        direction=""
    else:
        status = "Green Object Detected"
        max_contour = max(contours, key=cv2.contourArea)
        # Draw a bounding box around the max contour and mask everything else
        x,y,w,h = cv2.boundingRect(max_contour)
        tolerance=0
        mask = np.zeros(mask_blur.shape, np.uint8)
        mask[y:y+h+tolerance, x:x+w+tolerance] = mask_blur[y:y+h+tolerance, x:x+w+tolerance]
        # cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), thickness=2)
      
        # Use Shi-Thomas feature detection
        corners = cv2.goodFeaturesToTrack(mask,3,0.1,20)
        corners = np.int0(corners)
        corners=np.array(corners)
        
        
        if len(corners)>=2:
            status = "Arrow Detected"
            
        #     corners.sort()
            # Draw rectangle on image
            # Calculate moments
            M = cv2.moments(max_contour)

            # Print centroid (center of mass) coordinates
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            # cv2.circle(img,(cX,cY),3,255,-1)
            # Put text on the image
            text = str(cX)+" "+str(cY)
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.5
            color = (0, 0, 0) # green
            thickness = 1
            # cv2.putText(img, text, (cX, cY), font, font_scale, color, thickness)
            loc=tuple(np.int0((corners[1,0]+corners[0,0])/2))
            # cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), thickness=2)
            # cv2.line(img, tuple([x,y+int(h/2)]), tuple([x+w,y+int(h/2)]), color, thickness)
            # cv2.line(img, tuple([x+int(w/2),y]), tuple([x+int(w/2),y+h]), color, thickness)
            # cv2.line(img, loc, (cX,cY), (255,255,255), thickness)
            dx = cX - loc[0]
            dy = cY - loc[1]
            # calculate the distance between the points
            distance = math.sqrt((dx)**2 + (dy)**2)
            angle = math.atan2(dy, dx) * 180 / math.pi
            tolerance=w/4 if w/4>h/4 else h/4
            confidence=(distance/tolerance)*100
            
            if confidence>confidence_threshold and confidence<150:
                # print(w,h)
                if abs(180 - angle) <= angle_tolerance:
                    direction="Right "
                if abs(0 - angle) <= angle_tolerance:
                    direction="Left "
                if abs(-90 - angle) <= angle_tolerance:
                    direction="Down "
                if abs(90 - angle) <= angle_tolerance:
                    direction="Up "
           
    cv2.putText(img, direction, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    
    # Define constants for text display
    FONT = cv2.FONT_HERSHEY_SIMPLEX
    FONT_SCALE = 0.5
    FONT_COLOR = (255, 255, 255)
    LINE_TYPE = 1
    


    # Draw the frame rate, detection confidence, and status text
    text = "Confidence: {:.2f}% | Status: {} | Frames: {:.2f}".format( confidence, status,f)
    cv2.putText(img, text, (10, 460), FONT, FONT_SCALE, FONT_COLOR, LINE_TYPE)
    f=f+1
    # Stack the original image, HSV image, and mask horizontally
    output = np.hstack((img, hsv_img, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)))

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
    fl.write(outstring)
    # press the 'q' key to stop the video stream
    if key == ord("q"):       
        break
