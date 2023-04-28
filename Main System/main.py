from robot import Localization as loc
import RPi.GPIO as gpio
import numpy as np
import time
import datetime
import serial
import math
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera

class Robot():
    def __init__(self) -> None:
        self.robot_pose=[0,0,0]
        self.blocks=[]
        self.STEP=0.2
        self.counterBR=0
        self.counterFL=0

        self.WHEEL_DIA=0.065
        self.WHEEl_BASE=0.205
        self.PI=3.142
        self.GEAR_RATIO=120
        self.TICKS_PER_MREV=8
        self.DIST_PER_REV=self.PI*self.WHEEL_DIA

        #STATES
        self.QR_DETECTED=0
        self.MAP
        
        
    def step(self,angle):
        KP=5
        self.BASE_DUTYCYCLE=40
        if int(gpio.input(self.EN_RIGHT)) != int(self.buttonBR):
            self.buttonBR = int(gpio.input(self.EN_RIGHT))
            self.counterBR+=1
        if int(gpio.input(self.EN_LEFT)) != int(self.buttonFL):
            self.buttonFL = int(gpio.input(self.EN_LEFT))
            self.counterFL+=1
        error=loc.get_yaw()-angle
        if error<0:
            self.pwm1.ChangeDutyCycle(min(self.BASE_DUTYCYCLE+error*KP,100))
            self.pwm2.ChangeDutyCycle(self.BASE_DUTYCYCLE)
            self.pwm3.ChangeDutyCycle(0)
            self.pwm4.ChangeDutyCycle(0)
        elif error>0:
            self.pwm2.ChangeDutyCycle(min(self.BASE_DUTYCYCLE-error*KP,100))
            self.pwm4.ChangeDutyCycle(0)
            self.pwm1.ChangeDutyCycle(self.BASE_DUTYCYCLE)
            self.pwm3.ChangeDutyCycle(0)
        else:
            self.pwm1.ChangeDutyCycle(self.BASE_DUTYCYCLE)
            self.pwm2.ChangeDutyCycle(self.BASE_DUTYCYCLE)
            self.pwm3.ChangeDutyCycle(0)
            self.pwm4.ChangeDutyCycle(0)


    def get_position(self,px):
        # FOV in degree
        hfov=62.2
        dppx=hfov/640

        if px>=640/2:
            px=px-(640/2)
        else:
            px=((640/2)-px)*-1
        azimuth=dppx*px
        return azimuth
    
        
    def preprocess(self,frame):
        # grab the current frame
            img = frame.array
            img=cv2.flip(img,0)
            img=cv2.flip(img,1)

            # Convert the image to the HSV color space
            hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            return img,hsv_img
    
    def detect_block(hsv_img):
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
            contours = sorted(contours, key=cv2.contourArea, reverse=True)[:1]

            if len(contours)==0:
                # Default the readings
                status = "Detecting..."
            else:
                # Update status
                status = "Red Block Detected"

                # Find the maximum contour
                max_contour = max(contours, key=cv2.contourArea)
                for max_contour in contours:
                    # Draw a bounding box around the max contour and mask everything else
                    x,y,w,h = cv2.boundingRect(max_contour)
                    tolerance=0
                    mask = np.zeros(mask_blur.shape, np.uint8)
                    mask[y:y+h+tolerance, x:x+w+tolerance] = mask_blur[y:y+h+tolerance, x:x+w+tolerance]   

            return x+(w/2),y+(h/2) 

    def detect_qr(self,img):
        pass

    def checkMap(self):
        pass

    def updateMap(self):
        pass

    def blockPose(self):
        pass

    def move(self):
        pass

    def main(self):

        loc.calibrate()
        self.robot_pose=loc.get_location(self.counterBR,self.counterFL,self.robot_pose)
        # Define variables for frame rate and status display
        # Get the current time in seconds since the epoch
        # Loop through each frame of the video
        for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=False):
            # Start time
            img,hsv_img=self.preprocess(frame)
            if not self.detect_qr(hsv_img):
                print("Checking For QR")

            
            while self.checkMap()!=9:
                blocks=self.blockPose()
                self.robot_pose=loc.get_location(self.counterBR,self.counterFL,self.robot_pose)
                self.updateMap(self.robot_pose,blocks)
                self.move()
                
            
            x,y=self.detect_block(hsv_img)
            cv2.circle(img, (int(x),int(y)), 2, (0,0,0), -1)
            cv2.circle(img, (int(x),int(y)), 20, (0,255,255), 1)
            angle=self.get_position(x)
            cv2.putText(img, str(angle), (100,400), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
            # Display the output
            cv2.imshow("Output", img)
            key = cv2.waitKey(1) & 0xFF
            # clear the stream in preparation for the next frame
            self.rawCapture.truncate(0)

            if angle>0:
                self.angle_control(angle,1)
                print("Clockwise")
            else:
                self.angle_control(angle,2)
                print("CounterClockwise")
            if key == ord("q"):       
                break
    
        
    
    # Calibrate location
    # Read QR
    # Map Area

    # Plan Order of Block Picking
    # While all blocks not picked
        # if to block 
            # Create Plan From Current to Goal
            # Reach Goal
            # Local Planning  
            # Pick Block
            
        # if to consrtuctio zone
            # Create Plan From Current to Goal
            # Reach Goal
    #End