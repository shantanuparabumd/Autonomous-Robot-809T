import RPi.GPIO as gpio
import numpy as np
import time
import datetime
import serial
import math
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera





class Camera:
    def __init__(self):
        # Define Constants
        self.camera = PiCamera()
        self.camera.resolution = (640, 480)
        self.camera.framerate = 25
        self.rawCapture = PiRGBArray(self.camera, size=(640,480))
        # allow the camera to warmup
        time.sleep(0.1)

        
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
    
    def control_angle(self,angle,direction):
        self.BASE_DUTYCYCLE=0.0
        Kp=6
        error=abs(angle)
        print("PWM: ",min(self.BASE_DUTYCYCLE+error*Kp,100.0))
        if direction==1:
            self.pwm1.ChangeDutyCycle(min(self.BASE_DUTYCYCLE+error*Kp,100.0))
            self.pwm4.ChangeDutyCycle(min(self.BASE_DUTYCYCLE+error*Kp,100.0))
            self.pwm2.ChangeDutyCycle(self.BASE_DUTYCYCLE)
            self.pwm3.ChangeDutyCycle(self.BASE_DUTYCYCLE)
        elif direction==2:
            self.pwm1.ChangeDutyCycle(self.BASE_DUTYCYCLE)
            self.pwm4.ChangeDutyCycle(self.BASE_DUTYCYCLE)
            self.pwm2.ChangeDutyCycle(min(self.BASE_DUTYCYCLE+error*Kp,100.0))
            self.pwm3.ChangeDutyCycle(min(self.BASE_DUTYCYCLE+error*Kp,100.0))
        
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
                # status = "Detecting..."
            else:
                # Update status
                # status = "Red Block Detected"

                # Find the maximum contour
                # max_contour = max(contours, key=cv2.contourArea)
                for max_contour in contours:
                    # Draw a bounding box around the max contour and mask everything else
                    x,y,w,h = cv2.boundingRect(max_contour)
                    tolerance=0
                    mask = np.zeros(mask_blur.shape, np.uint8)
                    mask[y:y+h+tolerance, x:x+w+tolerance] = mask_blur[y:y+h+tolerance, x:x+w+tolerance]   

            return x+(w/2),y+(h/2) 

    def main(self):
        # Define variables for frame rate and status display
        # Get the current time in seconds since the epoch
        # Loop through each frame of the video
        for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=False):
            # Start time
            img,hsv_img=self.preprocess(frame)
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
    
    

        






