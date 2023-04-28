import RPi.GPIO as gpio
import numpy as np
import time
import datetime
import serial
import math
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera

from datetime import datetime
import smtplib
from smtplib import SMTP
from smtplib import SMTPException
import email
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.image import MIMEImage
import threading

import random

import multiprocessing





class Robot:

    def __init__(self):
        # Define Constants
        # CONSTANTS
        self.WHEEL_DIA=0.065
        self.WHEEl_BASE=0.205
        self.PI=3.142
        self.GEAR_RATIO=120
        self.TICKS_PER_MREV=8
        self.DIST_PER_REV=self.PI*self.WHEEL_DIA
        self.BUFFER=0.05
        self.BASE_DUTYCYCLE=25
        self.TRUE_ANGLE=0.0
        self.PICKED=0
        self.OPEN=0
        self.TRUE_ANGLE=0

        self.counterBR = np.uint64(0)
        self.counterFL = np.uint64(0)
        self.buttonBR = int(0)
        self.buttonFL = int(0)

        # PINS
        self.L_IN=31
        self.L_OUT=33
        self.R_IN=35
        self.R_OUT=37
        self.EN_LEFT=7
        self.EN_RIGHT=12

        self.pose=[0,0,0]
        self.setup()
        self.yaw=0.0
        self.acc=0.0
        self.ser = serial.Serial('/dev/ttyUSB0', 9600)
        self.count_line=0
        while self.count_line<=10:
            if(self.ser.in_waiting>0):
                self.count_line += 1
                line = self.ser.readline()
        # time.sleep(1)
        self.camera = PiCamera()
        self.camera.resolution = (640, 480)
        self.camera.framerate = 25
        self.rawCapture = PiRGBArray(self.camera, size=(640,480))
        # allow the camera to warmup
        time.sleep(0.1)

        self.Kp = 6
        self.Ki = 0.01
        self.Kd = 0.4
        self.setpoint = 0
        self.last_error = 0
        self.integral = 0

    def compute_distance_and_angle(self,current, goal):
        # Extract x and y coordinates for both points
        x1, y1 = current
        x2, y2 = goal

        # Compute distance between the points using Pythagoras theorem
        distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

        # Compute angle between the points using trigonometry
        angle = math.atan2(y2 - y1, x2 - x1) * 180 / math.pi

        return distance, angle

    def get_location(self,RP):
        Theta=RP[2]
        X=RP[0]
        Y=RP[1]
        TOTAL_TICKS=(self.counterBR+self.counterFL)/2
        self.counterFL,self.counterBR=0,0
        MOTOR_REV=TOTAL_TICKS/self.TICKS_PER_MREV
        WHEEL_REV=MOTOR_REV/self.GEAR_RATIO
        DISTANCE=WHEEL_REV*self.DIST_PER_REV
        self.get_yaw()
        Theta=math.radians(self.yaw)
        X=X+math.cos(Theta)*DISTANCE
        Y=Y+math.sin(Theta)*DISTANCE

        return X,Y,self.yaw,DISTANCE


    def get_yaw(self):
            for i in range(10):
                if(self.ser.in_waiting>0):
                    self.count_line += 1
                    line = self.ser.readline()
                    if self.count_line> 10:
                            line = line.rstrip().lstrip()
                            line = str(line)
                            line = line.strip("'")
                            line = line.strip("b'")
                            values = line.split("\\t")
                            _,x=values[0].split(": ")
                            _,acc_x=values[3].split(": ")
                            self.yaw=float(x)
                            self.acc=acc_x
                    else:
                        print("Initialization")

        # return yaw
        

    def pwm_setup(self, A, B,C,D):
# self.L_IN,self.R_OUT,self.L_OUT,self.R_IN
        self.pwm1=gpio.PWM(A,50)
        self.pwm1.start(0)
        self.pwm2=gpio.PWM(B,50)
        self.pwm2.start(0)
        self.pwm3=gpio.PWM(C,50)
        self.pwm3.start(0)
        self.pwm4=gpio.PWM(D,50)
        self.pwm4.start(0)
        
        time.sleep(0.1)

    def setup(self):
        gpio.setmode(gpio.BOARD)
        # gpio.cleanup()
        gpio.setup(self.L_IN, gpio.OUT)
        gpio.setup(self.L_OUT, gpio.OUT)
        gpio.setup(self.R_IN, gpio.OUT)
        gpio.setup(self.R_OUT, gpio.OUT)
        gpio.setup(self.EN_LEFT, gpio.IN, pull_up_down = gpio.PUD_UP)
        gpio.setup(self.EN_RIGHT, gpio.IN, pull_up_down = gpio.PUD_UP)
        # Add the interrupt event to the GPIO pin for both rising and falling edges
        gpio.add_event_detect(self.EN_LEFT, gpio.BOTH, callback=self.update_FL)
        # Add the interrupt event to the GPIO pin for both rising and falling edges
        gpio.add_event_detect(self.EN_RIGHT, gpio.BOTH, callback=self.update_BR)
        gpio.setup(36, gpio.OUT)
        self.servo=gpio.PWM(36,50)
        self.servo.start(5)


        
    
    def update(self, angle):
        # Compute error
        error = abs(angle)
        
        # Compute proportional term
        proportional_term = self.Kp * error
        
        # Compute integral term
        self.integral += error
        integral_term = self.Ki * self.integral
        
        # Compute derivative term
        derivative_term = self.Kd * (error - self.last_error)
        self.last_error = error
        
        # Compute control variable
        control_variable = proportional_term + integral_term + derivative_term
        
        return abs(control_variable)

    def control_angle(self,angle,direction):
        self.BASE_DUTYCYCLE=0.0
        control_val=self.update(angle)
        
        if direction==1:
            self.pwm1.ChangeDutyCycle(min(self.BASE_DUTYCYCLE+control_val,50.0))
            self.pwm4.ChangeDutyCycle(min(self.BASE_DUTYCYCLE+control_val,50.0))
            self.pwm2.ChangeDutyCycle(self.BASE_DUTYCYCLE)
            self.pwm3.ChangeDutyCycle(self.BASE_DUTYCYCLE)
        elif direction==2:
            self.pwm1.ChangeDutyCycle(self.BASE_DUTYCYCLE)
            self.pwm4.ChangeDutyCycle(self.BASE_DUTYCYCLE)
            self.pwm2.ChangeDutyCycle(min(self.BASE_DUTYCYCLE+control_val,50.0))
            self.pwm3.ChangeDutyCycle(min(self.BASE_DUTYCYCLE+control_val,50.0))
        

    def update_BR(self,channel):
      
        if int(gpio.input(channel)) != int(self.buttonBR):
            self.buttonBR = int(gpio.input(channel))
            self.counterBR+=1
        
    
    def update_FL(self,channel):
        if int(gpio.input(channel)) != int(self.buttonFL):
            self.buttonFL = int(gpio.input(channel))
            self.counterFL+=1
        

    def control(self):
        KP=1.5
        self.BASE_DUTYCYCLE=40
        error=self.TRUE_ANGLE-self.yaw
        print(error)
        if error<0:
            self.pwm1.ChangeDutyCycle(min(self.BASE_DUTYCYCLE+error*KP,100))
            self.pwm2.ChangeDutyCycle(self.BASE_DUTYCYCLE)
            self.pwm3.ChangeDutyCycle(0)
            self.pwm4.ChangeDutyCycle(0)
        elif error>0:
            self.pwm2.ChangeDutyCycle(min(max(0,self.BASE_DUTYCYCLE-error*KP),100))
            self.pwm4.ChangeDutyCycle(0)
            self.pwm1.ChangeDutyCycle(self.BASE_DUTYCYCLE)
            self.pwm3.ChangeDutyCycle(0)
        else:
            self.pwm1.ChangeDutyCycle(self.BASE_DUTYCYCLE)
            self.pwm2.ChangeDutyCycle(self.BASE_DUTYCYCLE)
            self.pwm3.ChangeDutyCycle(0)
            self.pwm4.ChangeDutyCycle(0)

    
    def stop(self):
        self.pwm3.ChangeDutyCycle(0)
        self.pwm4.ChangeDutyCycle(0)
        self.pwm1.ChangeDutyCycle(0)
        self.pwm2.ChangeDutyCycle(0)


    def main(self):
        
        # self.pwm_setup(self.L_IN,self.R_OUT,self.L_OUT,self.R_IN)
        # Define variables for frame rate and status display
        # Loop through each frame of the video
        for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
            # grab the raw NumPy array representing the image, then initialize the timestamp
            # and occupied/unoccupied text
            img = frame.array
            img=cv2.flip(img,0)
            img=cv2.flip(img,1)
            self.get_yaw()
            print(self.yaw)
            cv2.putText(img, str(self.yaw), (100,400), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
                # show the frame
            cv2.imshow("Frame", img)
            key = cv2.waitKey(1) & 0xFF
            # clear the stream in preparation for the next frame
            self.rawCapture.truncate(0)
            # if the `q` key was pressed, break from the loop
            if key == ord("q"):
                break
        
          
            
    def main2(self):
        camera_process = multiprocessing.Process(target=self.main)
        camera_process.start()

        # Start your function in a separate process
        function_process = multiprocessing.Process(target=self.get_yaw)
        function_process.start()

        # Wait for the processes to finish
        camera_process.join()
        function_process.join()


if __name__ == "__main__":
    r=Robot()
    r.main()



        






