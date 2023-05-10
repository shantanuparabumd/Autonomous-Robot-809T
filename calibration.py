import RPi.GPIO as gpio
import numpy as np
import time
import datetime
import serial
import math
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
from queue import PriorityQueue
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







class Robot:

    def __init__(self):

        #Initial Pose
        self.pose=[0.3,0.3,0]

        # CONSTANTS
        self.WHEEL_DIA=0.065
        self.WHEEl_BASE=0.205
        self.PI=3.142
        self.GEAR_RATIO=120
        self.TICKS_PER_MREV=8
        self.DIST_PER_REV=self.PI*self.WHEEL_DIA
        self.BUFFER=0.05
        self.BASE_DUTYCYCLE=100
        self.BASE_DUTYCYCLE_ANGLE=0
        self.BASE_DUTYCYCLE_R=40
        self.TRUE_ANGLE=0.0

        
        # STATUS
        self.COLOR='WHITE'
        self.R_SCORE=0
        self.G_SCORE=0
        self.B_SCORE=0
        self.PICKED=0
        self.SERVO_OPEN=0
        self.MOVING=0
        self.REACHED=0

        # States
        self.EXPLORE=1
        self.SEARCH=0
        self.PICK=0
        self.DROP=0
        self.MOVE_TO_ZONE=0
        self.STATUS=''
        
        # Counters
        self.POSITION=0
        self.DROP_COUNT=0
        self.counterBR = np.uint64(0)
        self.counterFL = np.uint64(0)
        self.buttonBR = int(0)
        self.buttonFL = int(0)
        self.turning=0
        # PINS
        self.L_IN=31
        self.L_OUT=33
        self.R_IN=37
        self.R_OUT=35
        self.EN_LEFT=7
        self.EN_RIGHT=12

        # LED Setup
        self.led_pins = {'RED':22, 'GREEN':26, 'BLUE':24, 'YELLOW':38, 'WHITE':40}
        
        self.setup()

        # # IMU SERIAL SETUP
        # self.yaw=0.0
        # self.acc=0.0
        # self.ser = serial.Serial('/dev/ttyUSB0', 9600)
        # self.count_line=0
        # while self.count_line<=10:
        #     if(self.ser.in_waiting>0):
        #         self.count_line += 1
        #         line = self.ser.readline()
        # # time.sleep(1)
        # self.camera = PiCamera()
        # self.camera.resolution = (640, 480)
        # self.camera.framerate = 25
        # self.rawCapture = PiRGBArray(self.camera, size=(640,480))
        # # allow the camera to warmup
        # time.sleep(0.1)


        #CONTROL LOOP SETUP
        self.Kp = 1.0
        self.Ki = 0.01
        self.Kd = 0.4
        self.Ki=0
        self.Kd=0
        self.setpoint = 0
        self.last_error = 0
        self.integral = 0
        self.set_angle=0

        # VIDEO WRITTER
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = cv2.VideoWriter('hw9_block_retrival.avi', fourcc, 10, (640, 480))

            
    #ROBOT SETUP

    def pwm_setup(self, A, B,C,D):
        "Function to setup the PWM"
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
        "Function to Setup the pins"
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

        for color in self.led_pins:
            gpio.setup(self.led_pins[color], gpio.OUT)


    # LOCALIZATION

    def compute_distance_and_angle(self,current, goal):
        """Function to compute the distance and angle to a particular goaal from current location"""
        # Extract x and y coordinates for both points
        x1, y1 = current
        x2, y2 = goal

        # Compute distance between the points using Pythagoras theorem
        distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

        # Compute angle between the points using trigonometry
        angle = math.atan2(y2 - y1, x2 - x1) * 180 / math.pi

        return distance, angle

    def get_location(self,RP):
        """Function to get the location of the robot using the encoder values and current location"""
        Theta=RP[2]
        X=RP[0]
        Y=RP[1]
        TOTAL_TICKS=(self.counterBR+self.counterFL)/2
        self.counterFL,self.counterBR=0,0
        MOTOR_REV=TOTAL_TICKS/self.TICKS_PER_MREV
        WHEEL_REV=MOTOR_REV/self.GEAR_RATIO
        DISTANCE=(WHEEL_REV*self.DIST_PER_REV)+WHEEL_REV*0.05
        self.update_yaw()
        Theta=math.radians(self.yaw)
        X=X+math.cos(Theta)*DISTANCE
        Y=Y+math.sin(Theta)*DISTANCE
        
        return X,Y,self.yaw,DISTANCE

    def update_yaw(self):
            """Function to update the current yaw of the robot"""
            for i in range(20):
                if(self.ser.in_waiting>0):
                    self.count_line += 1
                    # print(count)
                    # Read serial stream

                    line = self.ser.readline()
                    # print(line)


                    # Avoid first n-liines of the serial information

                    if self.count_line> 10:
                        # Strip serial stream of extra characters

                        line = line.rstrip().lstrip()
                        # print("Original: ",line)

                        line = str(line)
                        line = line.strip("'")
                        line = line.strip("b'")
                        # print("Formatted: ",line)
                        values = line.split("\\t")
                        _,x=values[0].split(": ")
                        _,acc_x=values[3].split(": ")
                        self.yaw=float(x)
                        self.acc=acc_x
                    
                    else:
                        print("Initialization")

    def update_BR(self,channel):
        """Update the Encoder Counts"""
        if self.turning==0:
            if int(gpio.input(channel)) != int(self.buttonBR):
                self.buttonBR = int(gpio.input(channel))
                self.counterBR+=1
    
    def update_FL(self,channel):
        """Update the Encoder Counts"""
        if self.turning==0:
            if int(gpio.input(channel)) != int(self.buttonFL):
                self.buttonFL = int(gpio.input(channel))
                self.counterFL+=1


        
    def get_position(self,px,height):
        """Get the depth and the angle of the block with respect to the robot"""
        block_height=57.15 #in mm
        # FOV in degree
        hfov=62.2
        dppx=hfov/640

        if px>=640/2:
            px=px-(640/2)
        else:
            px=((640/2)-px)*-1
        azimuth=dppx*px
        frame_height=480
        focal_length = 3.04
        sensor_height=2.76
        d = (focal_length*block_height*frame_height)/(height*sensor_height)
        return azimuth,d
    
    def update(self, angle):
        """Control Value for PID"""
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
        """Function to control the angle of robot"""
        self.turning=1
        
        control_val=angle
        print(min(max(control_val,self.BASE_DUTYCYCLE_ANGLE),100.0))
        if direction==1:
            self.pwm1.ChangeDutyCycle(min(max(control_val,self.BASE_DUTYCYCLE_ANGLE),100.0))
            self.pwm4.ChangeDutyCycle(min(max(control_val,self.BASE_DUTYCYCLE_ANGLE),100.0))
            self.pwm2.ChangeDutyCycle(0)
            self.pwm3.ChangeDutyCycle(0)
        elif direction==2:
            self.pwm1.ChangeDutyCycle(0)
            self.pwm4.ChangeDutyCycle(0)
            self.pwm2.ChangeDutyCycle(min(max(control_val,self.BASE_DUTYCYCLE_ANGLE),100.0))
            self.pwm3.ChangeDutyCycle(min(max(control_val,self.BASE_DUTYCYCLE_ANGLE),100.0))
        self.counterBR,self.counterFL=0,0

    

    def control(self):
        """Function to move the robot Forward"""
        self.turning=0
        self.counterBR,self.counterFL=0,0
        self.pwm1.ChangeDutyCycle(self.BASE_DUTYCYCLE)
        self.pwm2.ChangeDutyCycle(self.BASE_DUTYCYCLE)
        self.pwm3.ChangeDutyCycle(0)
        self.pwm4.ChangeDutyCycle(0)

    def gameover(self):
        """End Run"""
        self.pwm3.ChangeDutyCycle(0)
        self.pwm4.ChangeDutyCycle(0)
        self.pwm1.ChangeDutyCycle(0)
        self.pwm2.ChangeDutyCycle(0)
        gpio.cleanup()
    
    def stop(self):
        """Stop Robot"""
        self.pwm3.ChangeDutyCycle(0)
        self.pwm4.ChangeDutyCycle(0)
        self.pwm1.ChangeDutyCycle(0)
        self.pwm2.ChangeDutyCycle(0)
    

    def block_in_gripper(self,x,y,h,w):
        """Check if the block is in the gripper"""
        if (x+w/2)>275 and (x+w/2)<360 and (y+h/2)>440:
            return True
        else:
            return False
        
    
    def check_goal(self,x,y,cx,cy):
        """Check if the robot has reached the goal position"""
        distance = math.sqrt((cx - x)**2 + (cy - y)**2)
        if distance <= 0.05:
            return True
        else:
            return False
        
                        

    def explore(self,img):
        """Explore the given area to search for blocks"""
        buffer=3
        X,Y,Theta,c_distance=self.get_location(self.pose)
        self.pose=[X,Y,Theta]
        angle=self.yaw
        
        # loc=[[0.25,0.25,90],[0.25,0.0,180],[0.8,0.8,10],[0.8,0.0,60],[1.2,0.4,45],[0.5,0.8,25]]
        # loc=[[0.0,0.0,45],[1.0,0.0,135],[1,0.5,180],[0.8,0.8,200],[0.5,0.8,270]]
        # loc=[[0.3,0.3,45],[1.5,0.3,90],[2.5,1.5,180],[1.5,2.5,270],[0.5,1.5,0]]
        loc=[[0.3,0.3,45],[0.8,0.8,270]]
        x,y,t=loc[self.POSITION]
        if self.MOVING==0:
            distance,g_angle=self.compute_distance_and_angle(self.pose[:2],(x,y))
            self.set_angle=g_angle

        # cv2.putText(img,f"X: {x} Y: {y} T: {self.set_angle}", (100,380), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 0, 0), 2, cv2.LINE_AA)
        if not self.check_goal(x,y,X,Y):
            self.MOVING=1
            if (self.set_angle<=buffer and self.set_angle>=0) or (self.set_angle<=360 and self.set_angle>=360-buffer):
                if (angle<(buffer+self.set_angle)%360 and angle>=0) or  (angle>(self.set_angle-buffer)%360 and angle<=360):
                    self.last_error = 0
                    self.integral = 0
                    self.control()
                    X,Y,Theta,c_distance=self.get_location(self.pose)
                    self.pose=[X,Y,Theta]
                    
                else:
                    diff=self.set_angle-angle
                    if diff<0:
                        if abs(diff)>180:
                            self.control_angle(abs(360+diff),1)
                        else:
                            self.control_angle(abs(diff),2)
                    elif diff>0:
                        if abs(diff)<180:
                            self.control_angle(abs(diff),1)
                        else:
                            self.control_angle(abs(diff),2)

            elif (self.set_angle>buffer and  self.set_angle<360-buffer):
                if angle<(buffer+self.set_angle)%360 and angle>(self.set_angle-buffer)%360 :
                    self.last_error = 0
                    self.integral = 0
                    self.control()
                    X,Y,Theta,c_distance=self.get_location(self.pose)
                    self.pose=[X,Y,Theta]
                else:
                    diff=self.set_angle-angle
                    if diff<0:
                        if abs(diff)>180:
                            self.control_angle(abs(diff),1)
                        else:
                            self.control_angle(abs(diff),2)
                    elif diff>0:
                        if abs(diff)<180:
                            self.control_angle(abs(diff),1)
                        else:
                            self.control_angle(abs(diff),2)
            
        elif self.REACHED==0:
            if (t<=buffer and t>=0) or (t<=360 and t>=360-buffer):
                if not (angle<(buffer+t)%360 and angle>=0) or  (angle>(t-buffer)%360 and angle<=360):
                    diff=t-angle
                    if diff<0:
                        if abs(diff)>180:
                            self.control_angle(abs(360+diff),1)
                        else:
                            self.control_angle(abs(diff),2)
                    elif diff>0:
                        if abs(diff)<180:
                            self.control_angle(abs(diff),1)
                        else:
                            self.control_angle(abs(diff),2)
                else:
                    self.REACHED=1
            elif (t>buffer and  t<360-buffer):
                if not (angle<(buffer+t)%360 and angle>(t-buffer)%360) :
                    diff=t-angle
                    if diff<0:
                        if abs(diff)>180:
                            self.control_angle(abs(diff),1)
                        else:
                            self.control_angle(abs(diff),2)
                    elif diff>0:
                        if abs(diff)<180:
                            self.control_angle(abs(diff),1)
                        else:
                            self.control_angle(abs(diff),2)
                else:
                    self.REACHED=1
        else:
            print("Reached Goal")
            self.stop()
            time.sleep(1)
            self.MOVING=0
            self.POSITION+=1
            if self.POSITION>=len(loc):
                self.POSITION=0
            self.last_error = 0
            self.integral = 0
            self.SEARCH=1
            self.REACHED=0
            self.PICK=1
            self.EXPLORE=0
        
   
    def travel(self,x,y,img):
        """Travel to a goal location"""
        buffer=3
        X,Y,Theta,c_distance=self.get_location(self.pose)
        self.pose=[X,Y,Theta]
        angle=self.yaw
        distance,g_angle=0,0
        if self.MOVING==0:
            distance,g_angle=self.compute_distance_and_angle(self.pose[:2],(x,y))
            self.set_angle=g_angle%360
            self.MOVING=1
            print(f"To {x,y} from {X,Y} set angle {self.set_angle} distance {distance}")
        if not self.check_goal(x,y,X,Y):
            print()
            self.MOVING=1
            if (self.set_angle<=buffer and self.set_angle>=0) or (self.set_angle<=360 and self.set_angle>=360-buffer):
                if (angle<(buffer+self.set_angle)%360 and angle>=0) or  (angle>(self.set_angle-buffer)%360 and angle<=360):
                    self.last_error = 0
                    self.integral = 0
                    self.control()
                    X,Y,Theta,c_distance=self.get_location(self.pose)
                    self.pose=[X,Y,Theta]
                else:
                    diff=self.set_angle-angle
                    if diff<0:
                        if abs(diff)>180:
                            self.control_angle(abs(360+diff),1)
                        else:
                            self.control_angle(abs(diff),2)
                    elif diff>0:
                        if abs(diff)<180:
                            self.control_angle(abs(diff),1)
                        else:
                            self.control_angle(abs(diff),2)

            elif (self.set_angle>buffer and  self.set_angle<360-buffer):
                self.MOVING=1
                if angle<(buffer+self.set_angle)%360 and angle>(self.set_angle-buffer)%360 :
                    self.last_error = 0
                    self.integral = 0
                    self.control()
                    X,Y,Theta,c_distance=self.get_location(self.pose)
                    self.pose=[X,Y,Theta]
                else:
                    diff=self.set_angle-angle
                    if diff<0:
                        if abs(diff)>180:
                            self.control_angle(abs(diff),1)
                        else:
                            self.control_angle(abs(diff),2)
                    elif diff>0:
                        if abs(diff)<180:
                            self.control_angle(abs(diff),1)
                        else:
                            self.control_angle(abs(diff),2)
            # print(self.pose)
            return 0
        
        else:
            # print(self.pose)
            return 1

    def reverse(self):
        """Function to move backwards"""
        print("Taking Reverse")
        
        
        self.pwm3.ChangeDutyCycle(self.BASE_DUTYCYCLE_R)
        self.pwm4.ChangeDutyCycle(self.BASE_DUTYCYCLE_R)
        self.pwm1.ChangeDutyCycle(0)
        self.pwm2.ChangeDutyCycle(0)

    def pick_block(self,img,lower,upper):
            X,Y,Theta,c_distance=self.get_location(self.pose)
            self.pose=[X,Y,Theta]
            """Function to pick up a block within range"""
        # Convert the image to the HSV color space
            hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            # Create a mask to isolate green pixels
            mask = cv2.inRange(hsv_img, lower, upper)
            # Apply erosion and dilation to remove noise and make corners smooth
            kernel = np.ones((3,3),np.uint8)
            erosion = cv2.erode(mask, kernel, iterations = 1)
            dilation = cv2.dilate(erosion, kernel, iterations = 1)
            # Apply gaussian blur to he mask
            mask_blur = cv2.GaussianBlur(dilation, (3, 3), 0)
            # Find the contours in the binary image
            contours, hierarchy = cv2.findContours(mask_blur, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = sorted(contours, key=cv2.contourArea, reverse=True)[:1]
            status=0
            x,y,w,h=0,0,0,0
            angle=0
            if len(contours)==0:
                status = 0
            else:
                for max_contour in contours:
                    area = cv2.contourArea(max_contour)
                    if area>400.0 and area<17000.0:
                        status = 1
                        x,y,w,h = cv2.boundingRect(max_contour)
                        tolerance=0
                        mask = np.zeros(mask_blur.shape, np.uint8)
                        mask[y:y+h+tolerance, x:x+w+tolerance] = mask_blur[y:y+h+tolerance, x:x+w+tolerance]    
                        cv2.circle(img, (int(x+(w/2)),int(y+(h/2))), 2, (0,0,0), -1)
                        cv2.circle(img, (int(x+(w/2)),int(y+(h/2))), 20, (0,255,255), 1)
                        cv2.rectangle(img,(int(x),int(y)),(int(x+w),int(y+h)),(0,0,0),1)
                        cv2.putText(img,f"Area {area}",(int(x-5),int(y-5)),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (25, 255, 0), 2, cv2.LINE_AA)
                        angle,d=self.get_position(x+(w/2),h)
             
            cv2.line(img, (320, 220), (320, 260), (0, 0, 0), 2)
            cv2.line(img, (300, 240), (340, 240), (0, 0, 0), 2)
            
            cv2.putText(img, str(angle), (100,400), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
            if status==1:
                if self.SERVO_OPEN==0 and self.PICKED==0:
                    self.servo.ChangeDutyCycle(13.0)
                    self.SERVO_OPEN=1
                    time.sleep(1)

                if angle<3 and angle>-3:
                    self.control()
                    X,Y,Theta,c_distance=self.get_location(self.pose)
                    self.pose=[X,Y,Theta]
                    if self.block_in_gripper(x,y,h,w):
                        X,Y,Theta,c_distance=self.get_location(self.pose)
                        self.pose=[X,Y,Theta]
                        self.stop()
                        self.servo.ChangeDutyCycle(7.5)
                        time.sleep(0.5)
                        
                        self.EXPLORE=0
                        self.SERVO_OPEN=0
                        self.PICK=0
                        self.SEARCH=0
                        self.MOVE_TO_ZONE=1
                        self.MOVING=0
                else:
                    if angle>0:
                        self.control_angle(angle,1)
                    else:
                        self.control_angle(angle,2)
            elif status==0:
                self.pwm1.ChangeDutyCycle(35.0)
                self.pwm4.ChangeDutyCycle(35.0)
                self.pwm2.ChangeDutyCycle(0.0)
                self.pwm3.ChangeDutyCycle(0.0)
                

    def main(self):
        """Main Function to Run the Algorithm"""
        self.pwm_setup(self.L_IN,self.R_OUT,self.L_OUT,self.R_IN)
        while True:
            pwm=int(input("Enter PWM: "))
            self.control_angle(pwm,1)
            time.sleep(4)
            self.control_angle(pwm,2)
            time.sleep(4)
            if pwm==0:
                break
            
        

if __name__ == "__main__":
    r=Robot()
    r.main()
    r.gameover()



        






