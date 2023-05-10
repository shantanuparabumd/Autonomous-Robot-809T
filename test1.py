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
        self.MOVING=0
        self.POSITION=0
        self.EXPLORE=1
        self.GET=0
        self.REACHED=0
        self.PLAN=0
        self.MOVE_TO_POINT=0
        self.PICK=0
        self.DROP=0
        

        self.drop_count=0
        self.counterBR = np.uint64(0)
        self.counterFL = np.uint64(0)
        self.buttonBR = int(0)
        self.buttonFL = int(0)
        self.turning=0
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

        self.Kp = 1.5
        self.Ki = 0.01
        self.Kd = 0.4
        self.Ki=0
        self.Kd=0
        self.setpoint = 0
        self.last_error = 0
        self.integral = 0

        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = cv2.VideoWriter('hw9_block_retrival.avi', fourcc, 10, (640, 480))

        self.set_angle=0

        #Mapping
        self.scale=1
        self.obstacle_size=4
        self.occupancy_grid=np.ones((305*self.scale, 305*self.scale,3))
        self.occupancy_grid[:,:]=[255,255,255]
        self.result = []
        self.path=None

    def update_occupancy_grid(self,x,y,color):
        xc=int(round(x*100))
        yc=int(round(y*100))
        self.occupancy_grid[xc-self.obstacle_size:xc+self.obstacle_size,yc-self.obstacle_size:yc+self.obstacle_size]=color

    def get_block_pose(self,depth,angle):
        new_depth=depth/math.cos(angle)
        yn=new_depth*math.sin(math.radians(self.pose[2])+angle)/1000
        xn=new_depth*math.cos(math.radians(self.pose[2])+angle)/1000
        xb=xn+(self.pose[0])
        yb=yn+(self.pose[1])
        return xb,yb
    
  

    def show(self):
            # Create an OpenCV image from the matrix
            array = np.uint8(self.occupancy_grid)

            # Display the image using OpenCV
            cv2.imshow('Image', array)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

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
        DISTANCE=(WHEEL_REV*self.DIST_PER_REV)+WHEEL_REV*0.05
        self.get_yaw()
        Theta=math.radians(self.yaw)
        X=X+math.cos(Theta)*DISTANCE
        Y=Y+math.sin(Theta)*DISTANCE
        
        return X,Y,self.yaw,DISTANCE


   

    def get_yaw(self):
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
                        # return yaw
                    
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


        
    def get_position(self,px,height):
        
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
        self.turning=1
        self.BASE_DUTYCYCLE=0.0
        minimum=35
        control_val=self.update(angle)
        print(f"PWM {min(max(self.BASE_DUTYCYCLE+control_val,minimum),100.0)} for Angle {angle}")
        if direction==1:
            self.pwm1.ChangeDutyCycle(min(max(self.BASE_DUTYCYCLE+control_val,minimum),100.0))
            self.pwm4.ChangeDutyCycle(min(max(self.BASE_DUTYCYCLE+control_val,minimum),100.0))
            self.pwm2.ChangeDutyCycle(self.BASE_DUTYCYCLE)
            self.pwm3.ChangeDutyCycle(self.BASE_DUTYCYCLE)
        elif direction==2:
            self.pwm1.ChangeDutyCycle(self.BASE_DUTYCYCLE)
            self.pwm4.ChangeDutyCycle(self.BASE_DUTYCYCLE)
            self.pwm2.ChangeDutyCycle(min(max(self.BASE_DUTYCYCLE+control_val,minimum),100.0))
            self.pwm3.ChangeDutyCycle(min(max(self.BASE_DUTYCYCLE+control_val,minimum),100.0))
        self.counterBR,self.counterFL=0,0

    def update_BR(self,channel):
        if self.turning==0:
            if int(gpio.input(channel)) != int(self.buttonBR):
                self.buttonBR = int(gpio.input(channel))
                self.counterBR+=1
    
    def update_FL(self,channel):
        if self.turning==0:
            if int(gpio.input(channel)) != int(self.buttonFL):
                self.buttonFL = int(gpio.input(channel))
                self.counterFL+=1

    def control(self):
        self.turning=0
        self.counterBR,self.counterFL=0,0
        self.BASE_DUTYCYCLE=50
        self.pwm1.ChangeDutyCycle(self.BASE_DUTYCYCLE)
        self.pwm2.ChangeDutyCycle(self.BASE_DUTYCYCLE)
        self.pwm3.ChangeDutyCycle(0)
        self.pwm4.ChangeDutyCycle(0)

    def gameover(self):
        self.pwm3.ChangeDutyCycle(0)
        self.pwm4.ChangeDutyCycle(0)
        self.pwm1.ChangeDutyCycle(0)
        self.pwm2.ChangeDutyCycle(0)
        gpio.cleanup()
    
    def stop(self):
        self.pwm3.ChangeDutyCycle(0)
        self.pwm4.ChangeDutyCycle(0)
        self.pwm1.ChangeDutyCycle(0)
        self.pwm2.ChangeDutyCycle(0)
    

    def block_in_gripper(self,x,y,h,w):
        if (x+w/2)>275 and (x+w/2)<360 and (y+h/2)>440:
            return True
        else:
            return False
            
    def send_mail(self):
        smtpUser = 'sparab@umd.edu'
        smtpPass = 'oeonpzethtnqlxxt'

        toAdd = 'ENPM809TS19@gmail.com'
        # toAdd = 'shantanuparab99@gmail.com'
        fromAdd = smtpUser
        subject = 'Homework 9 Part 2 Submission'
        msg = MIMEMultipart()
        msg['Subject'] = subject
        msg['From'] = fromAdd
        msg['To'] = toAdd
        msg['Cc'] = 'rpatil10@umd.edu'
        msg.preamble = "Homework Submission"

        body=MIMEText("This mail servers as a part of homework 9 submission.\n PFA the image recorded after block retrival")
        msg.attach(body)

        fp=open('hw9.jpg','rb')

        img = MIMEImage(fp.read())
        fp.close()
        msg.attach(img)

        s=smtplib.SMTP('smtp.gmail.com',587)
        s.ehlo()
        s.starttls()
        s.ehlo()

        s.login(smtpUser,smtpPass)
        s.sendmail(fromAdd,toAdd,msg.as_string())
        s.quit()
        print("Email Delivered")

    def compute_distance_and_angle(self,current, goal):
        # Extract x and y coordinates for both points
        x1, y1 = current
        x2, y2 = goal

        # Compute distance between the points using Pythagoras theorem
        distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

        # Compute angle between the points using trigonometry
        angle = math.atan2(y2 - y1, x2 - x1) * 180 / math.pi
        angle=angle%360
        return distance, angle
    
    def check_validity(self,x,y,bx,by):
        distance = math.sqrt((bx - x)**2 + (by - y)**2)
        if distance <= 1.0:
            return True
        else:
            return False
        
    
    def check_goal(self,x,y,cx,cy):
        distance = math.sqrt((cx - x)**2 + (cy - y)**2)
        if distance <= 0.2:
            return True
        else:
            return False
        
    def get_blocks(self,img):
            blocks=[]
            hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                    
            colors = {
            'red': ([147, 126, 100], [179,255,220]),
            'green': ([75,67,76],[102,229,229]),
            'blue': ([86,72,122], [122,226,255]),
            'yellow': ([17,85,131], [38,248,230]),
            'black': ([100,35,15], [155,255,95]),
            'white': ([0,0,203], [151,33,255])
            }

            # Loop over the colors and find the contours of the corresponding color
            for color, (lower_range, upper_range) in colors.items():
                # Threshold the image to get a binary mask of the color range
                mask = cv2.inRange(hsv_img, np.array(lower_range), np.array(upper_range))

                # Apply erosion and dilation to remove noise and make corners smooth
                kernel = np.ones((3,3),np.uint8)

                erosion = cv2.erode(mask, kernel, iterations = 1)
                dilation = cv2.dilate(erosion, kernel, iterations = 1)

                # Apply gaussian blur to he mask
                mask_blur = cv2.GaussianBlur(dilation, (3, 3), 0)

                # Find the contours in the binary mask
                contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                contours = sorted(contours, key=cv2.contourArea, reverse=True)[:3]
                # Find the centers of the contours and add them to the corresponding list
                for cnt in contours:
                    area = cv2.contourArea(cnt)
            #         cv2.imshow('image', mask)
            #         cv2.waitKey(0)
            #         cv2.destroyAllWindows()
            #         print(f"Color {color} Area {area}")
                    if area>450.0 and area<7000.0:
                        x,y,w,h = cv2.boundingRect(cnt)
                        
                        
                        if w<90 and w>20 and h<200 and h>12:
                            print(f"Color {color} {x,y,w,h}")
                            tolerance=0
                            mask = np.zeros(mask_blur.shape, np.uint8)
                            mask[y:y+h+tolerance, x:x+w+tolerance] = mask_blur[y:y+h+tolerance, x:x+w+tolerance]    

                            # # Draw a circle around the contour with its respective color
                            if color == 'red':
                                cv2.rectangle(img, (x,y),(x+w,y+h), (0,0,255), 1)
                                blocks.append([x,y,w,h,[0,0,255]])
                            elif color == 'green':
                                cv2.rectangle(img, (x,y),(x+w,y+h), (0,255,0), 1)
                                blocks.append([x,y,w,h,[0,255,0]])
                            elif color == 'blue':
                                cv2.rectangle(img, (x,y),(x+w,y+h), (255,0,0), 1)
                                blocks.append([x,y,w,h,[255,0,0]])
                            elif color == 'yellow':
                                cv2.rectangle(img, (x,y),(x+w,y+h), (0,255,255), 1)
                                blocks.append([x,y,w,h,[0,255,255]])
                            elif color == 'black':
                                cv2.rectangle(img, (x,y),(x+w,y+h), (0,0,0), 1)
                                blocks.append([x,y,w,h,[0,0,0]])
                            elif color == 'white':
                                cv2.rectangle(img, (x,y),(x+w,y+h), (255,255,255), 1)
                                blocks.append([x,y,w,h,[255,255,200]])
            return blocks
                        
    def update_map(self,blocks,img):
        for block in blocks:
                x,y,w,h,color=block
                X,Y,T,d=self.get_location(self.pose)
                self.pose=[X,Y,T]
                angle,depth=self.get_position(x+(w/2),h)
                cv2.putText(img,f" Depth: {round(depth,2)}", (x,y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.25, (255, 0, 0), 1, cv2.LINE_AA)
                cv2.putText(img,f" Angle: {round(angle,2)}", (x,y-25), cv2.FONT_HERSHEY_SIMPLEX, 0.25, (255, 0, 0), 1, cv2.LINE_AA)
                xb,yb=self.get_block_pose(depth,math.radians(angle))

                cv2.putText(img,f" x: {round(xb,2)} y {round(yb,2)} ", (x,y+h+10), cv2.FONT_HERSHEY_SIMPLEX, 0.25, (0, 255, 0), 1, cv2.LINE_AA)
                cv2.putText(img,f" X: {round(X,2)} Y {round(Y,2)} ", (x,y+h+25), cv2.FONT_HERSHEY_SIMPLEX, 0.25, (0, 0, 255), 1, cv2.LINE_AA)
                
                if self.check_validity(X,Y,xb,yb):
                    cv2.putText(img,f"Valid ", (int(x+w/2),int(y+h/2)), cv2.FONT_HERSHEY_SIMPLEX, 0.25, (0, 255, 0), 1, cv2.LINE_AA)
                    self.update_occupancy_grid(xb,yb,color)
                    self.update_occupancy_grid(X,Y,[0,0,0])
        # self.show()

    def explore(self,img):
        buffer=3
        X,Y,Theta,c_distance=self.get_location(self.pose)
        self.pose=[X,Y,Theta]
        angle=self.yaw
        
        # loc=[[0.25,0.25,90],[0.25,0.0,180],[0.8,0.8,10],[0.8,0.0,60],[1.2,0.4,45],[0.5,0.8,25]]
        # loc=[[0.0,0.0,45],[1.0,0.0,135],[1,0.5,180],[0.8,0.8,200],[0.5,0.8,270]]
        loc=[[0.0,0.0,45],[1.5,0.0,135],[1.5,1.0,225],[0.0,1.0,315],[0.0,0.0,0]]
        if self.POSITION<5:
            x,y,t=loc[self.POSITION]
            if self.MOVING==0:
                distance,g_angle=self.compute_distance_and_angle(self.pose[:2],(x,y))
                self.set_angle=g_angle

            cv2.putText(img,f"X: {x} Y: {y} T: {self.set_angle}", (100,380), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 0, 0), 2, cv2.LINE_AA)
            if not self.check_goal(x,y,X,Y):
                self.MOVING=1
                add=False
                cv2.putText(img,f"Acceptable Angle 1 {(self.set_angle+3)%360} - {(self.set_angle-3)%360}", (100,400), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 0, 0), 2, cv2.LINE_AA)
                if (self.set_angle<=buffer and self.set_angle>=0) or (self.set_angle<=360 and self.set_angle>=360-buffer):
                    if (angle<(buffer+self.set_angle)%360 and angle>=0) or  (angle>(self.set_angle-buffer)%360 and angle<=360):
                        self.last_error = 0
                        self.integral = 0
                        self.control()
                        X,Y,Theta,c_distance=self.get_location(self.pose)
                        
                        self.pose=[X,Y,Theta]
                        display_status="Moving towards block"
                        # print("moving Forward")
                    else:
                        diff=self.set_angle-angle
                        # print(F"{self.set_angle} {angle} {diff}")
                        if diff<0:
                            if abs(diff)>180:
                                # print(f"{abs(360+diff)} Clockwise")
                                self.control_angle(abs(360+diff),1)
                            else:
                                # print(f"{abs(diff)} Anti Clockwise")
                                self.control_angle(abs(diff),2)
                        elif diff>0:
                            if abs(diff)<180:
                                # print(f"{abs(diff)} Clockwise")
                                self.control_angle(abs(diff),1)
                            else:
                                # print(f"{abs(diff)} Anti Clockwise")
                                self.control_angle(abs(diff),2)

                elif (self.set_angle>buffer and  self.set_angle<360-buffer):
                    self.MOVING=1
                    if angle<(buffer+self.set_angle)%360 and angle>(self.set_angle-buffer)%360 :
                        self.last_error = 0
                        self.integral = 0
                        self.control()
                        X,Y,Theta,c_distance=self.get_location(self.pose)
                        
                        self.pose=[X,Y,Theta]
                        display_status="Moving towards block"
                        # print("moving Forward")
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
                        cv2.putText(img,f"Acceptable Angle 2 {round((angle+3)%360,2)} - {round((angle-3)%360,)}", (100,400), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 0, 0), 2, cv2.LINE_AA)
                        # print("Correcting angle")
                        self.MOVING=1
                        diff=t-angle
                        # print(F"{t} {angle} {diff}")
                        if diff<0:
                            if abs(diff)>180:
                                # print(f"{abs(360+diff)} Clockwise")
                                self.control_angle(abs(360+diff),1)
                            else:
                                # print(f"{abs(diff)} Anti Clockwise")
                                self.control_angle(abs(diff),2)
                        elif diff>0:
                            if abs(diff)<180:
                                # print(f"{abs(diff)} Clockwise")
                                self.control_angle(abs(diff),1)
                            else:
                                # print(f"{abs(diff)} Anti Clockwise")
                                self.control_angle(abs(diff),2)
                    else:
                        self.REACHED=1
                elif (t>buffer and  t<360-buffer):
                    self.MOVING=1
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
                self.last_error = 0
                self.integral = 0
                self.GET=1
                self.REACHED=0
        else:
            self.EXPLORE=0
        
        # Planning
    def check_goal2(self,c,g):
        distance = math.sqrt((g[0] - c[0])**2 + (g[1] - c[1])**2)
        if distance <= 10:
            return True
        else:
            return False
        
    def heuristic(self,a, b):
        """Returns the Euclidean distance between two points"""
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def weighted_astar(self,start, goal, img, w=2):
        """Returns the path from start to goal using the Weighted A* algorithm"""
        # Check if new point is within bounds of image
        
            
        # Define action set
        actions = [(10, 0), (10, 30), (10, -30), (10, 60), (10, -60), (10, 90), (10, -90),(10, 180),(10, -120),(10, 120),(10, -150),(10, 150)]
        
        # Define colors for obstacles
        black = np.array([0, 0, 0])
        yellow = np.array([0, 255, 255])
        
        if goal[0] < 0 or goal[0] >= img.shape[1] or goal[1] < 0 or goal[1] >= img.shape[0]:
            print("Goal Out of Bounds")

        # Check if new point is a valid point
        color = img[goal[1], goal[0]]
        if (color == black).all() or (color == yellow).all():
            print(f"Color {color} Invalid")
        # Create priority queue
        frontier = PriorityQueue()
        frontier.put(start, 0)
        
        # Initialize costs
        cost = {}
        cost[start] = 0
        
        # Initialize parents
        parent = {}
        parent[start] = None
        
        # Initialize actions
        act = {}
        act[start] = None
        
        # Loop until goal is found or queue is empty
        while not frontier.empty():
            # Get the current node
            current = frontier.get()
            
            # Check if goal is found
            if self.check_goal2(current,goal):
                break
            
            # Loop through each action
            for action in actions:
                # Calculate the new point
                x, y = current[0], current[1]
                dx = action[0] * math.cos(math.radians(action[1]))
                dy = action[0] * math.sin(math.radians(action[1]))
                next_point = (int(x + dx), int(y + dy))
                
                # Check if new point is within bounds of image
                if next_point[0] < 0 or next_point[0] >= img.shape[1] or next_point[1] < 0 or next_point[1] >= img.shape[0]:
                    continue
                
                # Check if new point is a valid point
                color = img[next_point[1], next_point[0]]
                if (color == black).all() or (color == yellow).all():
                    continue
                
                # Calculate the new cost
                new_cost = cost[current] + action[0] * w
                
                # Check if new cost is less than existing cost
                if next_point not in cost or new_cost < cost[next_point]:
                    cost[next_point] = new_cost
                    priority = new_cost + self.heuristic(goal, next_point)
                    frontier.put(next_point, priority)
                    parent[next_point] = current
                    act[next_point]=action
            
        print(current)
        if self.check_goal2(current,goal):
            print("Goal Found")
        
            # Construct the path by backtracking from goal to start
            path = []
            act_set=[]
    #         current = goal
            while current != start:
                path.append(current)
                current = parent[current]
                a=act[current]
                act_set.append(a)
            act_set.reverse()
            path.append(start)
            path.reverse()

            return path,act_set
        else:
            print("Goal Not Found")
            return None,None

    def draw_path(self,img, path, color=(0, 255, 0), thickness=2):
        """Draws the given path on the input image"""
        for i in range(len(path) - 1):
            print(path[i],path[i+1])
            cv2.line(img, path[i], path[i+1], color, thickness)
        return img
    
    def detect_obstacles(self):
        # Define the colors to detect and their respective color ranges
        img=np.uint8(self.occupancy_grid)
        # cv2.imshow("Blocks", img)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        colors = {
            'red': ([0, 255, 255], [25, 255, 255]),
            'green': ([34, 255, 255], [67, 255, 255]),
            'blue': ([116, 255, 255], [123, 255, 255]),
            'yellow': ([26, 255, 255], [30, 255, 255]),
            'black': ([0, 0, 0], [1, 1, 1])
        }
        # Initialize lists to store the centers and colors of the detected contours
        red_centers = []
        green_centers = []
        blue_centers = []

        # Loop over the colors and find the contours of the corresponding color
        for color, (lower_range, upper_range) in colors.items():
            # Threshold the image to get a binary mask of the color range
            mask = cv2.inRange(hsv_img, np.array(lower_range), np.array(upper_range))

            # Find the contours in the binary mask
            contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Find the centers of the contours and add them to the corresponding list
            for cnt in contours:
                M = cv2.moments(cnt)
                if M['m00'] == 0:
                    continue
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                if color == 'red':
                    red_centers.append((cx, cy, color))
                elif color == 'green':
                    green_centers.append((cx, cy, color))
                elif color == 'blue':
                    blue_centers.append((cx, cy, color))

                # # Draw a circle around the contour with its respective color
                if color == 'red':
                    cv2.circle(img, (cx, cy), 10, (0, 0, 255), -1)
                elif color == 'green':
                    cv2.circle(img, (cx, cy), 10, (0, 255, 0), -1)
                elif color == 'blue':
                    cv2.circle(img, (cx, cy), 10, (255, 0, 0), -1)
                elif color == 'yellow':
                    cv2.circle(img, (cx, cy), 20, (0, 255, 255), -1)
                elif color == 'black':
                    cv2.circle(img, (cx, cy), 20, (0, 0, 0), -1)
        # cv2.imshow("Blocks", img)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
            

        # Order the list of centers as red, green, blue, red, green, blue, ...
        centers = []
        red_centers = sorted(red_centers, key=self.distance_from_origin)
        blue_centers = sorted(blue_centers, key=self.distance_from_origin)
        green_centers = sorted(green_centers, key=self.distance_from_origin)
        rc,bc,gc=red_centers.copy(),blue_centers.copy(),green_centers.copy()
        while True:
            if red_centers:
                centers.append(red_centers.pop(0))
            if green_centers:
                centers.append(green_centers.pop(0))
            if blue_centers:
                centers.append(blue_centers.pop(0))
            if not (red_centers or green_centers or blue_centers):
                break

        return centers,rc,bc,gc
    
    def distance_from_origin(self,point):
        """Returns the Euclidean distance between a point and the origin (0, 0)"""
        x, y, _ = point
        return math.sqrt(x**2 + y**2)
    
    def travel(self,x,y,img):
        x,y=x,y
        buffer=3
        X,Y,Theta,c_distance=self.get_location(self.pose)
        self.pose=[X,Y,Theta]
        angle=self.yaw

        if self.MOVING==0:
            distance,g_angle=self.compute_distance_and_angle(self.pose[:2],(x,y))
            self.set_angle=g_angle
            cv2.putText(img,f"Angle {round(self.set_angle,2)}",(400,25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (10,255,40), 1, cv2.LINE_AA)

        cv2.putText(img,f"X: {x} Y: {y} T: {self.set_angle}", (100,380), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 0, 0), 2, cv2.LINE_AA)
        if not self.check_goal(x,y,X,Y):
            self.MOVING=1
            cv2.putText(img,f"Acceptable Angle 1 {(self.set_angle+3)%360} - {(self.set_angle-3)%360}", (100,400), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 0, 0), 2, cv2.LINE_AA)
            if (self.set_angle<=buffer and self.set_angle>=0) or (self.set_angle<=360 and self.set_angle>=360-buffer):
                if (angle<(buffer+self.set_angle)%360 and angle>=0) or  (angle>(self.set_angle-buffer)%360 and angle<=360):
                    self.last_error = 0
                    self.integral = 0
                    self.control()
                    X,Y,Theta,c_distance=self.get_location(self.pose)
                    
                    self.pose=[X,Y,Theta]
                    display_status="Moving towards block"
                    # print("moving Forward")
                else:
                    diff=self.set_angle-angle
                    # print(F"{self.set_angle} {angle} {diff}")
                    if diff<0:
                        if abs(diff)>180:
                            # print(f"{abs(360+diff)} Clockwise")
                            self.control_angle(abs(360+diff),1)
                        else:
                            # print(f"{abs(diff)} Anti Clockwise")
                            self.control_angle(abs(diff),2)
                    elif diff>0:
                        if abs(diff)<180:
                            # print(f"{abs(diff)} Clockwise")
                            self.control_angle(abs(diff),1)
                        else:
                            # print(f"{abs(diff)} Anti Clockwise")
                            self.control_angle(abs(diff),2)

            elif (self.set_angle>buffer and  self.set_angle<360-buffer):
                self.MOVING=1
                if angle<(buffer+self.set_angle)%360 and angle>(self.set_angle-buffer)%360 :
                    self.last_error = 0
                    self.integral = 0
                    self.control()
                    X,Y,Theta,c_distance=self.get_location(self.pose)
                    
                    self.pose=[X,Y,Theta]
                    display_status="Moving towards block"
                    # print("moving Forward")
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
            return 0
        
        else:
            return 1

    def reverse(self):
        self.BASE_DUTYCYCLE=40
        
        self.pwm3.ChangeDutyCycle(self.BASE_DUTYCYCLE)
        self.pwm4.ChangeDutyCycle(self.BASE_DUTYCYCLE)
        self.pwm1.ChangeDutyCycle(0)
        self.pwm2.ChangeDutyCycle(0)

    def pick_block(self,img,lower,upper):
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
                    if area>400.0 and area<7000.0:
                        status = 1
                    x,y,w,h = cv2.boundingRect(max_contour)
                    tolerance=0
                    mask = np.zeros(mask_blur.shape, np.uint8)
                    mask[y:y+h+tolerance, x:x+w+tolerance] = mask_blur[y:y+h+tolerance, x:x+w+tolerance]    
                    cv2.circle(img, (int(x+(w/2)),int(y+(h/2))), 2, (0,0,0), -1)
                    cv2.circle(img, (int(x+(w/2)),int(y+(h/2))), 20, (0,255,255), 1)
                    angle,d=self.get_position(x+(w/2),h)
                    print(f"Area {area} Depth {d}")
                    # if d>100:
                    #     status=0
            cv2.line(img, (320, 220), (320, 260), (0, 0, 0), 2)
            cv2.line(img, (300, 240), (340, 240), (0, 0, 0), 2)
            
            cv2.putText(img, str(angle), (100,400), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)

            if self.PICK==1:
                # self.control()
                if status==1:
                    if self.OPEN==0 and self.PICKED==0:
                        self.servo.ChangeDutyCycle(13.0)
                        self.OPEN=1
                        time.sleep(1)

                    if angle<3 and angle>-3:
                        self.control()
                        display_status="Moving towards block"
                        # print("moving Forward")
                        if self.block_in_gripper(x,y,h,w):
                            self.servo.ChangeDutyCycle(7.5)
                            time.sleep(0.1)
                            self.OPEN=0
                            self.PICK=0
                            self.PLAN=1
                            self.MOVING=0
                            self.MOVE_TO_POINT=0
                            display_status="Block is Picked"
                    else:
                        if angle>0:
                            self.control_angle(angle,1)
                            print(f" Angel {angle} Clockwise")
                        else:
                            self.control_angle(angle,2)
                            print(f"Angel {angle} CounterClockwise")
                elif status==0:
                    display_status="Searching for Block"
                    # print("Searching Block")
                    self.pwm1.ChangeDutyCycle(35.0)
                    self.pwm4.ChangeDutyCycle(35.0)
                    self.pwm2.ChangeDutyCycle(0.0)
                    self.pwm3.ChangeDutyCycle(0.0)
                

    def main(self):
        
        self.pwm_setup(self.L_IN,self.R_OUT,self.L_OUT,self.R_IN)
        # Define variables for frame rate and status display
        status = 0
        display_status="Searching for Block"
        complete=0
        k=0
        # self.get_count()
        # Get the current time in seconds since the epoch
        # Loop through each frame of the video

        block=0
        px,py=5,5
        point=0
        int_point=0
        block_of_color=None
        for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=False):
            # self.get_count()
            # Start time
            # grab the current frame
            img = frame.array
            img=cv2.flip(img,0)
            img=cv2.flip(img,1)
            
            if self.GET==1:
                blocks=self.get_blocks(img)
                self.update_map(blocks,img)
                centers,rc,bc,gc=self.detect_obstacles()
                # print(f"Blocks detected {len(rc)}")
                if len(rc)>=1 or len(bc)>=3 or len(gc)>3:
                    display_status="Exploration Complete"
                    self.EXPLORE=0
                    self.PLAN=1
                    print("Exploration Complete")
                # cv2.imshow("Blocks", img)
                # cv2.waitKey(0)
                # cv2.destroyAllWindows()
                self.show()
                self.GET=0
                        
            cv2.line(img, (320, 220), (320, 260), (0, 0, 0), 2)
            cv2.line(img, (300, 240), (340, 240), (0, 0, 0), 2)
            if self.EXPLORE==1:
                display_status="Exploring"
                self.explore(img)
            
            else:
                self.result = []
                for i, elem in enumerate(centers):
                    self.result.append(elem)

                    self.result.append((5, 5, 'black'))

            if self.PLAN==1:
                display_status="Planning Path"
                print("Plannning Path")
                x,y,color=self.result[point]
                rx,ry,_,_=self.get_location(self.pose)
                rx,ry=int(rx*100),int(ry*100)
                point+=1
                self.path,a=self.weighted_astar((rx,ry),(x,y),self.occupancy_grid,2)
                colors = {
                    'blue': (255, 0, 0),
                    'green': (0, 255, 0),
                    'red': (0, 0, 255),
                    'black': (0, 20, 0)
                }
                print(f"From {rx,ry} to {x,y} block of color {color} the path is as follows")
                block_of_color=color
                print(self.path)
                img_path=self.draw_path(np.uint8(self.occupancy_grid),self.path,colors[color],2)
                # cv2.imshow("Blocks", img_path)
                # cv2.waitKey(0)
                # cv2.destroyAllWindows()
                self.PLAN=0
                self.MOVE_TO_POINT=1
                if color=='black':
                    self.PICK=0
                    self.DROP=1
                else:
                    self.PICK=1
                    self.DROP=0
            
            if self.MOVE_TO_POINT==1:
                mx,my=self.path[int_point]
                cv2.putText(img,f"Moving to {round(mx/100,2)},{round(my/100,2)}",(400,15),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (10,255,40), 1, cv2.LINE_AA)
                if self.travel(mx/100,my/100,img)==1:
                    int_point+=1
                if int_point>=len(self.path):
                    self.path=None
                    int_point=0
                    self.PLAN=0
                    self.MOVE_TO_POINT=0

            if self.MOVE_TO_POINT==0 and self.PLAN==0:
                colors = {
                    'red': ([147, 126, 100], [179,255,220]),
                    'green': ([75,67,76],[102,229,229]),
                    'blue': ([86,72,122], [122,226,255]),
                    'yellow': ([17,85,131], [38,248,230]),
                    'black': ([100,35,15], [155,255,95]),
                    'white': ([0,0,203], [151,33,255])
                    }
                if self.PICK==1:
                    # Define the lower and upper bounds of the green color range in HSV
                    lower = np.array(colors[block_of_color][0])
                    upper = np.array(colors[block_of_color][1])
                    print(f"Picking {block_of_color} blcok")
                    self.pick_block(img,lower,upper)
                # Start Searching for block and pick
                if self.DROP==1:
                    self.servo.ChangeDutyCycle(13.0)
                    time.sleep(0.3)
                    self.reverse()
                    time.sleep(0.2)
                    display_status="Block Placed"
                    self.PLAN=1
                    self.DROP=0
                    self.drop_count+=1

                # Drop the Object

             
            X,Y,Theta,c_distance=self.get_location(self.pose)
            cv2.rectangle(img, (0,0),(200,20), (255,255,255), 2)
            cv2.putText(img,f"Pose {round(X,2)},{round(Y,2)},{round(Theta,2)}",(5,15),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (10,255,40), 1, cv2.LINE_AA)
            cv2.putText(img,display_status,(5,470),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,200), 1, cv2.LINE_AA)
            # Display the output
            cv2.imshow("Arrow Image", img)
            if complete==1:
                # cv2.imwrite("hw9.jpg",img)
                # self.send_mail()
                self.show()
                break
            
            
            self.out.write(img)

            key = cv2.waitKey(1) & 0xFF
            self.rawCapture.truncate(0)
            if key == ord("q"):      
                break
            if key == ord("w"):      
                self.show()
            if key == ord("f"):      
                self.MOVING=0
            
   
        
        


if __name__ == "__main__":
    r=Robot()
    r.main()
    r.gameover()



        






