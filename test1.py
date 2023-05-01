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

        self.Kp = 5
        self.Ki = 0.001
        self.Kd = 0.02
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
    
    def update_occupancy_grid(self,x,y,color):
        xc=int(round(x*100))
        yc=int(round(y*100))
        self.occupancy_grid[xc-self.obstacle_size:xc+self.obstacle_size,yc-self.obstacle_size:yc+self.obstacle_size]=color

    def get_block_pose(self,depth,angle):
        new_depth=depth/math.cos(angle)
        yn=new_depth*math.sin(math.radians(self.pose[2])+angle)
        xn=new_depth*math.cos(math.radians(self.pose[2])+angle)
        xb=xn+self.pose[0]
        yb=yn+self.pose[1]
        return xb/1000,yb/1000
    
  

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
        control_val=self.update(angle)
        print(control_val)
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
        KP=5
        self.counterBR,self.counterFL=0,0
        self.BASE_DUTYCYCLE=60
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
    
    def check_goal(self,x,y,cx,cy):
        distance = math.sqrt((cx - x)**2 + (cy - y)**2)
        if distance <= 0.1:
            return True
        else:
            return False
        
    def get_blocks(self,img):
            blocks=[]
            # Convert the image to the HSV color space
            hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            # Define the lower and upper bounds of the green color range in HSV
            lower_red = np.array([155, 165, 79])
            upper_red = np.array([177, 255, 237])

            # Create a mask to isolate green pixels
            mask = cv2.inRange(hsv_img, lower_red, upper_red)

            # Apply erosion and dilation to remove noise and make corners smooth
            kernel = np.ones((3,3),np.uint8)
            erosion = cv2.erode(mask, kernel, iterations = 1)
            dilation = cv2.dilate(erosion, kernel, iterations = 1)

            # Apply gaussian blur to he mask
            mask_blur = cv2.GaussianBlur(dilation, (3, 3), 0)

            # Find the contours in the binary image
            contours, hierarchy = cv2.findContours(mask_blur, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = sorted(contours, key=cv2.contourArea, reverse=True)[:3]
            if len(contours)>0:
                for max_contour in contours:
                    area = cv2.contourArea(max_contour)
                    if area>450.0 and area<3100.0:
                        x,y,w,h = cv2.boundingRect(max_contour)
                        tolerance=0
                        mask = np.zeros(mask_blur.shape, np.uint8)
                        mask[y:y+h+tolerance, x:x+w+tolerance] = mask_blur[y:y+h+tolerance, x:x+w+tolerance]    
                        cv2.rectangle(img, (x,y),(x+w,y+h), (255,0,0), 1)
                        blocks.append([x,y,w,h])
            return blocks
                        
    def update_map(self,blocks,img):
        for block in blocks:
                x,y,w,h=block
                X,Y,T,d=self.get_location(self.pose)
                self.pose=[X,Y,T]
                angle,depth=self.get_position(x+(w/2),h)
                cv2.putText(img,f" Depth: {round(depth,2)}", (x,y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.25, (255, 0, 0), 1, cv2.LINE_AA)
                cv2.putText(img,f" Angle: {round(angle,2)}", (x,y-25), cv2.FONT_HERSHEY_SIMPLEX, 0.25, (255, 0, 0), 1, cv2.LINE_AA)
                
                xb,yb=self.get_block_pose(depth,math.radians(angle))
                cv2.putText(img,f" x: {round(xb,2)} y {round(yb,2)} ", (x,y+h+10), cv2.FONT_HERSHEY_SIMPLEX, 0.25, (0, 255, 0), 1, cv2.LINE_AA)
                self.update_occupancy_grid(xb,yb,[0,0,255])
                self.update_occupancy_grid(X,Y,[0,0,0])
        # self.show()

    def explore(self,img):
        buffer=3
        X,Y,Theta,c_distance=self.get_location(self.pose)
        self.pose=[X,Y,Theta]
        angle=self.yaw
        
        # loc=[[0.25,0.25,90],[0.25,0.0,180],[0.8,0.8,10],[0.8,0.0,60],[1.2,0.4,45],[0.5,0.8,25]]
        loc=[[0.0,0.0,45],[1.0,0.0,135],[1,0.5,180],[0.8,0.8,200],[0.5,0.8,270]]
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
                        print("moving Forward")
                    else:
                        diff=self.set_angle-angle
                        print(F"{self.set_angle} {angle} {diff}")
                        if diff<0:
                            if abs(diff)>180:
                                print(f"{abs(360+diff)} Clockwise")
                                self.control_angle(abs(360+diff),1)
                            else:
                                print(f"{abs(diff)} Anti Clockwise")
                                self.control_angle(abs(diff),2)
                        elif diff>0:
                            if abs(diff)<180:
                                print(f"{abs(diff)} Clockwise")
                                self.control_angle(abs(diff),1)
                            else:
                                print(f"{abs(diff)} Anti Clockwise")
                                self.control_angle(abs(diff),2)

                elif (self.set_angle>buffer and  self.set_angle<360-buffer):
                    if angle<(buffer+self.set_angle)%360 and angle>(self.set_angle-buffer)%360 :
                        self.last_error = 0
                        self.integral = 0
                        self.control()
                        X,Y,Theta,c_distance=self.get_location(self.pose)
                        
                        self.pose=[X,Y,Theta]
                        display_status="Moving towards block"
                        print("moving Forward")
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
               
            elif not (angle<(buffer+t)%360 and angle>(t-buffer)%360):
                cv2.putText(img,f"Acceptable Angle 2 {round((angle+3)%360,2)} - {round((angle-3)%360,)}", (100,400), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 0, 0), 2, cv2.LINE_AA)
                print("Correcting angle")
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
                print("Reached Goal")
                self.stop()
                time.sleep(1)
                self.MOVING=0
                self.POSITION+=1
                self.last_error = 0
                self.integral = 0
        else:
            self.EXPLORE=0
        
       
                
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

        for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=False):
            # self.get_count()
            # Start time
            # grab the current frame
            img = frame.array
            img=cv2.flip(img,0)
            img=cv2.flip(img,1)
            
            if self.MOVING==0:
                blocks=self.get_blocks(img)
                self.update_map(blocks,img)
                # self.MOVING=1
                        
            cv2.line(img, (320, 220), (320, 260), (0, 0, 0), 2)
            cv2.line(img, (300, 240), (340, 240), (0, 0, 0), 2)
            if self.EXPLORE==1:
                self.explore(img)
            else:
                print("Exploration Complete")
                complete=1
             
            X,Y,Theta,c_distance=self.get_location(self.pose)
            cv2.putText(img,f"Robot Pose {round(X,2)},{round(Y,2)},{round(Theta,2)}",(50,50),cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 250, 0), 2, cv2.LINE_AA)
            # cv2.putText(img, display_status, (100,100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 250, 0), 2, cv2.LINE_AA)
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



        






