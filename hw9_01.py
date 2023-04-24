import RPi.GPIO as gpio
import numpy as np
import time
import datetime
import serial
import math
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera





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

        self.location=[]
        self.setup()
        self.yaw=0.0
        self.acc=0.0
        self.ser = serial.Serial('/dev/ttyUSB0', 9600)
        self.count_line=0
        while self.count_line<=10:
            if(self.ser.in_waiting>0):
                self.count_line += 1
                line = self.ser.readline()
        self.fl = open('hw7data.txt','a') 
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

        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = cv2.VideoWriter('hw9_block_tracking.avi', fourcc, 10, (640, 480))

    def get_yaw(self):
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

    def gameover(self):
        self.setup()
        gpio.output(self.L_IN,False)
        gpio.output(self.L_OUT,False)
        gpio.output(self.R_IN,False)
        gpio.output(self.R_OUT,False)
        gpio.cleanup()

    def forward(self):
        self.setup()
        # Left Wheels
        gpio.output(self.L_IN, True)
        gpio.output(self.L_OUT, False)

        # Right Wheels
        gpio.output(self.R_IN, False)
        gpio.output(self.R_OUT, True)



    def pivotleft(self):
        self.setup()
        # Left Wheels
        gpio.output(self.L_IN, False)
        gpio.output(self.L_OUT, True)

        # Right Wheels
        gpio.output(self.R_IN, False)
        gpio.output(self.R_OUT, True)

        

    def reverse(self):
        self.setup()
        # Left Wheels
        gpio.output(self.L_IN, False)
        gpio.output(self.L_OUT, True)

        # Right Wheels
        gpio.output(self.R_IN, True)
        gpio.output(self.R_OUT, False)


        

    def pivotright(self):
        self.setup()
        # Left Wheels
        gpio.output(self.L_IN, True)
        gpio.output(self.L_OUT, False)

        # Right Wheels
        gpio.output(self.R_IN, True)
        gpio.output(self.R_OUT, False)

        
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
        
        return control_variable

    def control_angle(self,angle,direction):
        self.BASE_DUTYCYCLE=0.0
        control_val=self.update(angle)
        print("PWM: ",min(self.BASE_DUTYCYCLE+control_val,50.0))
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
        
        

    def main(self):
        self.pwm_setup(self.L_IN,self.R_OUT,self.L_OUT,self.R_IN)
        # Define variables for frame rate and status display
        status = 0
        # Get the current time in seconds since the epoch
        # Loop through each frame of the video
        for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=False):
            # Start time
            x,w=0,0
            # grab the current frame
            img = frame.array
            img=cv2.flip(img,0)
            img=cv2.flip(img,1)

            # Convert the image to the HSV color space
            hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            # Define the lower and upper bounds of the green color range in HSV
            lower_green = np.array([53, 145, 51])
            upper_green = np.array([179, 255, 247])

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
                status = 0
            else:
                # Update status
                

                # Find the maximum contour
                # max_contour = max(contours, key=cv2.contourArea)
                for max_contour in contours:
                    area = cv2.contourArea(max_contour)
                    cv2.putText(img, str(area), (100,200), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
                    if area>4100.0 and area<4500.0:
                        status = 1
                    # Draw a bounding box around the max contour and mask everything else
                    x,y,w,h = cv2.boundingRect(max_contour)
                    tolerance=0
                    mask = np.zeros(mask_blur.shape, np.uint8)
                    mask[y:y+h+tolerance, x:x+w+tolerance] = mask_blur[y:y+h+tolerance, x:x+w+tolerance]    
                    # cv2.rectangle(img, (x, y), (x+w, y+h), (0,0,255), 2)
                    # X,Y,Z=get_position(x+(w/2),y+(h/2),h)
                    cv2.circle(img, (int(x+(w/2)),int(y+(h/2))), 2, (0,0,0), -1)
                    cv2.circle(img, (int(x+(w/2)),int(y+(h/2))), 20, (0,255,255), 1)
                    # co=str(int(x+(w/2)))+" "+str(int(y+(h/2)))+str(y)+" "+str(h)
                    # cv2.putText(img, str(status), (100,100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
                    
        
        
            cv2.line(img, (320, 220), (320, 260), (0, 0, 0), 2)
            cv2.line(img, (300, 240), (340, 240), (0, 0, 0), 2)

            angle=self.get_position(x+(w/2))
            cv2.putText(img, str(angle), (100,400), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)


            # Display the output
            cv2.imshow("Arrow Image", img)
            self.out.write(img)

            key = cv2.waitKey(1) & 0xFF
            # clear the stream in preparation for the next frame
            self.rawCapture.truncate(0)
            if status==1:
                if angle>0:
                    self.angle_control(angle,1)
                    print("Clockwise")
                else:
                    self.angle_control(angle,2)
                    print("CounterClockwise")
            elif status==0:
                self.pwm1.ChangeDutyCycle(35.0)
                self.pwm4.ChangeDutyCycle(35.0)
                self.pwm2.ChangeDutyCycle(0.0)
                self.pwm3.ChangeDutyCycle(0.0)



            # Writing to file
            # fl.write(outstring)
            # press the 'q' key to stop the video stream
            if key == ord("q"):       
                break
   

    def angle_control(self,angle,direction):
        print("In angle Control")
        self.BASE_DUTYCYCLE=50
        self.BUFFER=0
        self.control_angle(angle,direction)
        
        

if __name__ == "__main__":
    r=Robot()
    r.main()
    r.gameover()


        






