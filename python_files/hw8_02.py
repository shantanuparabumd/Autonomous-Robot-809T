import RPi.GPIO as gpio
import numpy as np
import time
import datetime
import serial
import math





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
        

    def pwm_setup(self, A, B):

        self.pwm1=gpio.PWM(A,50)
        self.pwm1.start(0)
        self.pwm2=gpio.PWM(B,50)
        self.pwm2.start(0)
        
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

        

    def control(self,DISTANCE):
        WHEEL_REV=DISTANCE/self.DIST_PER_REV
        MOTOR_REV=WHEEL_REV*self.GEAR_RATIO
        TOTAL_TICKS=(self.TICKS_PER_MREV*MOTOR_REV)
        TOTAL_TICKS=int(TOTAL_TICKS-MOTOR_REV*self.BUFFER)
        KP=5
        while self.counterBR<=TOTAL_TICKS or self.counterFL<TOTAL_TICKS:
            # ang_val=self.get_yaw()
            # print("In Control: ",ang_val)
            # print("countBR: ", self.counterBR, "counterFL: ", self.counterFL, "BR state: ", gpio.input(12), "FL state: ", gpio.input(7))
            if int(gpio.input(self.EN_RIGHT)) != int(self.buttonBR):
                self.buttonBR = int(gpio.input(self.EN_RIGHT))
                self.counterBR+=1

            if int(gpio.input(self.EN_LEFT)) != int(self.buttonFL):
                self.buttonFL = int(gpio.input(self.EN_LEFT))
                self.counterFL+=1
            # print("Value: ",val)
            # val=int(input("Enter PWM value: "))
            self.get_yaw()
            error=self.TRUE_ANGLE-self.yaw
            if self.TRUE_ANGLE==0:
                if self.yaw>180:
                    error=self.yaw-self.TRUE_ANGLE
                else:
                    error=self.TRUE_ANGLE-self.yaw
            print("Error: ",error)
            if error>0:
                # print("Duty Cycle: Left: ",min(self.BASE_DUTYCYCLE+error*KP,100)," Right: ",self.BASE_DUTYCYCLE)
                self.pwm1.ChangeDutyCycle(min(self.BASE_DUTYCYCLE+error*KP,100))
                self.pwm2.ChangeDutyCycle(self.BASE_DUTYCYCLE)
            elif error<0:
                # print("Duty Cycle: Left: ",self.BASE_DUTYCYCLE," Right: ",min(self.BASE_DUTYCYCLE-error*KP,100))
                self.pwm2.ChangeDutyCycle(min(self.BASE_DUTYCYCLE-error*KP,100))
                self.pwm1.ChangeDutyCycle(self.BASE_DUTYCYCLE)
            else:
                # print("Duty Cycle: Left: ",self.BASE_DUTYCYCLE," Right: ",self.BASE_DUTYCYCLE)
                self.pwm2.ChangeDutyCycle(self.BASE_DUTYCYCLE)
                self.pwm1.ChangeDutyCycle(self.BASE_DUTYCYCLE)


    def control_angle(self):
        self.get_yaw()
        print("True Angle: ",self.TRUE_ANGLE,"Current Value: ",self.yaw)
        ANGLE1,ANGLE2=self.TRUE_ANGLE,self.TRUE_ANGLE
        if self.TRUE_ANGLE==0.0:
            ANGLE1=360
            ANGLE2=0
            while not ((ANGLE1-2)<=self.yaw or self.yaw<=(ANGLE2+2)):
                self.pwm2.ChangeDutyCycle(self.BASE_DUTYCYCLE)
                self.pwm1.ChangeDutyCycle(self.BASE_DUTYCYCLE)
                self.get_yaw()
                print("True Angle: ",self.TRUE_ANGLE,"Current Value: ",self.yaw)
            self.pwm2.ChangeDutyCycle(0)
            self.pwm1.ChangeDutyCycle(0)
        else:
            while not ((ANGLE1-2)<=self.yaw<=(ANGLE2+2)):
                self.pwm2.ChangeDutyCycle(self.BASE_DUTYCYCLE)
                self.pwm1.ChangeDutyCycle(self.BASE_DUTYCYCLE)
                self.get_yaw()
                print("True Angle: ",self.TRUE_ANGLE,"Current Value: ",self.yaw)
            self.pwm2.ChangeDutyCycle(0)
            self.pwm1.ChangeDutyCycle(0)
        time.sleep(1)
    
    

    def main(self):
        x,y=0,0
        theta=math.pi/2
        self.location.append((x,y))
        self.setup()
        time.sleep(5)
        d=1.30
        self.distance_control(d,1)
        x,y=x+d*math.cos(theta),y+d*math.sin(theta)
        self.location.append((x,y))
        t=math.pi/2
        theta=theta-t
        self.angle_control(t*180/math.pi,1)
        self.distance_control(0.8,1)
        x,y=x+d*math.cos(theta),y+d*math.sin(theta)
        self.location.append((x,y))
        t=math.pi/2
        theta=theta-t
        self.angle_control(t*180/math.pi,1)
        self.distance_control(1.30,1)
        x,y=x+d*math.cos(theta),y+d*math.sin(theta)
        self.location.append((x,y))
        t=math.pi/2
        theta=theta-t
        self.angle_control(t*180/math.pi,1)
        self.distance_control(0.8,1)
        x,y=x+d*math.cos(theta),y+d*math.sin(theta)
        self.location.append((x,y))
        t=math.pi/2
        theta=theta-t
        self.angle_control(t*180/math.pi,1)
        with open("coordinates2.txt", "w") as file:
            for coordinate in self.location:
                file.write(f"{coordinate[0]} {coordinate[1]}\n")


    def distance_control(self,distance,direction):
        DISTANCE=distance
        if direction==1:
            self.forward()
            self.pwm_setup(self.L_IN,self.R_OUT)

        elif direction==2:
            self.reverse()
            self.pwm_setup(self.L_OUT,self.R_IN)

        else:
            print("Proper direction not selected")

        self.control(DISTANCE)
        if int(gpio.input(12)) != int(self.buttonBR):
            self.buttonBR = int(gpio.input(12))
            self.counterBR+=1

        if int(gpio.input(7)) != int(self.buttonFL):
            self.buttonFL = int(gpio.input(7))
            self.counterFL+=1
        # print("countBR: ", self.counterBR, "counterFL: ", self.counterFL, "BR state: ", gpio.input(12), "FL state: ", gpio.input(7))
        self.gameover()
        self.pwm1.stop()
        self.pwm2.stop()
        # print("Distance Travelled: ",self.counterBR,self.counterFL)
        # if self.counterBR>self.counterFL:
        #     self.counterBR=(self.counterBR-self.counterFL)
        self.counterFL=0
        # elif self.counterFL>self.counterBR:
        #     self.counterFL=(self.counterFL-self.counterBR)
        self.counterBR=0
        time.sleep(2)
        
        

    def angle_control(self,angle,direction):
        print("In angle Control")
        self.BASE_DUTYCYCLE=50
        self.BUFFER=0
        ANGLE=angle
        DISTANCE=(ANGLE/360)*self.PI*self.WHEEl_BASE
        if direction==1:
            self.TRUE_ANGLE+=ANGLE
            self.pivotright()
            self.pwm_setup(self.L_IN,self.R_IN)

        elif direction==2:
            self.TRUE_ANGLE-=ANGLE
            self.pivotleft()
            self.pwm_setup(self.L_OUT,self.R_OUT)

        else:
            print("Proper direction not selected")
        
        
        self.TRUE_ANGLE=self.TRUE_ANGLE%360
        self.control_angle()
        if int(gpio.input(12)) != int(self.buttonBR):
            self.buttonBR = int(gpio.input(12))
            self.counterBR+=1

        if int(gpio.input(7)) != int(self.buttonFL):
            self.buttonFL = int(gpio.input(7))
            self.counterFL+=1
        # print("countBR: ", self.counterBR, "counterFL: ", self.counterFL, "BR state: ", gpio.input(12), "FL state: ", gpio.input(7))
        self.gameover()
        self.pwm1.stop()
        self.pwm2.stop()
        if self.counterBR>self.counterFL:
            self.counterBR=2*(self.counterBR-self.counterFL)
            self.counterFL=0
        elif self.counterFL>self.counterBR:
            self.counterFL=2*(self.counterFL-self.counterBR)
            self.counterBR=0
        # print(self.counterBR,self.counterFL)
        # print("Thanks Angle Reached")

if __name__ == "__main__":
    r=Robot()
    r.main()
    r.gameover()


        






