import RPi.GPIO as gpio
import numpy as np
import time
import datetime
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
        self.BUFFER=0.75
        self.BASE_DUTYCYCLE=25

        self.counterBR = np.uint64(0)
        self.counterFL = np.uint64(0)
        self.buttonBR = int(0)
        self.buttonFL = int(0)
        self.location=[]

        self.KP=0

        # PINS
        self.L_IN=31
        self.L_OUT=33
        self.R_IN=35
        self.R_OUT=37
        self.EN_LEFT=7
        self.EN_RIGHT=12

        self.setup()

        

        counter = np.uint64(0)


        #FILES
        self.fl = open('hw7data.txt','a') 

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
        while self.counterBR<=TOTAL_TICKS or self.counterFL<TOTAL_TICKS:
            # print("countBR: ", self.counterBR, "counterFL: ", self.counterFL, "BR state: ", gpio.input(12), "FL state: ", gpio.input(7))
            if int(gpio.input(self.EN_RIGHT)) != int(self.buttonBR):
                self.buttonBR = int(gpio.input(self.EN_RIGHT))
                self.counterBR+=1

            if int(gpio.input(self.EN_LEFT)) != int(self.buttonFL):
                self.buttonFL = int(gpio.input(self.EN_LEFT))
                self.counterFL+=1
            # print("Value: ",val)
            # val=int(input("Enter PWM value: "))
            error=self.counterBR-self.counterFL
            # print(error)
            if error>0:
                # print("Duty Cycle: Left: ",min(self.BASE_DUTYCYCLE+error*KP,100)," Right: ",self.BASE_DUTYCYCLE)
                self.pwm1.ChangeDutyCycle(min(self.BASE_DUTYCYCLE+error*self.KP,100))
                self.pwm2.ChangeDutyCycle(self.BASE_DUTYCYCLE)
            elif error<0:
                # print("Duty Cycle: Left: ",self.BASE_DUTYCYCLE," Right: ",min(self.BASE_DUTYCYCLE-error*KP,100))
                self.pwm2.ChangeDutyCycle(min(self.BASE_DUTYCYCLE-error*self.KP,100))
                self.pwm1.ChangeDutyCycle(self.BASE_DUTYCYCLE)
            else:
                # print("Duty Cycle: Left: ",self.BASE_DUTYCYCLE," Right: ",self.BASE_DUTYCYCLE)
                self.pwm2.ChangeDutyCycle(self.BASE_DUTYCYCLE)
                self.pwm1.ChangeDutyCycle(self.BASE_DUTYCYCLE)
            


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
        theta=theta+t
        self.angle_control(t*180/math.pi,2)
        self.distance_control(0.8,1)
        x,y=x+d*math.cos(theta),y+d*math.sin(theta)
        self.location.append((x,y))
        t=math.pi/2
        theta=theta+t
        self.angle_control(t*180/math.pi,2)
        self.distance_control(1.30,1)
        x,y=x+d*math.cos(theta),y+d*math.sin(theta)
        self.location.append((x,y))
        t=math.pi/2
        theta=theta+t
        self.angle_control(t*180/math.pi,2)
        self.distance_control(0.8,1)
        x,y=x+d*math.cos(theta),y+d*math.sin(theta)
        self.location.append((x,y))
        t=math.pi/2
        theta=theta+t
        self.angle_control(t*180/math.pi,2)
        with open("coordinates1.txt", "w") as file:
            for coordinate in self.location:
                file.write(f"{coordinate[0]} {coordinate[1]}\n")



    def distance_control(self,distance,direction):
        self.BASE_DUTYCYCLE=50
        self.BUFFER=0.25
        self.KP=5
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
        print("countBR: ", self.counterBR, "counterFL: ", self.counterFL, "BR state: ", gpio.input(12), "FL state: ", gpio.input(7))
        self.gameover()
        self.pwm1.stop()
        self.pwm2.stop()
        if self.counterBR<self.counterFL:
            self.counterBR=self.counterFL-self.counterBR
            self.counterFL=0
        elif self.counterFL<self.counterBR:
            self.counterFL=self.counterBR-self.counterFL
            self.counterBR=0
        else:
            self.counterBR=0
            self.counterFL=0
        print("Distance: BR: ",self.counterBR," FL: ",self.counterFL)
        time.sleep(1)
        

    def angle_control(self,angle,direction):
        self.BASE_DUTYCYCLE=75
        self.BUFFER=0.8
        self.KP=5
        ANGLE=angle
        if direction==1:
            self.pivotright()
            self.pwm_setup(self.L_IN,self.R_IN)

        elif direction==2:
            self.pivotleft()
            self.pwm_setup(self.L_OUT,self.R_OUT)

        else:
            print("Proper direction not selected")
        DISTANCE=(ANGLE/360)*self.PI*self.WHEEl_BASE
        print(DISTANCE)
        self.control(DISTANCE)
        if int(gpio.input(12)) != int(self.buttonBR):
            self.buttonBR = int(gpio.input(12))
            self.counterBR+=1

        if int(gpio.input(7)) != int(self.buttonFL):
            self.buttonFL = int(gpio.input(7))
            self.counterFL+=1
        print("countBR: ", self.counterBR, "counterFL: ", self.counterFL, "BR state: ", gpio.input(12), "FL state: ", gpio.input(7))
        self.gameover()
        self.pwm1.stop()
        self.pwm2.stop()
        if self.counterBR<self.counterFL:
            self.counterBR=5*(self.counterFL-self.counterBR)
            self.counterFL=0
        elif self.counterFL<self.counterBR:
            self.counterFL=5*(self.counterBR-self.counterFL)
            self.counterBR=0
        else:
            self.counterBR=0
            self.counterFL=0
        print("Angle: BR: ",self.counterBR," FL: ",self.counterFL)
        time.sleep(1)

if __name__ == "__main__":
    r=Robot()
    r.main()
    r.gameover()


        





