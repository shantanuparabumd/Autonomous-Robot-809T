import RPi.GPIO as gpio
import time


#Define pin allocations
trig=16
echo=18

def distance():
    gpio.setmode(gpio.BOARD)
    gpio.setup(trig,gpio.OUT)
    gpio.setup(echo,gpio.IN)

    #Ensure output has no value
    gpio.output(trig, False)
    time.sleep(0.01)

    #Genereate trigger pulse
    gpio.output(trig,True)
    time.sleep(0.00001)
    gpio.output(trig, False)

    #Generate echo time signal
    while gpio.input(echo) == 0:
        pulse_start = time.time()

    while gpio.input(echo)== 1:
        pulse_end = time.time()

    pulse_duration = pulse_end-pulse_start

    #Convert time to distance
    distance = pulse_duration*17150
    distance = round(distance, 2)

    #Cleanup gpio 7 return distance
    gpio.cleanup()
    return distance

start=time.time()

while time.time()-start<10:
    print("Distance: ", distance(), " cm" )

print("Scan Completed")
