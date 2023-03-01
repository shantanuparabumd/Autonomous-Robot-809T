import RPi.GPIO as gpio
import time
import cv2


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
distance_array=[]
for i in range(10):
    distance_array.append(distance())
    time.sleep(1)
print("Scan Completed")
# Read the image from file
image = cv2.imread('capture.jpg')
image=cv2.flip(image,0)
image=cv2.flip(image,1)
# Add text to the image
distance_string = "Distance: {:.2f}".format(sum(distance_array)/len(distance_array))
cv2.putText(image, distance_string, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

cv2.imwrite('distance_image.jpg', image)

# Display the image with the text
cv2.imshow('Output', image)
cv2.waitKey(0)
cv2.destroyAllWindows()

