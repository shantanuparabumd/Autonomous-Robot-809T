import RPi.GPIO as GPIO
import time

# Set the pin numbering mode
GPIO.setmode(GPIO.BOARD)


# self.led_pins = {'RED':22, 'GREEN':26, 'BLUE':24, 'YELLOW':38, 'WHITE':40}
# Define the pin numbers for the LEDs
pin=22

# Set up the pins as outputs

GPIO.setup(pin, GPIO.OUT)

# Perform the chasing LED effect
while True:
    
    GPIO.output(pin, GPIO.HIGH)
    time.sleep(0.2)
    GPIO.output(pin, GPIO.LOW)
    time.sleep(0.2)
    
        

# Clean up the GPIO pins
GPIO.cleanup()
