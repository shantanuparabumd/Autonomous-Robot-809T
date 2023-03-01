import cv2
import numpy as np

# Define a function to update the mask based on the current slider positions
def update_mask(lower_hue, lower_saturation, lower_value, upper_hue, upper_saturation, upper_value):
    # Define the lower and upper bounds of the color range based on the current slider positions
    lower_range = np.array([lower_hue, lower_saturation, lower_value])
    upper_range = np.array([upper_hue, upper_saturation, upper_value])

    # Create a mask for the color range using cv2.inRange()
    mask = cv2.inRange(hsv_image, lower_range, upper_range)

    # Apply the mask to the original image using cv2.bitwise_and()
    result = cv2.bitwise_and(image, image, mask=mask)

    # Display the result
    cv2.imshow("Result", result)


# Load the image
img = cv2.imread("capture.jpg")
scale_percent = 60 # percent of original size
width = int(img.shape[1] * scale_percent / 100)
height = int(img.shape[0] * scale_percent / 100)
dim = (width, height)
# resize image
image = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)

# Convert the image to the HSV color space
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Create a window to display the result
cv2.namedWindow("Result")

# Create sliders for the lower and upper bounds of the color range using cv2.createTrackbar()
cv2.createTrackbar("Lower Hue", "Result", 0, 179, lambda x: None)
cv2.createTrackbar("Lower Saturation", "Result", 0, 255, lambda x: None)
cv2.createTrackbar("Lower Value", "Result", 0, 255, lambda x: None)
cv2.createTrackbar("Upper Hue", "Result", 0, 179, lambda x: None)
cv2.createTrackbar("Upper Saturation", "Result", 0, 255, lambda x: None)
cv2.createTrackbar("Upper Value", "Result", 0, 255, lambda x: None)

# Set the initial slider positions
cv2.setTrackbarPos("Lower Hue", "Result", 0)
cv2.setTrackbarPos("Lower Saturation", "Result", 0)
cv2.setTrackbarPos("Lower Value", "Result", 0)
cv2.setTrackbarPos("Upper Hue", "Result", 179)
cv2.setTrackbarPos("Upper Saturation", "Result", 255)
cv2.setTrackbarPos("Upper Value", "Result", 255)

# Display the original image
cv2.imshow("Result", image)

while True:
    # Wait for a key press
    key = cv2.waitKey(1) & 0xFF

    # If the 'q' key is pressed, quit the program
    if key == ord("q"):
        break

    # Get the current slider positions
    lower_hue = cv2.getTrackbarPos("Lower Hue", "Result")
    lower_saturation = cv2.getTrackbarPos("Lower Saturation", "Result")
    lower_value = cv2.getTrackbarPos("Lower Value", "Result")
    upper_hue = cv2.getTrackbarPos("Upper Hue", "Result")
    upper_saturation = cv2.getTrackbarPos("Upper Saturation", "Result")
    upper_value = cv2.getTrackbarPos("Upper Value", "Result")

    # Update the mask based on the current slider positions
    update_mask(lower_hue, lower_saturation, lower_value, upper_hue, upper_saturation, upper_value)

# Destroy all windows
print("Lower Hue: ",lower_hue)
print("Lower Saturation: ",lower_saturation)
print("Lower Value: ",lower_value)

print("Upper Hue: ",upper_hue)
print("Upper Saturation: ",upper_saturation)
print("Upper Value: ",upper_value)

cv2.destroyAllWindows()
