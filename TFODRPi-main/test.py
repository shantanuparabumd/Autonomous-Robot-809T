import numpy as np
import cv2
import tflite_runtime.interpreter as tflite
import picamera
from picamera.array import PiRGBArray
import time

# Load the TFLite model
interpreter = tflite.Interpreter(model_path="detect.tflite")
interpreter.allocate_tensors()

# Get input and output tensors
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# Set up the camera
camera = picamera.PiCamera()
camera.resolution = (640, 480)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(640, 480))

# Warm up the camera
time.sleep(2)

# Loop over the frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    # Convert the frame to a numpy array
    image = frame.array

    # Preprocess the input image
    image = cv2.resize(image, (input_details[0]['shape'][2], input_details[0]['shape'][1]))
    image = np.expand_dims(image, axis=0)
    image = np.float32(image)
    image = (2.0 / 255.0) * image - 1.0

    # Set the input tensor
    interpreter.set_tensor(input_details[0]['index'], image)

    # Run inference
    interpreter.invoke()

    # Get the output tensor
    output = interpreter.get_tensor(output_details[0]['index'])
    boxes=np.squeeze(interpreter.get_tensor(output_details[1]['index']))
    count=interpreter.get_tensor(output_details[3]['index'])
    print(count.shape[1])

    # # Get the class labels
    # with open("labels.txt", "r") as f:
    #     labels = [line.strip() for line in f.readlines()]

    # # Print the top 5 predictions
    # top_k = output[0].argsort()[-5:][::-1]
    # print(output)
    # for i in top_k:
    #     print(f"{labels[i]}: {output[0][i]}")
    #     # print(output[:][i])
    #     # Get the bounding box coordinates
    #     ymin, xmin, ymax, xmax = boxes[i]

    #     # Scale the coordinates to the original image size
    #     height, width, channels = image.shape
    #     xmin = int(xmin * width)
    #     ymin = int(ymin * height)
    #     xmax = int(xmax * width)
    #     ymax = int(ymax * height)

    #     # Draw the bounding box
    #     cv2.rectangle(image, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)

    #     # Print the label and confidence score
    #     label = f"{labels[i]}: {float(output[0][i]):.2f}"
    #     cv2.putText(image, label, (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # # Display the image
    # cv2.imshow("Object detection", image)
    # cv2.waitKey(1)

    # Clear the stream in preparation for the next frame
    rawCapture.truncate(0)
