import cv2

cap = cv2.VideoCapture(0, cv2.CAP_GSTREAMER)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    # If frame is read correctly, show it in a window
    if ret:
        cv2.imshow('frame', frame)

    # Exit the program if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture and destroy the window
cap.release()
cv2.destroyAllWindows()
