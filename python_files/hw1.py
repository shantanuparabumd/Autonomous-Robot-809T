# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
# initialize the Raspberry Pi camera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 25
rawCapture = PiRGBArray(camera, size=(640,480))
# allow the camera to warmup
time.sleep(0.1)

# define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('final_video.avi', fourcc, 10, (640, 480))

font = cv2.FONT_HERSHEY_SIMPLEX #Setting the Font
textcolorf = (0,0,255)
textcolor = (100,200,0)
b=580
x=0
y=400
c=0
frames=0
i=0
begin=time.time()
# keep looping
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=False):
# grab the current frame
    image = frame.array
    image=cv2.flip(image,0)
    image=cv2.flip(image,1)
    # show the frame to our screen
    if frames<70:
        frame=cv2.rectangle(image,(0,0),(640,480),(0,0,0),b)
        image=cv2.putText(image,'Welcome to 809T',(i,y), font,2,textcolor,2,cv2.LINE_AA)
    if frames>250:
        image=cv2.rectangle(image,(0,0),(640,480),(0,0,0),c)
        image=cv2.putText(image,'Thankyou for Watching',(50,240), font,1.5,textcolor,1,cv2.LINE_AA)
        c=c+20
    image=cv2.putText(image,'Frames='+str(frames),(20,30), font,0.5,textcolorf,1,cv2.LINE_AA)
    
    
    
    # Press Q on keyboard to exit
    frames=frames+1
    i=i+2
    b=b-10
    if i>50:
        i=50
        y=460
    if b<0:
        b=0
    
    # write frame to video file
    out.write(image)
    cv2.imshow("Frame", image)
    key = cv2.waitKey(1) & 0xFF
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
    # press the 'q' key to stop the video stream
    if int(time.time()-begin)>100:
        break
    if key == ord("q"):       
        break