# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
rawCapture = PiRGBArray(camera)

# allow the camera to warmup
time.sleep(0.1)

while(1):

    # grab frames from the camera
    camera.capture(stream, 'bgr', use_video_port=True)
    image = stream.array

    # display the image on screen and wait for a keypress
    cv2.imshow("Image", image)
    cv2.waitKey(0)