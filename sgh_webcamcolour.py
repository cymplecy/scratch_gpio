#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2, math
import numpy as np
import threading
lock = threading.Lock()
import picamera
import picamera.array
import time as time



class ColourTracker(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self._stop = threading.Event()
        #cv2.namedWindow("ColourTrackerWindow", cv2.CV_WINDOW_AUTOSIZE)
        #self.capture = cv2.VideoCapture(0)
        #self.capture.set(3,240)
        #self.capture.set(4,160)
        self.scale_down = 1
        self.green = [None,None,False]
        self.limits = [110,64,128,130,128,224]
        

        print "ColourTracker init"
        
    def stop(self):
        self._stop.set()
        print "ColourTracker Stop Set"

    def stopped(self):
        return self._stop.isSet()        

        
    def run(self):
        print "ColourTracker started"
        with picamera.PiCamera() as camera:
            with picamera.array.PiRGBArray(camera) as stream:
                camera.resolution = (320, 240)


                while not self.stopped():
                    print "loop"
                    #for x in ["ab"]:
                    camera.capture(stream, 'bgr', use_video_port=True)
                    orig_img = stream.array
                    #orig_img = cv2.flip(orig_img, 1)
                    img = cv2.GaussianBlur(orig_img, (15,15), 0)
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                    #print orig_img
                    #img = cv2.resize(img, (len(orig_img[0]) / self.scale_down, len(orig_img) / self.scale_down))
                    red_lower = np.array((self.limits[0],self.limits[1],self.limits[2]),np.uint8)
                    red_upper = np.array((self.limits[3],self.limits[4],self.limits[5]),np.uint8)
                    red_binary = cv2.inRange(img, red_lower, red_upper)
                    test = cv2.copyMakeBorder(red_binary,0,0,0,0,cv2.BORDER_REPLICATE)
                    #test[:,:,1] = 0
                    #test[:,:,2] = 0
                    #test = red_binary
                    #test = cv2.GaussianBlur(orig_img, (5,5), 0)
                    #test[:,:,0] = 0
                    #test[:,:,1] = 0

                    #dilation = np.ones((3, 3), "uint8")
                    #test = cv2.erode(red_binary, dilation)
                    dilation = np.ones((15,15), "uint8")
                    test = cv2.dilate(red_binary, dilation)
                    test = cv2.copyMakeBorder(red_binary,0,0,0,0,cv2.BORDER_REPLICATE)
                    contours, hierarchy = cv2.findContours(red_binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
                    max_area = 0
                    largest_contour = None
                    for idx, contour in enumerate(contours):
                        area = cv2.contourArea(contour)
                        if area > max_area:
                            max_area = area
                            largest_contour = contour
                    if not largest_contour == None:
                        moment = cv2.moments(largest_contour)
                        if moment["m00"] > 500:
                            rect = cv2.minAreaRect(largest_contour)
                            rect = ((rect[0][0], rect[0][1] ), (rect[1][0], rect[1][1] ), rect[2])
                            box = cv2.cv.BoxPoints(rect)
                            box = np.int0(box)
                            cv2.drawContours(orig_img,[box], 0, (0, 0, 255), 2)
                            with lock:
                                self.green = [rect[0][0],rect[0][1],True]
                    cv2.imshow("ColourTrackerWindow", orig_img)
                    cv2.imshow("mask", test)
                    c = cv2.waitKey(100)
                    stream.seek(0)
                    stream.truncate()
        print "ColourTracker stopped"

        # webcam code
        # print "ColourTracker started"
        # while not self.stopped():
        #     print "loop"
        #     f, orig_img = self.capture.read()
        #     #orig_img = cv2.flip(orig_img, 1)
        #     img = cv2.GaussianBlur(orig_img, (15,15), 0)
        #     img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        #     #img = cv2.resize(img, (len(orig_img[0]) / self.scale_down, len(orig_img) / self.scale_down))
        #     red_lower = np.array((65,64, 128),np.uint8)
        #     red_upper = np.array((75, 128, 224),np.uint8)
        #     red_lower = np.array((self.limits[0],self.limits[1],self.limits[2]),np.uint8)
        #     red_upper = np.array((self.limits[3],self.limits[4],self.limits[5]),np.uint8)
        #     red_binary = cv2.inRange(img, red_lower, red_upper)
        #     test = red_binary
        #     #test[:,:,1] = 0
        #     #test[:,:,2] = 0
        #     #test = red_binary
        #     #test = cv2.GaussianBlur(orig_img, (5,5), 0)
        #     #test[:,:,0] = 0
        #     #test[:,:,1] = 0
        #
        #     #dilation = np.ones((3, 3), "uint8")
        #     #test = cv2.erode(red_binary, dilation)
        #     dilation = np.ones((15,15), "uint8")
        #     test = cv2.dilate(test, dilation)
        #     red_binary = test
        #     contours, hierarchy = cv2.findContours(red_binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        #     max_area = 0
        #     largest_contour = None
        #
        #     for idx, contour in enumerate(contours):
        #         area = cv2.contourArea(contour)
        #         if area > max_area:
        #             max_area = area
        #             largest_contour = contour
        #     if not largest_contour == None:
        #         moment = cv2.moments(largest_contour)
        #         if moment["m00"] > 500:
        #             rect = cv2.minAreaRect(largest_contour)
        #             rect = ((rect[0][0], rect[0][1] ), (rect[1][0], rect[1][1] ), rect[2])
        #             box = cv2.cv.BoxPoints(rect)
        #             box = np.int0(box)
        #             cv2.drawContours(orig_img,[box], 0, (0, 0, 255), 2)
        #             with lock:
        #                 self.green = [rect[0][0],rect[0][1],True]
        #     cv2.imshow("ColourTrackerWindow", orig_img)
        #     cv2.imshow("mask", test)
        #     c = cv2.waitKey(100)
        #     #time.sleep(0.5)
        #
        # cv2.destroyWindow("ColourTrackerWindow")
        # cv2.destroyWindow("mask")
        # self.capture.release()
        # print "ColourTracker stopped"

