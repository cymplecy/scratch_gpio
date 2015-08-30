#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2, math
import numpy as np
class ColourTracker:
    def __init__(self):
        cv2.namedWindow("ColourTrackerWindow", cv2.CV_WINDOW_AUTOSIZE)
        self.capture = cv2.VideoCapture(0)
        self.capture.set(3,240)
        self.capture.set(4,160)
        self.scale_down = 1
        
    def run(self):
        while True:
            f, orig_img = self.capture.read()
            #orig_img = cv2.flip(orig_img, 1)
            img = cv2.GaussianBlur(orig_img, (15,15), 0)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            #img = cv2.resize(img, (len(orig_img[0]) / self.scale_down, len(orig_img) / self.scale_down))
            red_lower = np.array((65,64, 128),np.uint8)
            red_upper = np.array((75, 128, 224),np.uint8)
            red_binary = cv2.inRange(img, red_lower, red_upper)
            test = red_binary
            #test[:,:,1] = 0
            #test[:,:,2] = 0 
            #test = red_binary
            #test = cv2.GaussianBlur(orig_img, (5,5), 0)
            #test[:,:,0] = 0
            #test[:,:,1] = 0

            #dilation = np.ones((3, 3), "uint8")
            #test = cv2.erode(red_binary, dilation)
            dilation = np.ones((15,15), "uint8")
            test = cv2.dilate(test, dilation)     
            red_binary = test
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
            cv2.imshow("ColourTrackerWindow", orig_img)
            cv2.imshow("mask", test)
            if cv2.waitKey(20) == 27:
                cv2.destroyWindow("ColourTrackerWindow")
                self.capture.release()
                break

if __name__ == "__main__":
    colour_tracker = ColourTracker()
    colour_tracker.run()