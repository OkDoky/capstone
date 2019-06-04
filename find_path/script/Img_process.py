#!/usr/bin/env python2
import numpy as np
import cv2


class Basic_map:
    def __init__(self):
        self.Color_HSV = {}
        self.Color_HSV['RED'] = [157, 90, 33, 255, 255, 255]
        self.Color_HSV['BLUE'] = [79, 131, 0, 148, 255, 255]
#    lower = np.array([126, 134, 0])
#    higher = np.array([255, 255, 255])
    # lower2 = np.array([91,161,0])
    # higher2 = np.array([124,255,255])
        self.x = 0
        self.y = 0
        return
    def find(self,img, color): #find specific color points, in this case start, end_nodes
        lowerBound = np.array(self.Color_HSV[color][:3])
        upperBound = np.array(self.Color_HSV[color][3:6])
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        color_mask = cv2.inRange(hsv, lowerBound, upperBound)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        erosion = cv2.erode(color_mask, kernel,iterations =1)
        dilation = cv2.dilate(erosion, kernel, iterations=3)
        #ret, thr = cv2.threshold(color_mask, 127, 255, 0)
        _, contours, _ = cv2.findContours(dilation, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            for i in range(len(contours)):
                # Get area value
                area = cv2.contourArea(contours[i])
                if area > 200:  # minimum  area
                    rect = cv2.minAreaRect(contours[i])
                    (self.x, self.y), (w, h), angle = cv2.minAreaRect(contours[i])
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    cv2.drawContours(img, [box], 0, (0, 0, 255), 2)
                    return True, round(self.x), round(self.y), dilation, w, h
                else:
                    return False, 0, 0, dilation, 0, 0
        else:
            return False, 0, 0, dilation, 0, 0

    def obstacle_load(self,img): # make base_map that has obstacle pixel data
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, base_map = cv2.threshold(img_gray, 72, 255, 1)
        return base_map
