#!/usr/bin/env python2

import cv2
import rospy
import numpy as np
import Img_process
from sensor_msgs.msg import CompressedImage     ## use this to sub & pub the videodata
rospy.init_node('cam')

def video_processing(ros_data):
    np_arr = np.fromstring(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    frame = image_np
    img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, base_map = cv2.threshold(img_gray, 72, 255, 1)
    lower = np.array([126, 134, 0])
    higher = np.array([255, 255, 255])
    lower2 = np.array([91,161,0])
    higher2 = np.array([124,255,255])
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    Gmask = cv2.inRange(hsv, lower, higher)
    Gmask2 = cv2.inRange(hsv,lower2,higher2)
    cv2.imshow('img', frame)
    cv2.imshow('img_', base_map)
    cv2.imshow('RED', Gmask)
    cv2.imshow('BLUE', Gmask2)
    k = cv2.waitKey(1) & 0xFF


if __name__ == '__main__':
    sub = rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, video_processing)
    rospy.spin()
