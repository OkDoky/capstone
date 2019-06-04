#!/usr/bin/env python2

import cv2
import rospy
import numpy as np
import Img_process
from resize import resize
from astar import astar
from sensor_msgs.msg import CompressedImage     ## use this to sub & pub the videodata
from std_msgs.msg import Int32MultiArray
from capstone_msgs.msg import *

rospy.init_node('cam')
pub = rospy.Publisher("/point_list", Int32MultiArray ,queue_size = 1)
flag = 0
final_path = 0
def finding_path(ros_data):
  global flag
  global final_path
  np_arr = np.fromstring(ros_data.data, np.uint8)
  image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

  frame = image_np
  #cv2.imwrite('/home/benlee/catkin_ws/src/find_path/script/map.png',frame, params=[cv2.IMWRITE_PNG_COMPRESSION,0])
  #frame = cv2.imread('/home/benlee/catkin_ws/src/find_path/script/map.png',cv2.IMREAD_COLOR)
  start_point = Img_process.Basic_map()
  end_point = Img_process.Basic_map()
  obstacle_map = Img_process.Basic_map()

  ret_start,start_x, start_y, start_img, s_w, s_h = start_point.find(frame, 'BLUE')
  ret_end, end_x, end_y, end_img, e_w, e_h = end_point.find(frame, 'RED')
  #print ("start",ret_start,"end",ret_end)
  capstone_map = obstacle_map.obstacle_load(frame)
  cv2.imshow('end_img', end_img)
  cv2.imshow('img', frame)
  cv2.imshow('start_img', start_img)
  cv2.imshow('obstacle', capstone_map)

  if ret_start == 1 and ret_end == 1:
    start = (int(start_x), int(start_y))
    end = (int(end_x), int(end_y))
    # print("start:", start)
    # print("end", end)
    if flag == 0:
      path = astar(capstone_map, start, end)
      final_path = Int32MultiArray(data = path)
      pub.publish(final_path)
      #print ("path", path)
      if path is not None and flag == 0:
        if len(path)>0:
          for i in range(0, len(path), 2):
            # print ("i",i)
            # print("path:", (path[i],path[i+1]))
            cv2.circle(frame, (path[i],path[i+1]), 1, (0,0,255), 1)
            flag = 1
          cv2.imwrite('/home/benlee/catkin_ws/src/find_path/script/path.png',frame, params=[cv2.IMWRITE_PNG_COMPRESSION,0])
        else:
          print("no path bro")
  else:
    print(" no image, setting random start,end poinst")

  found_path = cv2.imread('/home/benlee/catkin_ws/src/find_path/script/path.png',cv2.IMREAD_COLOR)
  flag = 0
  cv2.imshow('find path',found_path)
  cv2.waitKey(1) & 0xFF

if __name__ == '__main__':
   sub = rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, finding_path)
   rospy.spin()
