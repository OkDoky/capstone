#!/usr/bin/env python2
import rospy
import numpy as np
from std_msgs.msg import Int32MultiArray

rospy.init_node('sub')

def callback(msg):
	print("what?", msg.data)

if __name__ == '__main__':
	sub = rospy.Subscriber('/point_list', Int32MultiArray, callback, queue_size =1)
	rospy.spin()
