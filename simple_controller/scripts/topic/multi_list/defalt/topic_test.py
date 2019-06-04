#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
from std_msgs.msg import Int32MultiArray

rospy.init_node("topic_list_mobile")
pub = rospy.Publisher("/point_list",Int32MultiArray,queue_size=1)

Plist = Int32MultiArray()

r = rospy.Rate(4)
#ppp = [0.1,0.1, 0.2,0.1, 0.25,0.25, 0.35,0.1, 0.5,0.75, 1.2,2.3, 2.5,1.5]
ppp = [[1,2],[2,3],[3,4]]
while not rospy.is_shutdown():
	Plist.data = ppp
	Plist.layout = len(ppp)
	pub.publish(Plist)
	print(Plist.data)
	print('layout = ', Plist.layout)
	r.sleep()
