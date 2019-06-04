#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
from std_msgs.msg import Int32MultiArray, Int16


rospy.init_node("topic_list_mobile")
pub = rospy.Publisher("/Robot1/point_list1",Int32MultiArray,queue_size=1)
pub1 = rospy.Publisher("/Robot2/point_list2",Int32MultiArray,queue_size=1)
pub2 = rospy.Publisher("/obstacle/angle", Int16, queue_size=1)
pub3 = rospy.Publisher("/Robot1/arrive_flag",Int16, queue_size = 1)
Plist = Int32MultiArray()

r = rospy.Rate(6)
### **********   y axis move   ************* ###
ppp = [
45, 22, 44, 21, 
44, 20, 43, 19
# , 
# 43, 18, 43, 17, 
# 42, 16, 42, 15, 
# 41, 14, 40, 13, 
# 39, 12, 39, 11"""
]
### **********   x axis move   ************* ###
ppp2 = [
45, 22, 44, 21, 
44, 20, 43, 19, 
43, 18, 43, 17, 
42, 16, 42, 15, 
41, 14, 40, 13, 
39, 12, 39, 11
]
while not rospy.is_shutdown():
	Plist.data = ppp
	pub.publish(Plist)
	Plist.data = ppp2
	pub1.publish(Plist)
	pub2.publish(-7)
	#pub3.publish(2)
	print(Plist)
	r.sleep()
