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
63, 52, 62, 51, 
62, 50, 61, 49, 
60, 48, 59, 47, 
58, 46, 57, 45, 
56, 44, 55, 43, 
54, 42, 53, 41, 
52, 40, 51, 39, 
50, 38, 50, 37, 
50, 36, 50, 35, 
50, 34, 50, 33, 
50, 32, 50, 31, 
50, 30, 50, 29, 
50, 28, 50, 27, 
50, 26, 50, 25, 
50, 24, 50, 23, 
49, 22, 49, 21, 
48, 20, 47, 19, 
46, 18, 45, 17, 
44, 16, 43, 15, 
42, 14, 41, 13, 
40, 12
]
### **********   x axis move   ************* ###
ppp2 = [
64, 53, 64, 52, 65, 51, 65, 50, 66, 49, 67, 48, 68, 47, 69, 46, 70, 45, 71, 44, 72, 43, 73, 42, 74, 41, 75, 40, 76, 39, 77, 38, 78, 37, 79, 36, 80, 35, 81, 34, 81, 33, 81, 32, 81, 31, 81, 30, 81, 29, 81, 28, 81, 27, 81, 26, 81, 25, 81, 24, 81, 23, 81, 22, 81, 21, 81, 20, 81, 19, 81, 18, 82, 17, 83, 16, 84, 15, 85, 14, 86, 13, 87, 12, 88, 11, 89, 10, 90, 9, 91, 8
]
while not rospy.is_shutdown():
	Plist.data = ppp
	pub.publish(Plist)
	Plist.data = ppp2
	pub1.publish(Plist)
	pub2.publish(83)
	#pub3.publish(2)
	print(Plist)
	r.sleep()
