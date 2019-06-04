#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
from std_msgs.msg import Int32MultiArray


rospy.init_node("topic_list_mobile")
pub = rospy.Publisher("/Robot1/point_list1",Int32MultiArray,queue_size=1)
pub1 = rospy.Publisher("/Robot2/point_list2",Int32MultiArray,queue_size=1)

Plist = Int32MultiArray()

r = rospy.Rate(10)
### **********   y axis move   ************* ###
ppp = [
0,0,0,1,0,2,0,3,
0,4,0,5,0,6,0,7,
0,8,0,9
]
### **********   x axis move   ************* ###
ppp2 = [
0,15,0,15,1,16,1,16,
2,18,2,18,4,19,4,19,
5,20,5,20,7,21,7,21,
8,20,8,20,9,19,9,19,
10,18,10,18,11,17,11,17,
12,16,12,16,13,14,13,14
]
while not rospy.is_shutdown():
	Plist.data = ppp2
	pub.publish(Plist)
	Plist.data = ppp2
	pub1.publish(Plist)
	print(Plist)
	r.sleep()
