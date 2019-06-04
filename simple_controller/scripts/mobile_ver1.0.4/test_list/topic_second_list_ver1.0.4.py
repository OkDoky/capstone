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
0,9,0,10,
1,11,2,12,
3,12,4,12,
5,13,6,14,
7,13,7,14,
8,15,9,16,
10,14,11,13

]
### **********   x axis move   ************* ###
ppp2 = [
0,0,1,0,2,0,3,0,
4,0,5,0,6,0,7,0,
8,0,9,0
]
while not rospy.is_shutdown():
	Plist.data = ppp
	pub.publish(Plist)
	Plist.data = ppp
	pub1.publish(Plist)
	print(Plist)
	r.sleep()
