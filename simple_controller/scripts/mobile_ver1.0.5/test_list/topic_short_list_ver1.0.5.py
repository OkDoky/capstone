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
0,24,0,24,1,25,1,25,
3,28,3,28,6,30,6,30,
8,32,8,32,11,33,11,33,
12,32,12,32,14,30,14,30,
16,28,16,28,17,27,17,27,
19,25,19,25,20,22,20,22
]
### **********   x axis move   ************* ###
ppp2 = [
0,240,0,240,16,256,16,256,
32,288,32,288,64,304,64,304,
80,320,80,320,112,336,112,336,
128,320,128,320,144,304,144,304,
160,288,160,288,176,272,176,272,
192,256,192,256,208,224,208,224
]
while not rospy.is_shutdown():
	Plist.data = ppp
	pub.publish(Plist)
	Plist.data = ppp2
	pub1.publish(Plist)
	print(Plist)
	r.sleep()
