#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
from std_msgs.msg import Int32MultiArray


rospy.init_node("topic second list")
pub = rospy.Publisher("/Robot1/next_point1",Int32MultiArray,queue_size=1)
pub1 = rospy.Publisher("/Robot2/next_point2",Int32MultiArray,queue_size=1)

Plist = Int32MultiArray()

r = rospy.Rate(10)
### **********   y axis move   ************* ###
ppp = [
20,22,20,22

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
