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
14, 13, 14, 12,
15, 11, 15, 10,
16, 9, 17, 8, 
18, 7, 19, 6, 
20, 5, 21, 4, 
22, 3, 23, 2
]
### **********   x axis move   ************* ###
ppp2 = [
14, 13, 14, 12, 
15, 11, 15, 10, 
16, 9, 17, 8, 
18, 7, 19, 6, 
20, 5, 21, 4, 
22, 3, 23, 2
]
while not rospy.is_shutdown():
	Plist.data = ppp
	pub.publish(Plist)
	Plist.data = ppp2
	pub1.publish(Plist)
	print(Plist)
	r.sleep()
