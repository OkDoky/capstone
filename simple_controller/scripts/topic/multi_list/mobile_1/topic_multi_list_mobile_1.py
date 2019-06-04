#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
from capstone_msgs.msg import *


rospy.init_node("topic_list_mobile_1")
pub = rospy.Publisher("/point_list_1",Smartfactory,queue_size=1)

Plist = Smartfactory()

r = rospy.Rate(4)
ppp = [0.1,0.1, 0.2,0.1, 0.25,0.25, 0.35,0.1, 0.5,0.75, 1.2,2.3, 2.5,1.5]
#ppp = [[1.0,2.2],[2.3,3.5],[3.12,4.00]]
while not rospy.is_shutdown():
	Plist.point_list = list(ppp)
	pub.publish(Plist)
	r.sleep()
