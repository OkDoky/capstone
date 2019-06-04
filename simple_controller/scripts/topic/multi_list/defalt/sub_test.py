#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
from capstone_msgs.msg import Smartfactory
from std_msgs.msg import Int32MultiArray

x=0
y=0
theta=0.0
goal_x = 0
goal_y = 0
listnum = 0
listlangth = 0

def newOdom (msg):
	global x
	global y
	global theta

	x=msg.pose.pose.position.x
	y=msg.pose.pose.position.y
	
	rot_q=msg.pose.pose.orientation
	(roll,pitch,theta)=euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def callback(sf):
	global goal_x
	global goal_y
	global listnum
	global listlangth
	print('sf.data = ',sf.data)
	listlangth = sf.layout
	print(listlangth)
	if listnum < listlangth:
		goal_list = sf.data[listnum]
		goal_x = goal_list[0]
		goal_y = goal_list[1]


rospy.init_node("speed_controller")

sub = rospy.Subscriber("/point_list", Int32MultiArray, callback)
sub = rospy.Subscriber("/odom", Odometry, newOdom)		###(topic_name, topic_type, function)
pub = rospy.Publisher("/cmd_vel", Twist,queue_size=1)	###(topic_name, topic_type, queue_size)

speed = Twist()

r = rospy.Rate(4)


#goal.x = 5.0
#goal.y = 5.0

#########################################
###start at 0,0 and go to 5,5 and stop###
#########################################

while not rospy.is_shutdown():
	inc_x = goal_x - x
	inc_y = goal_y - y
	
	angle_to_goal = atan2(inc_y, inc_x)
	
	if (angle_to_goal - theta)>0.1:
		speed.linear.x = 0.0
		speed.angular.z = 0.1
	elif abs(inc_x)<0.1 and abs(inc_y)<0.1:
		speed.linear.x = 0.0
		speed.angular.z = 0.0
		listnum = listnum + 2
	elif (angle_to_goal - theta)<-0.1:
		speed.linear.x = 0.0
		speed.angular.z = -0.1
	else:
		speed.linear.x = 0.2
		speed.angular.z = 0.0
	print(goal_x, goal_y)
	pub.publish(speed)
	r.sleep()
