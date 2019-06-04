#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
from capstone_msgs.msg import Smartfactory
from capstone_msgs.srv import *

x=0.0
y=0.0
theta=0.0
goal_point_x =0.0
goal_point_y =0.0
pointList = []

def add_two_points(req):
	global goal_point_x
	global goal_point_y
	global pointList
	res = goalPointResponse()
	res.send_point = list(req.goal_point)
	templist = list(req.goal_point)
	print "res :",res.send_point
	goal_point_x = templist[0]
	goal_point_y = templist[1]
	pointList = [goal_point_x,goal_point_y]
	getPoint(pointList)
	return res

def add_two_float_server():
	rospy.Service('add_goal_point', goalPoint, add_two_points)
	print "Ready to add two points."
	rospy.spin()

def newOdom (msg):
	global x
	global y
	global theta

	x=msg.pose.pose.position.x
	y=msg.pose.pose.position.y

	rot_q=msg.pose.pose.orientation
	(roll,pitch,theta)=euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
	print(x , y)
	
def getPoint (pointList):
	
	global goal_point_x
	global goal_point_y
	goal_point_x = pointList[0]
	goal_point_y = pointList[1]

###############################################################
#####     get point by service and go to the goal point  ######
###############################################################

rospy.init_node("controller_service_server")
sub = rospy.Subscriber("/odom", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)
serv = rospy.Service('add_goal_point', goalPoint, add_two_points)
speed = Twist()

r = rospy.Rate(4)

print "x : ",goal_point_x
print "y : ",goal_point_y

while not rospy.is_shutdown():
	inc_x = goal_point_x - x
	inc_y = goal_point_y - y
	
	angle_to_goal = atan2(inc_y, inc_x)
	if (angle_to_goal - theta)>0.2:
		speed.linear.x = 0.0
		speed.angular.z = 0.3
	elif (angle_to_goal - theta)<-0.2:
		speed.linear.x = 0.0
		speed.angular.z = -0.3
	elif abs(inc_x)<0.05 and abs(inc_y)<0.05:
		speed.linear.x = 0.0
		speed.angular.z = 0.0
	else:
		speed.linear.x = 0.1
		speed.angular.z = 0.0

	pub.publish(speed)
	r.sleep()
