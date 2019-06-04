#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import *
from std_msgs.msg import Int32MultiArray, Int16


x=0.0
y=0.0
theta=0.0
flag = 0
PI = pi

goal_x = 0.0
goal_y = 0.0
init_x =0.0
init_y = 0.0
listnum = 0
listlength = 0
pointList = 0
err_theta = 0.0
err_theta_p = 0.0
r_size_x = 20.0/0.8
r_size_y = 20.0/0.8
threshold = 1.8

yaw = 0.0
r2d = 180.0/PI

# def gaussian_function(mu, sigma2, x):
# 	coefficient = 1.0 / sqrt(2.0 * PI * sigma2)
# 	exponential = exp(-0.5 * (x-mu)**2 / sigma2)


def newOdom (msg):
	global x
	global y
	global theta
	global pointList
	global listnum
	global init_x
	global init_y
	global flag
	global yaw

	if flag == 0:
		init_x = msg.pose.pose.position.x*r_size_x
		init_y = msg.pose.pose.position.y*r_size_y
		flag = 1;

	x=msg.pose.pose.position.x*r_size_x + 32.0 - float(pointList[0])
	y=msg.pose.pose.position.y*r_size_y + float(pointList[1])

	rot_q=msg.pose.pose.orientation
	(roll,pitch,theta)=euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
	print('theta : ', theta)
	if (theta<0):
		theta = -(theta+pi)
	yaw = theta * r2d

def callback(sf):
	global goal_x
	global goal_y
	global listnum
	global listlength
	global pointList
	pointList = sf.data
	listlength = len(sf.data)
	
def thetaPID():
	global err_theta
	global err_theta_p
	Kp = 0.4
	Ki = 0.06
	pid_theta = err_theta*Kp + (err_theta-err_theta_p)*Ki
	err_theta_p = err_theta
	return pid_theta



rospy.init_node("speed_controller_2")

sub = rospy.Subscriber("/Robot2/point_list2", Int32MultiArray, callback)
sub = rospy.Subscriber("/Robot2/odom", Odometry, newOdom)		###(topic_name, topic_type, function)
pub = rospy.Publisher("/Robot2/cmd_vel", Twist,queue_size=1)	###(topic_name, topic_type, queue_size)
#print('ros')
speed = Twist()

r = rospy.Rate(4)


while not rospy.is_shutdown():

	####  find next goal point (path) ####
	if listnum < listlength: 
		goal_x = 32 - pointList[listnum] + init_x
		goal_y = pointList[listnum+1] + init_y
	else:
		speed.linear.x = 0.0
		speed.angular.z = 0.0

	####  find yaw value ####
	inc_x = (goal_x - x)
	inc_y = (goal_y - y)
	angle_to_goal = atan2(inc_y, inc_x)
	err_theta = angle_to_goal - theta

	err_theta_d = err_theta * r2d

	if (err_theta_d<0.0):
		err_theta_d = 360.0 + err_theta_d

	pid_theta = thetaPID()

	pid_theta_d = pid_theta * r2d
	#print("listnum : ", listnum, "listlength : ", listlength)
	print('x = ',"%.2f"%x, ',goal_x = ',"%.2f"%goal_x,'---y = ',"%.2f"%y,',goal_y = ',"%.2f"%goal_y) 

	if (err_theta > 0.2 or err_theta > (pi - 0.2)):
		# print("===========")
		# print("theta ", theta)
		# print("angle_to_goal", angle_to_goal)
		#print("err_theta_d",err_theta_d)
		#print("pid_theta",pid_theta)
		speed.angular.z = pid_theta* 0.2
		speed.linear.x = 0
		#print(yaw)
	elif (listnum>=listlength and abs(inc_x)<threshold and abs(inc_y)<threshold):
		speed.linear.x = 0.0
		speed.angular.z = 0.0
	elif (abs(inc_x)<threshold and abs(inc_y)<threshold):
		listnum = listnum +2
	else:
		speed.linear.x = 0.08
		speed.angular.z = pid_theta * 0.3
	pub.publish(speed)
	r.sleep()
