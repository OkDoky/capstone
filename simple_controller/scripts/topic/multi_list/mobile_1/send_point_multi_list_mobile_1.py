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
yaw_temp = 0.0
r2d = 180.0/pi


def newOdom (msg):
	global x
	global y
	global theta
	global pointList
	global listnum
	global init_x
	global init_y
	global flag
	global r2d
	global yaw, yaw_temp

	if flag == 0:
		init_x = msg.pose.pose.position.x*r_size_x
		init_y = msg.pose.pose.position.y*r_size_y
		flag = 1

	x=msg.pose.pose.position.x*r_size_x + 32 - pointList[0]
	y=msg.pose.pose.position.y*r_size_y + pointList[1]

	rot_q=msg.pose.pose.orientation
	(roll,pitch,theta)=euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
	# yaw = theta
	# err_yaw = (yaw - yaw_temp)*r2d
	# if(err_yaw>20):
	# 	yaw = theta - pi
	# elif(err_yaw < -20):
	# 	yaw = theta +pi
	# yaw_temp = yaw
	if(theta > pi):
		theta = theta - pi
	elif(theta < -pi):
		theta = theta + pi

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
	Kp = 1.0
	Ki = 0.005
	if (err_theta*err_theta_p<0):
		iii = err_theta + err_theta_p
	else:
		iii = err_theta - err_theta_p
	pid_theta = err_theta*Kp + iii*Ki
	err_theta_p = err_theta
	return pid_theta



rospy.init_node("speed_controller_1")

sub = rospy.Subscriber("/Robot1/point_list1", Int32MultiArray, callback)
sub = rospy.Subscriber("/Robot1/odom", Odometry, newOdom)		###(topic_name, topic_type, function)
pub = rospy.Publisher("/Robot1/cmd_vel", Twist,queue_size=1)	###(topic_name, topic_type, queue_size)
arrive = rospy.Publisher("/Robot1/arrive_flag", Int16, queue_size=1)

speed = Twist()

r = rospy.Rate(1)


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
	err_theta_plus = (angle_to_goal + theta) * r2d
	err_theta_d = err_theta * r2d
	pid_theta = thetaPID()
	# if(err_theta_d > 180):
	# 	err_theta_d = err_theta_d - 360
	# elif(err_theta_d < -180):
	# 	err_theta_d = err_theta_d + 360
	pid_theta_d = pid_theta * r2d
	print('x = ',"%.2f"%x, ',goal_x = ',"%.2f"%goal_x,'---y = ',"%.2f"%y,',goal_y = ',"%.2f"%goal_y) 

	#print("listnum : ", listnum, "listlength : ", listlength)
	#print('x = ',x, ', y = ', y)
	
	###########   err_theta too much   ##############
	# if (err_theta_d > 120):
	# 	speed.angular.z = pid_theta * 0.2
	# 	speed.linear.x = -0.005 
	
	if ((err_theta_d > 5 and err_theta_d < 180) or (err_theta_d < -5 and err_theta_d > -180)):
		print("===========")
		print("err_theta_d ", err_theta_d)
		print("pid_theta_d ", pid_theta_d)
		#print("err_theta_plus", err_theta_plus)
		speed.angular.z = pid_theta* 0.25
		speed.linear.x = 0.0
	elif (listnum>=listlength and abs(inc_x)<threshold and abs(inc_y)<threshold):
		speed.linear.x = 0.0
		speed.angular.z = 0.0
		print('final')
	elif (abs(inc_x)<threshold and abs(inc_y)<threshold):
		listnum = listnum +6
	else:
		speed.linear.x = 0.06
		speed.angular.z = pid_theta * 0.6
		print("************")
		print("theta_d : ",theta*r2d)
		print("angle_to_goal_d", angle_to_goal*r2d)
	pub.publish(speed)
	r.sleep()
