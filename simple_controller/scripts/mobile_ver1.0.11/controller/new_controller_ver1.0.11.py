#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import *
from std_msgs.msg import Int32MultiArray, Int16
from matplotlib import pyplot as plt
from matplotlib.ticker import (AutoMinorLocator,MultipleLocator)
import numpy as np
import sys
import time

x = 0.0
y = 0.0
size_x = 160.0
size_y = 122.0
theta = 0.0
flag = 0
goal_x = 0.0
goal_y = 0.0
init_x = 0.0
init_y = 0.0
listnum = 0
listlength = 0
pointList = 0
err_theta = 0.0
err_theta_p = 0.0
err_dist = 0.0
err_dist_p = 0.0
r_size_x = size_x/2.095
r_size_y = size_y/1.585
threshold = 3.5
yaw = 0.0
yaw_temp = 0.0
r2d = 180.0/pi
robot_path = []
two_robot_point = []
err_dist_list = []
goal_angle_list = []
theta_list = []
time_flag = 0
start_time = 0
goal2_theta = 0
push_dist = 0.0
second_x = 0.0
second_y = 0.0
other_dist = 0.0
push_flag = 0
arrive_flag = 0
data_flag = 0
temp1_y = 0.0
temp2_y = 0.0
temp1_x = 0.0 
temp2_x = 0.0

def change_list_to_xy(path):
	path_draw = []
	for a in range(0,len(path),2):
		temp = [path[a], path[a+1]]
		path_draw.append(temp)
	return path_draw

def plot_path(path, robot_path):
	global flag
	global theta_list
	global goal_angle_list
	global err_dist_list
	fig = plt.figure()
	area1 = fig.add_subplot(2,2,1)
	area2 = fig.add_subplot(2,2,2)
	area3 = fig.add_subplot(2,2,3)
	area4 = fig.add_subplot(2,2,4)
	path_draw=[]
	path_draw = change_list_to_xy(path)
	data = np.array(path_draw)
	data2 = np.array(robot_path)
	area1.set_title('green - compare real path and path list')
	ml = MultipleLocator(5)
	Al = AutoMinorLocator(10)
	area1.set_xlabel('X')
	area1.set_ylabel('Y')
	area1.plot(*data.T)

	for a in range(0,len(robot_path)):
		area1.scatter(robot_path[a][0], robot_path[a][1],s=10)

	area2.set_title('error distance')
	area2.set_xlabel('time')
	area2.set_ylabel('err dist')
	for a in range(0, len(err_dist_list)):
		area2.scatter(a/10.0,err_dist_list[a],s=10)

	area3.set_title('angle to goal')
	area3.set_xlabel('time')
	area3.set_ylabel('goal angle')
	for a in range(0, len(goal_angle_list)):
		area3.scatter(a/10.0,goal_angle_list[a],s=10)

	area4.set_title('theta')
	area4.set_xlabel('time')
	area4.set_ylabel('theta')
	for a in range(0, len(theta_list)):
		area4.scatter(a/10.0,theta_list[a],s=10)

	area1.grid(which='major',color='#CCCCCC', linestyle='--')
	area1.grid(which='minor',color='#CCCCCC', linestyle=':')
	area2.grid(which='major',color='#CCCCCC', linestyle='--')
	area2.grid(which='minor',color='#CCCCCC', linestyle=':')
	area3.grid(which='major',color='#CCCCCC', linestyle='--')
	area3.grid(which='minor',color='#CCCCCC', linestyle=':')
	area4.grid(which='major',color='#CCCCCC', linestyle='--')
	area4.grid(which='minor',color='#CCCCCC', linestyle=':')

	plt.show()
##########  after get pointList  ##############
def newOdom (msg):
	global x, y, second_y, second_x, temp1_x, temp2_x, temp1_y, temp2_y
	global theta
	global pointList
	global listnum, listlength
	global init_y, init_x
	global flag
	global r2d
	global yaw, yaw_temp
	global robot_path
	global two_robot_point
	global time_flag, data_flag
	global start_time
	global size_x, arrive_flag, err_dist_list, goal_angle_list, theta_list
	global goal2_theta, push_dist, push_flag, other_dist
	two_robot_point = [float("{0:.3f}".format(size_x-x)),float("{0:.3f}".format(y))]
	robot_path.append(two_robot_point)
	speed = Twist()
	if flag == 0 and data_flag == 1:
		init_x = msg.pose.pose.position.x*r_size_x
		init_y = msg.pose.pose.position.y*r_size_y
		temp1_x = init_x
		temp2_x = init_x
		temp1_y = init_y
		temp2_y = init_y
		flag = 1
	elif flag ==1 or flag == 2 or flag == 3:
		if time_flag == 0:
			start_time = rospy.Time.now()
			print("start_time : ",start_time)
			time_flag = 1
		x = msg.pose.pose.position.x*r_size_x + size_x - pointList[0]
		y = msg.pose.pose.position.y*r_size_y + pointList[1]
		temp2_x = temp1_x
		temp1_x = x
		x = (temp1_x + temp2_x)/2.0
		temp2_y = temp1_y
		temp1_y = y
		y =  (temp1_y + temp2_y)/2.0
		rot_q = msg.pose.pose.orientation
		(roll,pitch,theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

		if(theta > pi):
			theta = theta - 2.0*pi
		elif(theta < -pi):
			theta = theta + 2.0*pi
		if flag == 1 or flag == 2:
			start()

	if listnum == listlength and flag == 2:
		err_theta = get_last_err_theta(goal2_theta/r2d)
		err_theta_d = err_theta * r2d
		pid_theta = thetaPID(err_theta)
		if ((err_theta_d > 3 and err_theta_d < 360) or (err_theta_d < -3 and err_theta_d > -360)):
			speed.linear.x = 0.0
			if(err_theta_d<0):
				speed.angular.z = -pid_theta
			else:
				speed.angular.z = pid_theta
		else:
			speed.angular.z = 0.0
			finish_time = rospy.Time.now()
			print('finish time : ', finish_time)
			print((finish_time - start_time)/1000000000.0)
			second_x = x
			second_y = y
			flag = 3
		pub.publish(speed)

	elif flag == 3 and push_flag == 1 and arrive_flag == 1:
		other_dist = forword_dist(push_dist)
		up_down = direction()
		if up_down =="foward":
			if other_dist <= 1.2:
				speed.angular.z = 0.0
				speed.linear.x = 0.0
				flag = 4
			else:
				speed.angular.z = 0.0
				speed.linear.x = other_dist * 0.01
		else:
			if other_dist <= 1.2:
				speed.angular.z = 0.0
				speed.linear.x = 0.0
				flag = 4
			else:
				speed.angular.z = 0.0
				speed.linear.x = -1*other_dist * 0.01
		pub.publish(speed)
	elif flag == 4:
		file = open('/home/dodo/robot1_path.txt', 'w')
		for a in range(0,len(robot_path)):
			line= "%5.2f %5.2f \n" %(robot_path[a][0],robot_path[a][1])
			file.write(line)
		file.close()
		f=open('/home/dodo/robot1_path.txt','r')
		lines = f.readlines()
		plot_path(pointList, robot_path)

		two_robot_point = []
		err_dist_list = []
		goal_angle_list = []
		theta_list = []
		pointList = 0
		data_flag = 0
		listnum = 0
		flag = 0
		arrive_flag = 0
		push_flag = 0
	arrive.publish(flag)

##########  after get pointList  ##############
def thetaPID(err_theta):
	global err_theta_p
	Kp = 1.0
	Ki = 0.001
	iii = err_theta - err_theta_p
	pid_theta = err_theta*Kp + iii*Ki
	err_theta_p = err_theta
	return pid_theta

##########  after get pointList  ##############
def distPID(err_dist):
	global err_dist_p
	Kp = 0.0014
	Ki = 0.0002
	Max_dist = 0.1
	Min_dist = 0.005
	iid = err_dist - err_dist_p
	pid_dist = err_dist*Kp + iid*Ki
	if pid_dist>Max_dist:
		pid_dist = Max_dist
	elif pid_dist<Min_dist:
		pid_dist = Min_dist
	err_dist_p = err_dist
	return pid_dist

##########  after get pointList  ##############
def speed_publisher(pid_theta, pid_dist, inc_x, inc_y):
	global r2d
	global flag
	global err_theta
	global threshold, listnum, listlength
	speed = Twist()
	err_theta_d = err_theta * r2d
	if ((err_theta_d > 3 and err_theta_d < 360) or (err_theta_d < -3 and err_theta_d > -360)):
		speed.linear.x = 0.0
		speed.angular.z = pid_theta
	elif (listnum==listlength and abs(inc_x)<threshold and abs(inc_y)<threshold):
		speed.linear.x = 0.0
		speed.angular.x = 0.0 
		print('finish')
		flag = 2
	elif(listnum<listlength and abs(inc_x)<threshold and abs(inc_y)<threshold):
		listnum = listnum + 2
	else:
		speed.linear.x = 0.1
		speed.angular.z = pid_theta
		print('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
	pub.publish(speed)

##########  after get pointList  ##############
def next_goal_point():
	global listlength, listnum
	global goal_x, goal_y
	global pointList
	global size_x
	if listnum < listlength:
		goal_x = size_x - pointList[listnum] + init_x
		goal_y = pointList[listnum+1] + init_y

##########  after get   ##############
def callback(sf):	
	global goal_x, goal_y
	global pointList
	global listlength, data_flag
	pointList = sf.data
	listlength = len(sf.data)
	data_flag = 1

##########  after get goal point  ##############
def get_err_theta(inc_x, inc_y, theta):
	global goal_angle_list
	global theta_list
	global r2d
	angle_to_goal = atan2(inc_y, inc_x)			###angle_to_goal = instance
	err_theta = angle_to_goal - theta
	print('atg_theta : ', angle_to_goal, 'theta : ', theta)
	goal_angle_list.append(angle_to_goal)
	theta_list.append(theta)
	return err_theta

##########  after get goal point  ##############
def get_err_dist(inc_x, inc_y):
	global err_dist_list
	err_dist = sqrt(inc_x**2 + inc_y**2)
	err_dist_list.append(err_dist)
	return err_dist

def start():
	global r2d
	global goal_x, goal_y
	global x,y
	global inc_x, inc_y 
	global init_x, init_y
	global pointList
	global listlength, listnum
	global flag
	global theta
	global threshold
	global err_theta, err_dist

	if flag == 1:
		next_goal_point()
		inc_x = goal_x - x 
		inc_y = goal_y - y
		err_theta = get_err_theta(inc_x, inc_y, theta)
		err_dist = get_err_dist(inc_x, inc_y)
		pid_theta = thetaPID(err_theta)
		pid_dist = distPID(err_dist)
		speed_publisher(pid_theta, pid_dist, inc_x, inc_y)

def callback2(msg):
	global goal_x, goal_y
	global pointList
	global listlength
	global flag
	pointList = sf.data
	listlength = len(sf.data)
	print('im in callback2')

def both_arrive(msg):
	global flag, listnum
	global time_flag, arrive_flag
	if msg.data == 3 and flag ==3:
		arrive_flag = 1
	elif flag == 3:
		sub = rospy.Subscriber("/Robot2/next_point2", Int32MultiArray, callback2)

##########  after get goal point  ##############
def get_last_err_theta(goal_theta):
	global r2d
	global theta
	angle_to_goal = goal_theta		###angle_to_goal = instance
	err_theta = angle_to_goal - theta
	return err_theta

def last_robot_angle(msg):
	global listnum, listlength
	global flag, r2d
	global goal2_theta
	goal2_theta = msg.data 

def forword_dist(push_dist):
	global x, y, second_x, second_y
	inc_x = x - second_x
	inc_y = y - second_y
	integral_dist = get_err_dist(inc_x, inc_y)
	other_dist = push_dist - integral_dist
	return other_dist

def direction():
	global pointList, listlength
	end_point = pointList[listlength-1]
	width = size_y
	if end_point > width/2:
		return "backward"
	else:
		return "foward"

def push(msg):
	global push_dist, push_flag
	push_dist = msg.data
	push_flag = 1

if __name__ == '__main__':
	rospy.init_node("speed_controller_1")

	sub = rospy.Subscriber("/Robot1/point_list1", Int32MultiArray, callback)
	sub = rospy.Subscriber("/Robot1/odom", Odometry, newOdom)		###(topic_name, topic_type, function)
	sub = rospy.Subscriber("/Robot2/arrive_flag", Int16, both_arrive)
	sub = rospy.Subscriber("/obstacle/angle", Int16, last_robot_angle)
	sub = rospy.Subscriber("/push",Int16, push)
	pub = rospy.Publisher("/Robot1/cmd_vel", Twist,queue_size=1)	###(topic_name, topic_type, queue_size)
	arrive = rospy.Publisher("/Robot1/arrive_flag", Int16, queue_size=1)
	r = rospy.Rate(6)
	r.sleep()
	rospy.spin()

