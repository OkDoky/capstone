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

x = 0.0
y = 0.0
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
r_size_x = 8.0/1.275
r_size_y = 6.0/0.955
threshold = 0.5
yaw = 0.0
yaw_temp = 0.0
r2d = 180.0/pi
robot_path =[]
two_robot_point=[]

def change_list_to_xy(path):
	path_draw =[]
	for a in range(0,len(path),2):
		print("where?",a)
		temp =[path[a], path[a+1]]
		path_draw.append(temp)
	return path_draw

def plot_path(path, robot_path):
	global flag
	# start_x=[1,7]
	# end_y=[1,10]

	path_draw=[]
	path_draw = change_list_to_xy(path)
	data = np.array(path_draw)
	data2 = np.array(robot_path)
	bboxType = dict(boxstyle="round4", alpha=0.5, fc='w')
	arrow=dict(arrowstyle ='-|>',connectionstyle ='arc3,rad=0.2',fc="w")
	plt.title('mobile path')
	ml = MultipleLocator(1)
	Al = AutoMinorLocator(20)
	plt.xlabel('X')
	plt.ylabel('Y')
	plt.plot(*data.T)
	#plt.plot(*data2.T)
	# plt.axes().yaxis.set_major_locator(ml)
	# plt.axes().yaxis.set_minor_locator(Al)
	plt.axes().xaxis.set_major_locator(ml)
	plt.axes().xaxis.set_minor_locator(Al)

	for a in range(0,len(robot_path)):
		plt.scatter(robot_path[a][0], robot_path[a][1],s=30)
	# plt.annotate("start", xy=(1,1),xytext =(1,10), size=10, ha ='center', va='center', arrowprops=arrow, bbox=bboxType)
	# plt.annotate("end", xy=(7,10),xytext =(7,13), size=10, ha ='center', va='center', arrowprops=arrow, bbox=bboxType)
	plt.grid(which='major',color='#CCCCCC', linestyle='--')
	plt.grid(which='minor',color='#CCCCCC', linestyle=':')
	plt.show()
	flag = 3
##########  after get pointList  ##############
def newOdom (msg):
	global x
	global y
	global theta
	global pointList
	global listnum
	global init_y, init_x
	global flag
	global r2d
	global yaw, yaw_temp
	global robot_path
	global two_robot_point
	two_robot_point = [float("{0:.3f}".format(32-x)),float("{0:.3f}".format(y))]
	robot_path.append(two_robot_point)
	print(robot_path)
	if flag == 0:
		init_x = msg.pose.pose.position.x*r_size_x
		init_y = msg.pose.pose.position.y*r_size_y
		flag = 1
	elif flag ==1:
		x = msg.pose.pose.position.x*r_size_x + 32 - pointList[0]
		y = msg.pose.pose.position.y*r_size_y + pointList[1]

		rot_q = msg.pose.pose.orientation
		(roll,pitch,theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

		if(theta > pi):
			theta = theta - pi
		elif(theta < -pi):
			theta = theta + pi
		start()
	elif flag ==2:
		print('flag==2')
		file = open('/home/dodo/robot_path.txt', 'w')
		for a in range(0,len(robot_path)):
			line= "%5.2f %5.2f \n" %(robot_path[a][0],robot_path[a][1])
			file.write(line)
		file.close()
		f=open('/home/dodo/robot_path.txt','r')
		lines = f.readlines()
		# print("shit?",lines)
		plot_path(pointList, robot_path)

##########  after get pointList  ##############
def thetaPID(err_theta):
	global err_theta_p
	Kp = 1.0
	Ki = 0.05
	if (err_theta*err_theta_p<0):
		iii = err_theta + err_theta_p
	else:
		iii = err_theta - err_theta_p
	pid_theta = err_theta*Kp + iii*Ki
	err_theta_p = err_theta_p
	return pid_theta

##########  after get pointList  ##############
def distPID(err_dist):
	global err_dist_p
	Kp = 0.5
	Ki = 0.01
	Max_dist = 0.5
	Min_dist = 0.1
	iid = err_dist - err_dist_p
	pid_dist = err_dist*Kp + iid*Ki
	if pid_dist>Max_dist:
		pid_dist = Max_dist
	elif pid_dist<Min_dist:
		pid_dist = Min_dist
	err_dist_p = err_dist
	return pid_dist

##########  after get pointList  ##############
def speed_publisher(pid_dist, pid_theta, inc_x, inc_y):
	global r2d
	global flag
	global err_theta
	global threshold, listnum, listlength
	speed = Twist()
	err_theta_d = err_theta * r2d
	if ((err_theta_d > 2 and err_theta_d < 360) or (err_theta_d < -2 and err_theta_d > -360)):
		speed.linear.x = 0.0
		if(err_theta_d<0):
			speed.angular.z = pid_theta*0.31
		else:
			speed.angular.z = -pid_theta*0.31
		#print('turn , angular.z = ', pid_theta)
	elif (listnum>=listlength and abs(inc_x)<threshold and abs(inc_y)<threshold):
		speed.linear.x = 0.0
		speed.angular.x = 0.0 
		flag = 2
		print('finish', flag)
	elif(abs(inc_x)<threshold and abs(inc_y)<threshold):
		listnum = listnum + 4
	else:
		speed.linear.x = 0.05
		print('go, linear.x = ', pid_dist)
		if(err_theta_d<0):
			speed.angular.z = -pid_theta*0.2
		else:
			speed.angular.z = pid_theta*0.2
	print(speed.angular.z)
	pub.publish(speed)

##########  after get pointList  ##############
def next_goal_point():
	global listlength, listnum
	global goal_x, goal_y
	global pointList
	if listnum < listlength:
		goal_x = 32 - pointList[listnum] + init_x
		goal_y = pointList[listnum+1] + init_y

##########  after get   ##############
def callback(sf):	
	global goal_x, goal_y
	global pointList
	global listlength
	pointList = sf.data
	listlength = len(sf.data)

##########  after get goal point  ##############
def get_err_theta(inc_x, inc_y, theta):
	angle_to_goal = atan2(inc_y, inc_x)			###angle_to_goal = instance
	err_theta = angle_to_goal - theta
	return err_theta

##########  after get goal point  ##############
def get_err_dist(inc_x, inc_y):
	err_dist = sqrt(inc_x**2 + inc_y**2)
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
	# elif flag == 2:
	# 	plot_path(pointList)
	# else:
	# 	pid_theta = 0.0
	# 	pid_dist = 0.0
	# 	speed_publisher(pid_theta, pid_dist)

if __name__ == '__main__':
	rospy.init_node("speed_controller_2")

	sub = rospy.Subscriber("/Robot2/point_list2", Int32MultiArray, callback)
	sub = rospy.Subscriber("/Robot2/odom", Odometry, newOdom)		###(topic_name, topic_type, function)
	pub = rospy.Publisher("/Robot2/cmd_vel", Twist,queue_size=1)	###(topic_name, topic_type, queue_size)
	arrive = rospy.Publisher("/Robot2/arrive_flag", Int16, queue_size=1)
	r = rospy.Rate(6)
	
	r.sleep()
	rospy.spin()

