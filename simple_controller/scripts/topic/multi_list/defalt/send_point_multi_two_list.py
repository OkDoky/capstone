#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, sqrt
from capstone_msgs.msg import Smartfactory
from std_msgs.msg import Int32MultiArray

x=0.0
y=0.0
theta=0.0

goal_x = 0.0
goal_y = 0.0
listnum = 0
listlength = 0
pointList = 0
first_x = 0.0
first_y = 0.0
err_theta = 0.0
err_theta_p = 0.0

def newOdom (msg):
	global x
	global y
	global theta
	global pointList

	x=msg.pose.pose.position.x + pointList[0]/100.0
	y=msg.pose.pose.position.y + pointList[1]/100.0
	
	rot_q=msg.pose.pose.orientation
	(roll,pitch,theta)=euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


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
	Kp = 1.2
	Ki = 0.005
	pid_theta = err_theta*Kp + (err_theta-err_theta_p)*Ki
	err_theta_p = err_theta
	return pid_theta



rospy.init_node("speed_controller")

sub = rospy.Subscriber("/point_list", Int32MultiArray, callback)
sub = rospy.Subscriber("/odom", Odometry, newOdom)		###(topic_name, topic_type, function)
pub = rospy.Publisher("/cmd_vel", Twist,queue_size=1)	###(topic_name, topic_type, queue_size)

speed = Twist()

r = rospy.Rate(4)



while not rospy.is_shutdown():
	if listnum < listlength:
		goal_x = pointList[listnum]/100.0
		goal_y = pointList[listnum+1]/100.0

	else:
		speed.linear.x = 0.0
		speed.angular.z = 0.0

	inc_x = (goal_x - x)
	inc_y = (goal_y - y)
	angle_to_goal = atan2(inc_y, inc_x)
	err_theta = angle_to_goal - theta
		
	pid_theta = thetaPID()
	print("listnum : ", listnum, "listlepngth : ", listlength)
	if (err_theta > 0.3):
		speed.angular.z = pid_theta* 0.2
		speed.linear.x = 0
		
	elif (listnum>=listlength and abs(inc_x)<0.05 and abs(inc_y)<0.05):
		speed.linear.x = 0.0
		speed.angular.z = 0.0
	elif (abs(inc_x)<0.05 and abs(inc_y)<0.05):
		listnum = listnum +8
	else:
		speed.linear.x = 0.08
		print(inc_x, ", ", inc_y) 
		speed.angular.z = pid_theta * 0.5

	pub.publish(speed)
	r.sleep()
