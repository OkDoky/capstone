#! /usr/bin/env python

import sys
import rospy
from capstone_msgs.srv import goalPoint

def add_goal_point_client(array1):
	result = []
	rospy.wait_for_service('obstacle_point')
	try:
		add_goal_point = rospy.ServiceProxy('obstacle_point', goalPoint)
		req = goalPoint()
		req.goal_point = array1
		result = obstacle_point(req.goal_point)
		return result
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def usage():
	return "%s [x y]"%sys.argv[0]

if __name__=="__main__":
	if len(sys.argv)==3:
		x = float(sys.argv[1])
		y = float(sys.argv[2])
		array1 = [x, y]
		add_goal_point_client(array1)
		#print lists
	else:
		print usage()
		sys.exit(1)
	print "Requesting %s %s"%(x, y)
