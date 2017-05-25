#!/usr/bin/python

# Distance oracle service for TinyRobos
# Gives the distance between any two april tags that it can see
# TODO what if you can't see both tags?

import rospy
from distance_oracle.srv import *

def handle_distance_req():
	pass

def init_oracle():
	rospy.init_node('distance_oracle_server')
	svc = rospy.Service('distance_oracle', DistanceOracle, handle_distance_req)
	rospy.loginfo("Distance oracle started...")
	rospy.spin()

if __name__ == "__main__":
	init_oracle()