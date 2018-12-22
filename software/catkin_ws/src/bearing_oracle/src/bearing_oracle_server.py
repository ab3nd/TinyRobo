#!/usr/bin/python

# bearing oracle service for TinyRobos
# Gives the bearing between any two april tags that it can see

import rospy
from math import pow, sqrt, atan2
from bearing_oracle.srv import *
from apriltags_ros.msg import *

class BearingServer():

	def __init__(self):
		self.currentTags = {}

	def bearing(self, p1, p2):
		y = p1.position.y - p2.position.y
		x = p1.position.x - p2.position.x
		theta = atan2(y,x)
		return theta

	def handle_bearing_req(self, req):
		if req.fromID in self.currentTags.keys():
			if req.toID in self.currentTags.keys():
					#We have both ends, so figure out the bearing between them
					p1 = self.currentTags[req.toID].pose
					p2 = self.currentTags[req.fromID].pose
					bearing = self.bearing(p1, p2)
					return BearingOracleResponse(float(bearing))
			else:
				rospy.logwarn("Bearing requested to {0}, it isn't visible".format(req.toID))
				return BearingOracleResponse(float('NaN'))
		else:
			rospy.logwarn("Bearing requested from {0}, but it isn't visible".format(req.fromID))
			return BearingOracleResponse(float('NaN'))

	def update_tags(self, msg):
		self.currentTags = {}
		for ii in range(len(msg.detections)):
			self.currentTags[int(msg.detections[ii].id)] = msg.detections[ii].pose

def init_oracle():
	ds = BearingServer()

	rospy.init_node('bearing_oracle_server')
	svc = rospy.Service('bearing_oracle', BearingOracle, ds.handle_bearing_req)
	rospy.loginfo("Bearing oracle started...")

	#Subscribe to april tag tag detections
	rospy.Subscriber("/tag_detections", AprilTagDetectionArray, ds.update_tags)
	rospy.spin()


if __name__ == "__main__":
	init_oracle()