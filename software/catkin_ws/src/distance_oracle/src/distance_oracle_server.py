#!/usr/bin/python

# Distance oracle service for TinyRobos
# Gives the distance between any two april tags that it can see
# TODO what if you can't see both tags?

import rospy
from math import pow, sqrt
from distance_oracle.srv import *
from apriltags_ros.msg import *

class DistanceServer():

	def __init__(self):
		self.currentTags = {}

	def handle_distance_req(self, req):
		print self.currentTags.keys()
		if req.toID in self.currentTags.keys() and req.fromID in self.currentTags.keys():
			#We have both ends, so figure out the distance between them
			p1 = self.currentTags[req.toID].pose.position
			p2 = self.currentTags[req.fromID].pose.position
			distance = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2))
			return DistanceOracleResponse(float(distance))
		else:
			rospy.logwarn("Distance requested between {0} and {1}, but at least one of them isn't visible".format(req.toID, req.fromID))
			return DistanceOracleResponse(float('NaN'))

	def update_tags(self, msg):
		self.currentTags = {}
		for ii in range(len(msg.detections)):
			self.currentTags[int(msg.detections[ii].id)] = msg.detections[ii].pose

def init_oracle():
	ds = DistanceServer()

	rospy.init_node('distance_oracle_server')
	svc = rospy.Service('distance_oracle', DistanceOracle, ds.handle_distance_req)
	rospy.loginfo("Distance oracle started...")

	#Subscribe to april tag tag detections
	rospy.Subscriber("/tag_detections", AprilTagDetectionArray, ds.update_tags)
	rospy.spin()


if __name__ == "__main__":
	init_oracle()