#!/usr/bin/python

# The virtual network service for TinyRobos
# MVP is just getting a message and publishing it to all the listening robots
# Next addition is only publishing within a fixed range
#   Getting range parameter (meters)
#   Doing the range math
# Next addition is stocastic network quality
#   Getting quality metric (0-1), percentage of messages that get through
#   Stochastic drop
# May want to consider having the network drop based on range, e.g. gaussian centered on robot

import rospy
from math import pow, sqrt
from net_message.srv import *
from net_message.msg import *
from apriltags_ros.msg import *

class NetworkServer():

	def __init__(self):
		self.currentTags = {}

	def handle_msg_req(self, req):
		if req.robotID in self.currentTags.keys():
			#We can see that robot, so publish a message to it
			msg = net_message()
			msg.content = req.content
			self.currentTags[int(req.robotID)].publish(msg)
			return net_messageResponse()

	def update_tags(self, msg):
		seenTags = [int(tag.id) for tag in msg.detections]
		#Remove any tags that are not visible anymore
		for oldID in self.currentTags.keys():
			if oldID not in seenTags:
				del self.currentTags[oldID]
		#For new tags, add a publisher to send messages to the robot with that tag
		for botID in seenTags:
			if botID in self.currentTags.keys():
				continue
			else:
				currentTags[botID] = rospy.Publisher("/messages_to_{0}".format(botID), net_message, 1)



def init_network():
	ns = NetworkServer()

	rospy.init_node('virtual_network_server')
	svc = rospy.Service('network_service', NetworkServer, ns.handle_msg_req)
	rospy.loginfo("Network service started...")

	#Subscribe to april tag tag detections
	rospy.Subscriber("/tag_detections", AprilTagDetectionArray, ds.update_tags)
	rospy.spin()


if __name__ == "__main__":
	init_oracle()