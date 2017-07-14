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
from network_service.srv import *
from network_service.msg import *
from distance_oracle.srv import *
from apriltags_ros.msg import *

class NetworkServer():

	def __init__(self, maxTxRange):
		self.currentTags = {}
		if(maxTxRange > 0):
			#Distance service needed to determine reachability based on range
			self.maxTxRange = maxTxRange
			rospy.wait_for_service('distance_oracle')
			self.getDistance = rospy.ServiceProxy('distance_oracle', DistanceOracle)
		else:
			self.getDistance = None

	def is_reachable(self, toID, fromID):
		#We don't care about distance, none was set
		if self.getDistance == None:
			rospy.logwarn("Always sending")
			return True
		else:
			#request distance from distance oracle and check that we're not over it
			distance = self.getDistance(toID, fromID)
			#rospy.logwarn("{0}, {1}".format(distance, self.maxTxRange))
			if distance.distance > self.maxTxRange:
				return False
			return True

	def handle_msg_req(self, req):
		if req.robotID in self.currentTags.keys():
			if self.is_reachable(req.robotID, req.fromID):
				#We can reach that robot over the network, so publish a message to it
				msg = NetMsgToRobot()
				msg.content = req.content
				msg.fromID = req.fromID
				self.currentTags[int(req.robotID)].publish(msg)
		return NetMsgResponse()

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
				self.currentTags[botID] = rospy.Publisher("/messages_to_{0}".format(botID), NetMsgToRobot, queue_size=5)



def init_network():
	
	rospy.init_node('virtual_network_server')

	#Get the maximum transmission range for the network
	#Default value of -1 means any range is ok
	maxTx = rospy.get_param("~txRange", -1.0)
	ns = NetworkServer(maxTxRange = maxTx)

	svc = rospy.Service('network_service', NetMsg, ns.handle_msg_req)
	rospy.loginfo("Network service started...")

	#Subscribe to april tag tag detections
	rospy.Subscriber("/tag_detections", AprilTagDetectionArray, ns.update_tags)
	rospy.spin()


if __name__ == "__main__":
	init_network()