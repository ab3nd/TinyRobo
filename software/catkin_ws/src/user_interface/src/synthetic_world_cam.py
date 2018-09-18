#!/usr/bin/python

#Connect to all the ARGOS simulated robots, and synthesize the top-down camera image of all of the robots

import rospy
import re

class ROSImageSynth(object):
	def __init__(self):
		#Subscribe to all of the robot position messages
		self.subs = {}
		self.checkSubs()
		self.topicRE = re.compile("\/bot[0-9]*\/position")

	def checkSubs(self):
		#Check for new position topics and subscribe to them
		for topic in rospy.get_published_topics()
			#If we're not already subscribed to it and it is a bot position
			if topic[0] not in self.subs.keys() and self.topicRE.match(topic[0]):
				self.subs[topic[0]] = rospy.Subscriber(topic[0], topic[1], callback = self.updateRobot, callback_args=topic[0])

	def updateRobot(self, topic, poseMsg):
		#Get the robot number out of the topic
		#Store the new pose
		print "Called updateRobot with {} for a topic".format(topic)

	def publishNewFrame(self):
		#Called by a timer, updates the image and publishes it
		pass

image_synth = ROSImageSynth()

r = rospy.Rate(30) #30hz
while not rospy.is_shutdown():
	image_synth.publishNewFrame()
	r.sleep()
