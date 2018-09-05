#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist, PointStamped
from apriltags_ros.msg import *
import random
import math
from tf import transformations as trans

from transitions import Machine


def dot(a, b):
	assert len(a) == len(b)
	d = 0
	for i, j in zip(a, b):
		d += i*j
	return d

class Point_Converter():
	def __init__(self, robot = 8):
		
		self.robot_id = robot

		self.currentTime = None
		self.currentX = 0.0
		self.currentY = 0.0
		self.currentZ = 0.0
		
		self.prevTime = None
		self.prevX = 0.0
		self.prevY = 0.0
		self.prevZ = 0.0

		all_tags = rospy.get_param('/apriltag_detector/tag_descriptions')
		self.tagsize = 0.051 #default, but we try to update it
		for tag in all_tags:
			if tag['id'] == self.robot_id:
				self.tagsize = tag['size']

		self.dist = 0
		self.vel = 0
		self.travelIsValid = false

	def update_tags(self, tags_msg):
		#Get the location of this robot from the tag
		tag = None
		try:
			tag = [x for x in tags_msg.detections if x.id == self.robot_id][0]
		except IndexError as ie:
			#This is caused by there not being a detection of the tag in this set of 
			#tag detections
			rospy.logwarn("Didn't see tag {0} in this frame".format(self.robot_id))
			return 

		#If we already have tags, calculate the distance moved and velocity moved
		if self.currentTime is None:
			self.currentTime = tag.pose.header.stamp
			self.currentX = tag.pose.pose.position.x 
			self.currentY = tag.pose.pose.position.y 
			self.currentZ = tag.pose.pose.position.z
			self.startX = self.currentX
			self.startY = self.currentY
		else:
			#Roll over the current and previous positions
			self.prevTime = self.currentTime
			self.prevX = self.currentX
			self.prevY = self.currentY
			self.prevZ = self.currentZ

			#Update from the tag
			self.currentTime = tag.pose.header.stamp
			self.currentX = tag.pose.pose.position.x 
			self.currentY = tag.pose.pose.position.y 
			self.currentZ = tag.pose.pose.position.z

			#Now we have two poses, so we can measure travel between them
			self.travelIsValid = True

			#Calculate the distance traveled
			self.dist = math.sqrt(sum([pow(x - y, 2) for x,y in [(self.prevX, self.currentX),(self.prevY, self.currentY)]]))
			#Distance is in meters, this is in m/sec
			self.vel = dist/((self.currentTime - self.prevTime).to_sec())
			rospy.logwarn("dist: {0} vel: {1}".format(self.dist, self.vel))

			#TODO what is the velocity noise floor? 

	def get_travel():
		if not self.travelIsValid:
			rospy.logerr("Requested travel but it's not valid")
			return None
		else:
			#Get the distance from start to here
			d = math.sqrt(sum([pow(x - y, 2) for x,y in [(self.startX, self.currentX),(self.startY, self.currentY)]]))
			return d

	def clear_travel():
		#Reset so travel is measured from where we are now
		self.startX = self.currentX
		self.startY = self.currentY

		
if __name__ == '__main__':
	rospy.init_node('point_driver', anonymous=True)

	robot_id = rospy.get_param("/robot_id")
	pc = Point_Converter(robot=robot_id)



	loc_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, pc.update_tags)
	go_sub = rospy.Subscriber('/touches', PointStamped, pc.update_target)

	while not rospy.is_shutdown():
		while pc.vel <= 0:
			pass


	
