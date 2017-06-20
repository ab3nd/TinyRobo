#!/usr/bin/python

# Distance oracle service for TinyRobos
# Gives the distance between any two april tags that it can see
# TODO what if you can't see both tags?

import rospy
from math import pow, sqrt
from laser_oracle.srv import *
from apriltags_ros.msg import *
from sensor_msgs.msg import LaserScan, Image

class LaserServer():

	def __init__(self):
		self.currentTags = {}
		self.image = None

	def handle_laser_req(self, req):
		rospy.logwarn("LEL NOT IMPLEMENTED")
		return False

	def update_tags(self, msg):
		self.currentTags = {}
		for ii in range(len(msg.detections)):
			self.currentTags[int(msg.detections[ii].id)] = msg.detections[ii].pose

	def update_image(self, msg):
		self.image = msg

def init_oracle():
	ls = LaserServer()

	rospy.init_node('laser_oracle_server')
	svc = rospy.Service('laser_oracle', LaserOracle, ls.handle_laser_req)
	rospy.loginfo("Laser oracle started...")

	#Subscribe to april tag tag detections
	rospy.Subscriber("/tag_detections", AprilTagDetectionArray, ls.update_tags)

	#Subscribe to images from the overhead camera
	rospy.Subscriber("/overhead_camera/image_rect", Image, ls.update_image)
	rospy.spin()


if __name__ == "__main__":
	init_oracle()