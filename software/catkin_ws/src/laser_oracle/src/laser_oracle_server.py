#!/usr/bin/python

# Distance oracle service for TinyRobos
# Gives the distance between any two april tags that it can see
# TODO what if you can't see both tags?

import rospy
from math import pow, sqrt
from laser_oracle.srv import *
from apriltags_ros.msg import *
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan, Image
from laserscan import VirtualLaserScan
from cv_bridge import CvBridge, CvBridgeError

class LaserServer():

	def __init__(self):
		self.currentTags = {}
		self.image = None
		self.bridge = CvBridge()

	def handle_laser_req(self, req):
		#TODO may want to make these rosparams
		#Parameters for the laser scan
		angle_min = 0.0
		#180 Degrees
		angle_max = 3.1415
		#About 5 degrees
		angle_increment = 0.0872665
		#Time between measurements
		#TODO this might be a LOT smaller
		time_increment = 0.06
		#Time between scans
		#TODO may want to throttle this
		scan_time = 1
		range_min = 0.0
		range_max = None


		#Get the origin from the ID of the robot, if we can see it
		if req.robotID in self.currentTags.keys():
			x = self.currentTags[req.robotID].pose.position.x
			y = self.currentTags[req.robotID].pose.position.y				

			if self.image is not None:
				vls = VirtualLaserScan(self.image.data, angle_min, angle_max, angle_increment, (x,y), range_max)

				#Generate a laser scan message and return it
				scanMsg = LaserScan()
				h = Header()
				h.stamp = rospy.Time.now()
				scanMsg.header = h

				scanMsg.angle_min = angle_min
				scanMsg.angle_max = angle_max
				scanMsg.angle_increment = angle_increment
				scanMsg.time_increment = time_increment
				scanMsg.scan_time = scan_time
				scanMsg.range_max = range_max
				scanMsg.range_min = range_min

				scanMsg.ranges = vls.ranges()
				scanMsg.intensities = []

				return LaserOracleResponse(scanMsg)

			else:
				rospy.logwarn("No image available.")
		else:
			rospy.logwarn("Can't see robot {0}".format(req.robotID))
			print self.currentTags.keys(), req.robotID


	def update_tags(self, msg):
		self.currentTags = {}
		for ii in range(len(msg.detections)):
			self.currentTags[int(msg.detections[ii].id)] = msg.detections[ii].pose

	def update_image(self, msg):
		self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")


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