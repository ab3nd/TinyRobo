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
import cv2
import numpy as np
import math
from tf import transformations as transf

lower_blue, upper_blue = np.array([100,70,30]), np.array([130,255,255])
kernel = np.ones([3,3], np.uint8)

class LaserServer():

	def __init__(self):
		self.currentTags = {}
		self.image = None
		self.bridge = CvBridge()
		#Debug image publisher, to publish the results of the image pipeline
		self.dbg_pub = rospy.Publisher("/oracle_dbg/image", Image, queue_size=5)


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
		scan_time = 1.0
		range_min = 0.0
		range_max = 100.0


		#Get the origin from the ID of the robot, if we can see it
		if req.robotID in self.currentTags.keys():
			#This is in physical coordinates
			#x = self.currentTags[req.robotID].pose.position.x
			#y = self.currentTags[req.robotID].pose.position.y				

			x=self.currentTags[req.robotID].tagCenterPx.x
			y=self.currentTags[req.robotID].tagCenterPx.y

			if self.image is not None:
				vls = VirtualLaserScan(self.image, angle_min, angle_max, angle_increment, (x,y), range_max)

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

				scanMsg.ranges = vls.ranges
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
			self.currentTags[int(msg.detections[ii].id)] = msg.detections[ii]

	def update_image(self, msg):
		self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
		self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

		#Draw a blue circle for each robot so they avoid each other
		for robotID in self.currentTags.keys():
			x=int(self.currentTags[robotID].tagCenterPx.x)
			y=int(self.currentTags[robotID].tagCenterPx.y)
			robotRad = 30
			cv2.circle(self.image, (x,y), robotRad, np.array([115,255,255]), -1)
			cv2.putText(self.image, str(robotID), (x-robotRad/2, y), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.3, np.array([115,255,30]))
		#B&W image that is white for every pixel in the range
		mask = cv2.inRange(self.image, lower_blue, upper_blue)
		#Erode the mask to remove little noise pixels
		mask = cv2.erode(mask, kernel, iterations=1)
		masked = cv2.bitwise_and(self.image, self.image, mask=mask)


		#Testing RPY conversion from quaternions
		for robotID in self.currentTags.keys():
			w = self.currentTags[robotID].pose.pose.orientation.w
			x = self.currentTags[robotID].pose.pose.orientation.x
			y = self.currentTags[robotID].pose.pose.orientation.y
			z = self.currentTags[robotID].pose.pose.orientation.z

			#yaw = math.atan2(2*((w*y) + (y*z)), 1-2*(math.pow(x,2) - math.pow(y,2)))
			#pitch = math.asin(2*((w*y) + (z*y)))
			#roll = math.atan2(2*((w*z) + (x*y)), 1-2*(math.pow(y,2) - math.pow(z,2)))

			# roll  = math.atan2(2*y*w + 2*x*z, 1 - 2*y*y - 2*z*z);
			# pitch = math.atan2(2*x*w + 2*y*z, 1 - 2*x*x - 2*z*z);
			# yaw   = math.asin(2*x*y + 2*z*w);
			(roll, pitch, yaw) = transf.euler_from_quaternion([w, x, y, z])
			#print roll, pitch, yaw
			#Draw a line starting from the robot center
			# pointing along the yaw direction

			cX = self.currentTags[robotID].tagCenterPx.x
			cY = self.currentTags[robotID].tagCenterPx.y
			endX = int(cX + 30*(math.cos(pitch)))
			endY = int(cY + 30*(math.sin(pitch)))

			# cR = math.sqrt(math.pow(cX, 2) + math.pow(cY,2))
			# cTheta = math.atan2(cY, cX)

			# #Now the other endpoint is a fixed distance and angle from that
			# endR = cR + 20
			# endTheta = yaw

			#convert back
			# endX = int(endR * math.cos(endTheta))
			# endY = int(endR * math.sin(endTheta))
			cX = int(cX)
			cY = int(cY)

			cv2.line(masked, (cX, cY), (endX, endY), (0,200,200), 3)

		#Convert back for debugging
		masked = cv2.cvtColor(masked, cv2.COLOR_HSV2BGR)
		self.dbg_pub.publish(self.bridge.cv2_to_imgmsg(masked, encoding="bgr8"))

def init_oracle():
	ls = LaserServer()

	rospy.init_node('laser_oracle_server')
	svc = rospy.Service('laser_oracle', LaserOracle, ls.handle_laser_req)
	rospy.loginfo("Laser oracle started...")

	#Subscribe to april tag tag detections
	rospy.Subscriber("/tag_detections", AprilTagDetectionArray, ls.update_tags)

	#Subscribe to images from the overhead camera
	rospy.Subscriber("/overhead_camera/image_rect_color", Image, ls.update_image)
	rospy.spin()


if __name__ == "__main__":
	init_oracle()