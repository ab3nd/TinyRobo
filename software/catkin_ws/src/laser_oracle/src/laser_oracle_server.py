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
		#Factor for pixel/meter conversion
		self.avgPxPerM = 0
		self.image = None
		self.bridge = CvBridge()
		#Debug image publisher, to publish the results of the image pipeline
		self.dbg_pub = rospy.Publisher("/oracle_dbg/image", Image, queue_size=5)

		#TODO may want to make these rosparams
		#Parameters for the laser scan
		self.angle_min = -135 *(math.pi/180)
		#180 Degrees
		self.angle_max = 135 *(math.pi/180)
		#About 5 degrees
		self.angle_increment = 20 * (math.pi/180)
		#Time between measurements
		#TODO this might be a LOT smaller
		self.time_increment = 0.06
		#Time between scans
		#TODO may want to throttle this
		self.scan_time = 1.0

		#TODO these need to be part of a conversion from 
		#Meters to pixels
		self.range_min = 0.0
		self.range_max = 0.2

	def handle_laser_req(self, req):


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


	def distancePose(self, p1, p2):
		return (math.sqrt(
			(math.pow(p1.position.x - p2.position.x, 2)) +
			(math.pow(p1.position.y - p2.position.y, 2)) + 
			(math.pow(p1.position.z - p2.position.z, 2))
				))

	def distancePixels(self, p1, p2):
		return math.sqrt(math.pow(p1.x-p2.x,2) + math.pow(p1.y-p2.y,2))

	def update_tags(self, msg):
		self.currentTags = {}
		for ii in range(len(msg.detections)):
			self.currentTags[int(msg.detections[ii].id)] = msg.detections[ii]
		if len(self.currentTags) > 0:
			if len(self.currentTags) == 1:
				#Calculate pixels per meter from tag size
				tag = self.currentTags.values()[0]
				d = 0
				for ii in range(1, len(tag.tagCornersPx)):
					d += self.distancePixels(tag.tagCornersPx[ii], tag.tagCornersPx[ii-1])
				d = d/len(tag.tagCornersPx)
				self.avgPxPerM = d/tag.size
			else:
				#Calculate pixels per meter from inter-tag distances
				#This is pretty heavy (all pairs) and may not need to be 
				for tagA in self.currentTags.values():
					for tagB in self.currentTags.values():
						if tagA == tagB:
							continue
						else:
							dMeters = self.distancePose(tagA.pose.pose, tagB.pose.pose)
							dPx = self.distancePixels(tagA.tagCenterPx, tagB.tagCenterPx)
							self.avgPxPerM += dPx/dMeters
				self.avgPxPerM = self.avgPxPerM/(len(self.currentTags)**2)
		
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

		#Find the contours in the image, as a list
		#and compressed with chain approximation. 
		cImg, contours, hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
		self.image = cv2.drawContours(self.image, contours, -1, (0,200,200),3)


		#masked = cv2.bitwise_and(self.image, self.image, mask=mask)


		# #Testing RPY conversion from quaternions
		# for robotID in self.currentTags.keys():
		# 	w = self.currentTags[robotID].pose.pose.orientation.w
		# 	x = self.currentTags[robotID].pose.pose.orientation.x
		# 	y = self.currentTags[robotID].pose.pose.orientation.y
		# 	z = self.currentTags[robotID].pose.pose.orientation.z

		# 	(roll, pitch, yaw) = transf.euler_from_quaternion([w, x, y, z])

		# 	#Draw a line starting from the robot center pointing along the 
		# 	#roll direction, which is what this ends up calling what I'd 
		# 	#call yaw. RPY are ambiguious, nothing to be done for it.
		# 	cX = self.currentTags[robotID].tagCenterPx.x
		# 	cY = self.currentTags[robotID].tagCenterPx.y
		# 	endX = int(cX + 30*(math.cos(roll)))
		# 	endY = int(cY + 30*(math.sin(roll)))
		# 	cX = int(cX)
		# 	cY = int(cY)
		# 	cv2.line(masked, (cX, cY), (endX, endY), (0,200,200), 3)

		# 	#The laser scan lines are of the scan max distance lenght, 
		# 	#translated to pixels, and are at angles around the orientation
		# 	#of the robot
		# 	rangePx = self.avgPxPerM * self.range_max
		# 	scanCount = int((abs(self.angle_min) + abs(self.angle_max))/self.angle_increment)
		# 	for angle in np.linspace(roll + self.angle_min, roll + self.angle_max, scanCount):
		# 		cX = self.currentTags[robotID].tagCenterPx.x
		# 		cY = self.currentTags[robotID].tagCenterPx.y
		# 		endX = int(cX + rangePx*(math.cos(angle)))
		# 		endY = int(cY + rangePx*(math.sin(angle)))
		# 		cX = int(cX)
		# 		cY = int(cY)
		# 		#No need to draw the line
		# 		#cv2.line(masked, (cX, cY), (endX, endY), (0,75,200), 3)

		#Convert back for debugging
		self.image = cv2.cvtColor(self.image, cv2.COLOR_HSV2BGR)
		self.dbg_pub.publish(self.bridge.cv2_to_imgmsg(self.image, encoding="bgr8"))

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