#!/usr/bin/python

# Laser oracle service for TinyRobos
# Creates laser rangefindings for a given robot ID

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
import array
import time
from tf import transformations as transf

import pyopencl as cl
#For debugging
import random

lower_blue, upper_blue = np.array([100,70,30]), np.array([130,255,255])
kernel = np.ones([3,3], np.uint8)
class LaserServer():

	def __init__(self):
		self.ctx = cl.create_some_context()
		self.currentTags = {}
		#Factor for pixel/meter conversion
		self.avgPxPerM = 0
		self.image = None
		self.bridge = CvBridge()
		#Debug image publisher, to publish the results of the image pipeline
		#self.dbg_pub = rospy.Publisher("/oracle_dbg/image", Image, queue_size=5)

		#Most of the parameters of the laser scan are passed in the
		#request. The requester should fill these two, since it knows
		#how often it is making requests.
		self.time_increment = 0.0
		self.scan_time = 0.0

		#These are in meters, but get converted to pixels for
		#the virtual laser scans
		#self.range_min = 0.0
		#self.range_max = 0.2

		#Radius of robot, in pixels, in the image
		#used to draw a blue dot over all robots for collision avoidance
		self.robotRad = 30

	def isReady(self):
		#Ready if we have an image to work with and some tags in it
		return self.image is not None and len(self.currentTags.keys()) > 0

	def handle_laser_req(self, req):
		#Get the origin from the ID of the robot, if we can see it
		if req.robotID in self.currentTags.keys() and self.image is not None:
			#start=time.time()
			#Use a copy of the existing image, not the original, or else
			#you get threading-related issues
			imgCpy = self.image.copy()

			#Pixel coordinates of robot in image
			cX=self.currentTags[req.robotID].tagCenterPx.x
			cY=self.currentTags[req.robotID].tagCenterPx.y

			#The laser scan lines are from the min to max scan distance,
			#translated to pixels, and are at angles around the orientation
			#of the robot
			maxRangePx = self.avgPxPerM * req.rangeMax
			#Minimum scan must be outside of robot's radius
			minRangePx = max(self.robotRad + 1, self.avgPxPerM * req.rangeMin)

			#Mask off everything in the image that's not within the
			#laser range of the robot
			mask = np.zeros(imgCpy.shape, dtype=np.uint8)
			cv2.circle(mask, (int(cX), int(cY)), int(maxRangePx) + 5, 255, -1)
			masked = cv2.bitwise_and(imgCpy, mask)

			#Find the contours in the image, as a list
			#and compressed with chain approximation.
			#cImg, contours, hierarchy = cv2.findContours(masked, cv2.RETR_LIST, cv2.CHAIN_APPROX_TC89_KCOS)#, cv2.CHAIN_APPROX_SIMPLE)
			cImg,contours,hierarchy = cv2.findContours(masked, cv2.RETR_LIST, cv2.CHAIN_APPROX_TC89_KCOS)#, cv2.CHAIN_APPROX_SIMPLE)

			#approximate each contour with a polygon approximation
			approximations = []
			for index, contour in enumerate(contours):
				isClosed = (hierarchy[0][index][2] < 0) and (hierarchy[0][index][3] < 0)
				epsilon = 0.01 * cv2.arcLength(contour, isClosed)
				approx = cv2.approxPolyDP(contour, epsilon, isClosed)
				approximations.append(approx)

			"""#For debugging reasons, try drawing the approximations on the image
			colorImg = cv2.cvtColor(imgCpy, cv2.COLOR_GRAY2BGR)
			for approx in approximations:
				#Pick a random color
				color = (random.randint(80,255), random.randint(80,255), random.randint(80,255))
				for index in range(len(approx) -1):
					#Draw a line from this point to the next one
					p1 = (approx[index][0][0], approx[index][0][1])
					p2 = (approx[index+1][0][0], approx[index+1][0][1])
					colorImg = cv2.line(colorImg, p1, p2, color, thickness = 3)
			self.dbg_pub.publish(self.bridge.cv2_to_imgmsg(colorImg, "bgr8"))"""

			#Convert robot quaternion to RPY, then check the line segment from the center
			#of each robot to the max range for intersection with a segment of a contour,
			#and find the minimum-distance intersection. That minimum-distance intersection
			#is the laser range for that robot and that scan.
			w = self.currentTags[req.robotID].pose.pose.orientation.w
			x = self.currentTags[req.robotID].pose.pose.orientation.x
			y = self.currentTags[req.robotID].pose.pose.orientation.y
			z = self.currentTags[req.robotID].pose.pose.orientation.z

			#The roll direction is what I'd call yaw.
			#RPY are ambiguious, nothing to be done for it.
			(roll, pitch, yaw) = transf.euler_from_quaternion([w, x, y, z])

			#Calculate the number of scans
			scanCount = int((abs(req.angleMin) + abs(req.angleMax))/req.angleIncrement)

			#This evenly spaces the scans, which may be a little off from the behavior of
			#a real laser scanner. Note that this assumes self.angle_min is negative, so
			#the scan is centered on the robot's heading
			scan = []
			line_segments_x = []
			line_segments_y = []
			for approx in approximations:
				for pointIdx in range(len(approx) - 1):
					line_segments_x.append(approx[pointIdx][0][0])
					line_segments_y.append(approx[pointIdx][0][1])
					line_segments_x.append(approx[pointIdx+1][0][0])
					line_segments_y.append(approx[pointIdx+1][0][1])
			queue = cl.CommandQueue(self.ctx)
			mf = cl.mem_flags
			scan_angle_buffer = cl.Buffer(self.ctx, mf.READ_ONLY | mf.COPY_HOST_PTR, hostbuf=memoryview(array.array("f",[i for i in np.linspace(roll + req.angleMin, roll + req.angleMax, scanCount)]).tostring()))
			line_segments_x_buffer = cl.Buffer(self.ctx, mf.READ_ONLY | mf.COPY_HOST_PTR, hostbuf=memoryview(array.array("f",line_segments_x).tostring()))
			line_segments_y_buffer = cl.Buffer(self.ctx, mf.READ_ONLY | mf.COPY_HOST_PTR, hostbuf=memoryview(array.array("f",line_segments_y).tostring()))
			ray_lengths = np.zeros(scanCount,np.float32)
			ray_lengths_buffer = cl.Buffer(self.ctx, mf.WRITE_ONLY, ray_lengths.nbytes)
			prg = cl.Program(self.ctx, """
			__kernel void raycast(
				__global const float *scan_angles, __global const float *line_segments_x, __global const float *line_segments_y, __global float *ray_lengths,
				float center_x, float center_y, float min_range, float max_range, int line_segment_count)
			{
				int gid = get_global_id(0);
				float current_min_distance = max_range;
				float start_x = center_x + min_range*cos((scan_angles[gid]));
				float start_y = center_y + min_range*sin((scan_angles[gid]));
				float end_x = center_x + max_range*cos((scan_angles[gid]));
				float end_y = center_y + max_range*sin((scan_angles[gid]));
				float min_x = start_x<end_x?start_x:end_x;
				float max_x = start_x>end_x?start_x:end_x;
				float min_y = start_y<end_y?start_y:end_y;
				float max_y = start_y>end_y?start_y:end_y;
				int i=0;
				while(i<line_segment_count){
					float seg_start_x=line_segments_x[2*i];
					float seg_start_y=line_segments_y[2*i];
					float seg_end_x=line_segments_x[2*i+1];
					float seg_end_y=line_segments_y[2*i+1];
					float seg_min_x = seg_start_x<seg_end_x?seg_start_x:seg_end_x;
					float seg_max_x = seg_start_x>seg_end_x?seg_start_x:seg_end_x;
					float seg_min_y = seg_start_y<seg_end_y?seg_start_y:seg_end_y;
					float seg_max_y = seg_start_y>seg_end_y?seg_start_y:seg_end_y;
					float den = (start_x-end_x)*(seg_start_y-seg_end_y)-(start_y-end_y)*(seg_start_x-seg_end_x);
					if(den == 0){
						i++;
						continue;
					}
					float intersect_x = (((start_x*end_y)-(start_y*end_x))*(seg_start_x-seg_end_x)-(start_x-end_x)*((seg_start_x*seg_end_y)-(seg_start_y*seg_end_x)))/den;
					float intersect_y = (((start_x*end_y)-(start_y*end_x))*(seg_start_y-seg_end_y)-(start_y-end_y)*((seg_start_x*seg_end_y)-(seg_start_y*seg_end_x)))/den;
					if(
					(intersect_x>=min_x &&
					intersect_x<=max_x &&

					intersect_y>=min_y &&
					intersect_y<=max_y &&

					intersect_x>=seg_min_x &&
					intersect_x<=seg_max_x &&

					intersect_y>=seg_min_y &&
					intersect_y<=seg_max_y)
					){
						float dist = sqrt(pown(intersect_x-center_x,2)+pown(intersect_y-center_y,2));
						if(dist<current_min_distance ){
							current_min_distance=dist;
						}
					}
					i++;
				}
				ray_lengths[gid]=current_min_distance;
			}
			""").build()

			prg.raycast(queue, ray_lengths.shape, None, scan_angle_buffer, line_segments_x_buffer, line_segments_y_buffer, ray_lengths_buffer, np.float32(cX), np.float32(cY), np.float32(minRangePx), np.float32(maxRangePx), np.int32(len(line_segments_x)/2))
			cl.enqueue_copy(queue,ray_lengths,ray_lengths_buffer)
			#Generate a laser scan message and return it
			scanMsg = LaserScan()
			h = Header()
			h.stamp = rospy.Time.now()
			scanMsg.header = h

			scanMsg.angle_min = req.angleMin
			scanMsg.angle_max = req.angleMax
			scanMsg.angle_increment = req.angleIncrement
			scanMsg.time_increment = self.time_increment
			scanMsg.scan_time = self.scan_time
			scanMsg.range_max = req.rangeMax+.01
			scanMsg.range_min = req.rangeMin
			scan=[i/self.avgPxPerM for i in ray_lengths]
			scanMsg.ranges = scan
			scanMsg.intensities = []
			#print (time.time()-start)*1000
			return LaserOracleResponse(scanMsg)
		else:
			rospy.logwarn("Can't see robot {0}, only {1}".format(req.robotID, self.currentTags.keys()))
			return None

	def distance(self, p1, p2):
		return math.sqrt(math.pow(p1[0]-p2[0],2) + math.pow(p1[1]-p2[1],2))

	def distancePose(self, p1, p2):
		return (math.sqrt(
			(math.pow(p1.position.x - p2.position.x, 2)) +
			(math.pow(p1.position.y - p2.position.y, 2)) +
			(math.pow(p1.position.z - p2.position.z, 2))
				))

	def distancePixels(self, p1, p2):
		return self.distance((p1.x, p1.y), (p2.x, p2.y))

	#Check for counterclockwise arrangement of points
	#Points are 2-tuples of the form (x,y) or [x,y]
	#translated from http://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
	def orientation(self, a,b,c):
		orient = ((c[1]-a[1]) * (b[0]-a[0])) - ((b[1]-a[1]) * (c[0] - a[0]))
		if orient == 0: #Collinear
			return 0
		if orient > 0: #clockwise
			return 1
		return 2 #counterclockwise

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


	def drawRobots(self, image):
		#Draw a blue circle for each robot so they avoid each other
		for robotID in self.currentTags.keys():
			try:
				x=int(self.currentTags[robotID].tagCenterPx.x)
				y=int(self.currentTags[robotID].tagCenterPx.y)
				cv2.circle(image, (x,y), self.robotRad, np.array([115,255,255]), -1)
			except KeyError, e:
				#The list changed while we were drawing, do nothing
				pass

		return image
			#Optional, draw the robot ID on the robot
			#cv2.putText(self.image, str(robotID), (x-robotRad/2, y), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.3, np.array([115,255,30]))


	def update_image(self, msg):
		img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
		img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

		#Draw blue dots over all the robots so they get detected as obstacles
		img = self.drawRobots(img)

		#B&W image that is white for every pixel in the range
		mask = cv2.inRange(img, lower_blue, upper_blue)
		#Erode the mask to remove little noise pixels
		mask = cv2.erode(mask, kernel, iterations=1)

		#Store the binary image for calculation of laser scans later
		self.image = mask


def init_oracle():
	ls = LaserServer()

	rospy.init_node('laser_oracle_server')

	#Subscribe to april tag tag detections
	rospy.Subscriber("/tag_detections", AprilTagDetectionArray, ls.update_tags)

	#Subscribe to images from the overhead camera
	rospy.Subscriber("/overhead_cam/image_rect_color", Image, ls.update_image)

	rate = rospy.Rate(3)
	while not ls.isReady():
		rate.sleep()

	svc = rospy.Service('laser_oracle', LaserOracle, ls.handle_laser_req)
	rospy.loginfo("Laser oracle started...")
	rospy.spin()


if __name__ == "__main__":
	init_oracle()
