#!/usr/bin/python

#Generates spoof april tags based on the command line 

import argparse
import rospy
#from apriltags_ros.msg import *
import cv2
import numpy as np

parser = argparse.ArgumentParser(description='Publish AprilTag messages based on the command line rather than actual tags.')
parser.add_argument('img_path', action='store', help='path to image file to use')
parser.add_argument('--rate', dest='pub_rate', action='store',type=int, default=10,
                    help='rate setting, default is 10Hz')

args = parser.parse_args()


#Load image
img = cv2.imread(args.img_path)

#Convert image to B&W by getting rid of all non-red pixels
# Convert BGR to HSV
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
# Orange robots
mask1 = cv2.inRange(hsv, np.array([9,150,230]), np.array([18,255,255]))

mask2 = cv2.inRange(hsv, np.array([170,230,210]), np.array([180,255,255]))
mask3 = cv2.inRange(hsv, np.array([0,220,120]), np.array([8,255,255]))
mask = mask1 | mask2 | mask3

#Get the external contours, simple edge approximation (reduces edge point count)
contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#Draw the contours
#cv2.drawContours(img, contours, -1, (0,255,0), 3)

#Get their centers in pixels 
robot_centers = []
for c in contours:
	M = cv2.moments(c)
	cX = int(M["m10"] / M["m00"])
	cY = int(M["m01"] / M["m00"])
	#cv2.circle(img, (cX, cY), 7, (0, 255, 0), -1)
	robot_centers.append((cX, cY))

# cv2.imshow("mask", img)
# cv2.waitKey(0)

rospy.init_node('faketags', anonymous=True)
#TODO set up apriltag publisher


#set up my rate based on the command line arg
r = rospy.Rate(args.pub_rate)
while not rospy.is_shutdown():
	
	for robot in robot_centers:
		print robot
		#TODO publish the tag

	r.sleep()
