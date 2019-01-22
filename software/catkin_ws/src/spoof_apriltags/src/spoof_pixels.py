#!/usr/bin/python

#Generates spoof april tags based on the command line 

import argparse
import rospy
from apriltags_ros.msg import *
import cv2
import numpy as np

parser = argparse.ArgumentParser(description='Publish AprilTag messages based on the command line rather than actual tags.')
parser.add_argument('img_path', action='store', help='path to image file to use')
parser.add_argument('--rate', dest='pub_rate', action='store', default=10,
                    help='rate setting, default is 10Hz')

args = parser.parse_args()

def hsv_color_getter(event,x,y,flags,param):
	if event == cv2.EVENT_MOUSEMOVE:
		print hsv[y,x]
	#Get the color of the pixel under the mouse
	#Convert to HSV
	#Draw it on the image?

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
for c in contours:
	M = cv2.moments(c)
	cX = int(M["m10"] / M["m00"])
	cY = int(M["m01"] / M["m00"])
	cv2.circle(img, (cX, cY), 7, (0, 255, 0), -1)

cv2.imshow("mask", img)
cv2.waitKey(0)
# # Bitwise-AND mask and original image
# res = cv2.bitwise_and(img,img, mask= mask)

# # # Setup SimpleBlobDetector parameters.
# params = cv2.SimpleBlobDetector_Params()
 
# # # Change thresholds
# params.minThreshold = 10;
# params.maxThreshold = 200;

# detector = cv2.SimpleBlobDetector(params)
# points = detector.detect(res)

# #Draw the keypoints with circles around them
# im_with_keypoints = cv2.drawKeypoints(res, points, np.array([]), (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
 
# # Show keypoints
# cv2.imshow("Keypoints", im_with_keypoints)
# cv2.setMouseCallback('Keypoints', hsv_color_getter)
# cv2.waitKey(0)

# print args.pub_rate
