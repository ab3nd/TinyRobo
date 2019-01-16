#!/usr/bin/python

#Generates spoof april tags based on the command line 

import argparse
import rospy
#from apriltags_ros.msg import *
import cv2
import numpy as np

parser = argparse.ArgumentParser(description='Publish AprilTag messages based on the command line rather than actual tags.')
parser.add_argument('img_path', action='store', help='path to image file to use')
parser.add_argument('--rate', dest='pub_rate', action='store', default=10,
                    help='rate setting, default is 10Hz')

args = parser.parse_args()

def hsv_color_getter(event,x,y,flags,param):
	if event == cv2.EVENT_MOUSEMOVE:
		print res[y,x]
	#Get the color of the pixel under the mouse
	#Convert to HSV
	#Draw it on the image?

#Load image
img = cv2.imread(args.img_path)

#Convert image to B&W by getting rid of all non-red pixels
# Convert BGR to HSV
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
# define range of robot colors in HSV
mask = cv2.inRange(hsv, np.array([0,50,50]), np.array([25,255,255]))
#mask2 = cv2.inRange(hsv, np.array([170,100,200]), np.array([180,255,255]))
#mask = mask1 | mask2
# Bitwise-AND mask and original image
res = cv2.bitwise_and(img,img, mask= mask)

# # Setup SimpleBlobDetector parameters.
# params = cv2.SimpleBlobDetector_Params()
 
# # Change thresholds
# params.minThreshold = 10;
# params.maxThreshold = 200;

#detector = cv2.SimpleBlobDetector()#params)
#points = detector.detect(res)

#Draw the keypoints with circles around them
#im_with_keypoints = cv2.drawKeypoints(res, points, np.array([]), (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
 
# Show keypoints
cv2.imshow("Keypoints", res)
cv2.setMouseCallback('Keypoints', hsv_color_getter)
cv2.waitKey(0)

# print args.pub_rate
