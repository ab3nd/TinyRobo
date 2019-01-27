#!/usr/bin/python

#Generates spoof april tags based on the command line 

import argparse
import rospy
from apriltags_ros.msg import *
from geometry_msgs.msg import Point
import cv2
import numpy as np

parser = argparse.ArgumentParser(description='Publish AprilTag messages based on the command line rather than actual tags.')
parser.add_argument('img_path', action='store', help='path to image file to use')
parser.add_argument('--rate', dest='pub_rate', action='store',type=int, default=10,
                    help='rate setting, default is 10Hz')

args = parser.parse_args()


#Load image
img = cv2.imread(args.img_path)

rospy.loginfo("Spoofing with {}".format(args.img_path))
#Convert image to B&W by getting rid of all non-red pixels
# Convert BGR to HSV
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
# Orange robots
mask1 = cv2.inRange(hsv, np.array([9,150,230]), np.array([18,255,255]))

mask2 = cv2.inRange(hsv, np.array([170,230,210]), np.array([180,255,255]))
mask3 = cv2.inRange(hsv, np.array([0,220,120]), np.array([8,255,255]))
mask = mask1 | mask2 | mask3

#Get the external contours, simple edge approximation (reduces edge point count)
mod_img, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#Draw the contours
#cv2.drawContours(img, contours, -1, (0,255,0), 3)

#Get their centers in pixels 
robot_centers = []
for c in contours:
    M = cv2.moments(c)
    #Skip zero-area contours
    if M["m00"] == 0:
        continue 
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    # cv2.circle(img, (cX, cY), 7, (0, 255, 0), -1)

    #Get a rotated rectangle around the contour
    bb = cv2.minAreaRect(c)
    box = cv2.boxPoints(bb)
    box = np.int0(box)
    robot_centers.append((cX, cY, box))

#cv2.imshow("mask", img)
#cv2.waitKey(0)

rospy.init_node('faketags', anonymous=True)
# set up apriltag publisher
tagPub = rospy.Publisher('tag_detections', AprilTagDetectionArray, queue_size=10) 

#set up my rate based on the command line arg
r = rospy.Rate(args.pub_rate)

#Correction because the images were smaller than the screen, and 
#were displayed centered in the screen, so the logged location of the user
#touches are in screen coordinates, and the detected robot location is in 
#image coordinates. 
fudge_x = 150 # (height of screen (1050) - height of image (750))/2
fudge_y = 340 # (width of screen (1680) - width of image (1000))/2

while not rospy.is_shutdown():
    
    
    detected_tags = []
    tagID = 0
    for robot in robot_centers:
        t = AprilTagDetection()

        #Tag IDs are just incremented
        t.id = tagID 
        tagID += 1

        #Size is fixed, location based on image
        t.size = 0.051
        t.tagCenterPx.x = robot[0] + fudge_x
        t.tagCenterPx.y = robot[1] + fudge_y
        t.tagCenterPx.z = 0.0
        
        #Corners from rotated bounding box in image
        for corner in robot[2]:
            c = Point()
            c.x = corner[0] + fudge_x
            c.y = corner[1] + fudge_y
            c.z = 0 #Tags are flat in the image
            t.tagCornersPx.append(c)
        
        #Real world pose isn't used because we're just testing UI
        #Set up the message header
        t.pose.header.stamp = rospy.Time.now()
        t.pose.header.frame_id = "/camera_frame"
        #Set the position, all zeros because it's in an image, so it doesn't have a real position
        t.pose.pose.position.x = 0
        t.pose.pose.position.y = 0
        t.pose.pose.position.z = 0

        #Set the orientation, this was just copied from recorded tags
        #and should be nearly flat
        t.pose.pose.orientation.x = 0.96745175362
        t.pose.pose.orientation.y = 0.0217528708868
        t.pose.pose.orientation.z = 0.214215655418
        t.pose.pose.orientation.w = -0.132949501691

        #Add it to the array
        detected_tags.append(t)

    #Build the message
    tags = AprilTagDetectionArray()
    tags.detections = detected_tags

    #Publish and wait 
    tagPub.publish(tags)
    r.sleep()
