#!/usr/bin/python

#Generates spoof april tags based on the command line 

import argparse
from apriltags_ros.msg import *

parser = argparse.ArgumentParser(description='Publish AprilTag messages based on the command line rather than actual tags.')

#Tag positions are just floating point numbers, each triplet of numbers is a tag location in meters
#IDs are assigned to them in order, so the first one gets ID 0, the next one gets ID 1, etc. 
parser.add_argument('tag_locations', metavar='N', type=float, nargs='+', help='values for tag positions')
parser.add_argument('--rate', dest='pub_rate', type=int, action='store', default=10, help='rate setting, default is 10Hz')

args = parser.parse_args()
print args.tag_locations
print args.pub_rate

#Blow up if there are not enough x, y location pairs. 
assert len(args.tag_locations) % 3

#Create a publisher
tag_pub = rospy.Publisher("tag_detections", AprilTagDetectionArray, queue_size=10)

r = rospy.rate(args.pub_rate)

while not rospy.is_shutdown():
	#The detection array (AprilTagDetectionArray.msg) just contains AprilTagDetection[] detections, 
	#which themselves are:
	#  int32 id
	#  float64 size
	#  geometry_msgs/PoseStamped pose (which has a position and orientation)
	tags = AprilTagDetectionArray()

	#Reset tag ID to 0 to show the same tags every time
	tag_id  = 0

	#Split the list into groups of three (x,y,z) and iterate them
	for x,y,z in zip(*(iter(args.tag_locations),) * 3)
		tag = AprilTagDetection()

		#Use and increment the tag ID
		tag.id = tag_id
		tag_id += 1

		#Set up the message header
		tag.pose.header.stamp = rospy.Time.now()
		tag.pose.header.frame_id = "/camera_frame"
		#Set the position
		tag.pose.position.x = x
		tag.pose.position.y = y
		tag.pose.position.z = z

		#Set the orientation, this was just copied from recorded tags
		#and should be just about flat
		tag.pose.orientation.x = 0.96745175362
        tag.pose.orientation.y = 0.0217528708868
        tag.pose.orientation.z = 0.214215655418
        tag.pose.orientation.w = -0.132949501691

		#This is the size of tags I was using. 
		tag.size = 0.051

		#TODO setting the tag center and tag corners in pixels is a problem, since I don't have the image

	tag_pub.publish(tags)
	r.sleep()


#For each tag triplet in the locations, generate and publish an AprilTag message 
