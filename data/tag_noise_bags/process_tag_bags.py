#!/usr/bin/python

# Given a bagfile, outputs a csv of the tag location and rotation for 
# tag 7 (the placebo robot). This is for qantifying camera/tag detection noise

import rosbag
import argparse
from tf import transformations as transf

args = argparse.ArgumentParser(description="Given a bagfile, outputs a csv of the tag location and rotation for tag 7 (the placebo robot).")
args.add_argument("file")
in_args = args.parse_args()

print in_args

bag = rosbag.Bag(in_args.file)
# topics = bag.get_type_and_topic_info()[1].keys()
# print topics
print "x,y,z,r,p,y"
for topic, msg, t in bag.read_messages(topics=['/tag_detections']):
    if len(msg.detections) > 0:
    	for detection in msg.detections:
    		if detection.id == 7:
    			pose = detection.pose.pose.position
    			#Convert to RPY?
    			o = detection.pose.pose.orientation
    			r, p, y = transf.euler_from_quaternion([o.w, o.x, o.y, o.z]) 
    			print "{0},{1},{2},{3},{4},{5}".format(pose.x, pose.y, pose.z, r, p, y)
bag.close()

