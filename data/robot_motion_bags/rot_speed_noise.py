#!/usr/bin/python

# Given a bagfile, outputs a csv of the tag location and rotation for 
# tag 7 (the placebo robot). This is for qantifying camera/tag detection noise

import rosbag
import argparse
import math
from tf import transformations as transf

args = argparse.ArgumentParser(description="Given a bagfile, outputs a csv of the rotational speed for each robot")
args.add_argument("file")
in_args = args.parse_args()

bag = rosbag.Bag(in_args.file)


#Euclidean distance
def dist(a, b):
	assert len(a) == len(b)
	return math.sqrt(sum([math.pow(i-j, 2) for i, j in zip(a,b)]))

prev_pos = None
prev_heading = None
prev_pos_time = None

#I already know what all my robots' IDs are
ids = [0,1,4,5,6,7,8,10,17]
events = {}
prev_headings = {i:None for i in ids}

for topic, msg, t in bag.read_messages():
	if topic == "/tag_detections":
		if len(msg.detections) > 0:
			#set up zero velocities for tags we can't see
			vels = {robot_id:0 for robot_id in ids}

			for tag in msg.detections:
				#Convert to RPY
				o = tag.pose.pose.orientation
				r, p, y = transf.euler_from_quaternion([o.w, o.x, o.y, o.z]) 

				#We have a previous heading, so calculate the rotational velocity
				if prev_headings[tag.id] is not None:
					#THIS ASSUMES THAT THE SMALLER ANGLE IS THE DIRECTION WE'RE MOVING
					d = abs(math.atan2(math.sin(r-prev_headings[tag.id]), math.cos(r-prev_headings[tag.id])))
					delta_t = t - prev_pos_time
					r_vel = d/delta_t.to_sec() # in rads/sec
					vels[tag.id] = r_vel

				prev_headings[tag.id] = r
			prev_pos_time = t
			events[t] = vels

line = "time," + ','.join([str(i) for i in ids])
print line
for t in sorted(events.keys()):
	line = [str(t.to_sec())]
	line.extend([str(events[t][robot]) for robot in sorted(events[t].keys())])
	print ','.join(line)
