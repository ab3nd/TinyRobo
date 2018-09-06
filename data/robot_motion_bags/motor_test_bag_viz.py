#!/usr/bin/python

# Given a bagfile, outputs a csv of the tag location and rotation for 
# tag 7 (the placebo robot). This is for qantifying camera/tag detection noise

import rosbag
import argparse
import math
from tf import transformations as transf

args = argparse.ArgumentParser(description="Given a bagfile, outputs a csv of the tag location and rotation for the given tag, as well as the commanded motion")
args.add_argument("file")
args.add_argument("robot")
in_args = args.parse_args()

bag = rosbag.Bag(in_args.file)
robot_id = int(in_args.robot)


#Euclidean distance
def dist(a, b):
	assert len(a) == len(b)
	return math.sqrt(sum([math.pow(i-j, 2) for i, j in zip(a,b)]))

prev_pos = None
prev_heading = None
prev_pos_time = None
#Set everything to 0 initially
lin_vel = rot_vel = m1 = m2 = vel = 0
#Store everything in a dict indexed by time
events = {}
#Assume everything starts at zero
lin_vel = rot_vel = m1 = m2 = vel = r_vel = 0.0

for topic, msg, t in bag.read_messages():
	if topic == "/cmd_vel":
		#Message is a twist message from the drive script
		lin_vel = msg.linear.x
		rot_vel = msg.angular.z
	elif topic == "/diff_drive_node/drive_cmd":
		#Message is a Motor_Vel_Cmd
		m1 = msg.motor1
		m2 = msg.motor2
	elif topic == "/tag_detections":
		if len(msg.detections) > 0:
			for tag in msg.detections:
				if tag.id == robot_id:
					tag_id = tag.id
					tag_x = tag.pose.pose.position.x
					tag_y = tag.pose.pose.position.y
					tag_z = tag.pose.pose.position.z
					#We have a previous pose, so calculate the velocity of the movement
					if prev_pos is not None:
						d = dist((tag_x, tag_y), prev_pos)
						delta_t = t - prev_pos_time
						vel = d/delta_t.to_sec() #In m/sec
					#update previous position (time updated later)
					prev_pos = (tag_x, tag_y)
					
					#Convert to RPY
					o = tag.pose.pose.orientation
					r, p, y = transf.euler_from_quaternion([o.w, o.x, o.y, o.z]) 

					#We have a previous heading, so calculate the rotational velocity
					if prev_heading is not None:
						#d = abs(prev_heading-r) #TODO this may fail weirdly across +/- pi radians
						#THIS ASSUMES THAT THE SMALLER ANGLE IS THE DIRECTION WE'RE MOVING
						d = abs(math.atan2(math.sin(r-prev_heading), math.cos(r-prev_heading)))
						delta_t = t - prev_pos_time
						r_vel = d/delta_t.to_sec() # in rads/sec
					prev_heading = r
					prev_pos_time = t

	else:
		#This is an error, something got bagged that this script doesn't deal with
		print topic, msg, t
	events[t] = [lin_vel, rot_vel, m1, m2, vel, r_vel]

bag.close()

#Now events is a dictionary, by time, of all of the messages, lets plot it
import pandas as pd
import matplotlib.pyplot as plt

bag_frame = pd.DataFrame.from_dict(events, orient="index")
by_time = bag_frame.sort_index()
by_time = by_time.rename(columns={0:"lin_vel", 1: "rot_vel", 2: "m1", 3: "m2", 4:"vel", 5:"r_vel"})

#Imbiggen the plots
plt.rcParams['figure.figsize'] = [16,13]

# #Plot linear velocity
# by_time.plot(y=["lin_vel", "vel"], kind="line")

# #Come up with a file name, save and close
# figname = in_args.file.split(".")[0] + "-vel-lin_vel.png"
# plt.savefig(figname)
# plt.close()

# #Plot angular velocity
# by_time.plot(y=["rot_vel", "r_vel"], kind="line")

# #Come up with a file name, save and close
# figname = in_args.file.split(".")[0] + "-rot_vel-r_vel.png"
# plt.savefig(figname)
# plt.close()

#Plot all together
try:
	#by_time.plot(y=['lin_vel', 'vel', "rot_vel", "r_vel"], kind="line")
	#For debugging, just the measured velocities
	by_time.plot(y=['vel', "r_vel"], kind="line")

	#Come up with a file name, save and close
	figname = in_args.file.split(".")[0] + "-robot_{0}_all.png".format(in_args.robot)
	plt.savefig(figname)
	plt.close()

except KeyError:
	print "{0} didn't have the right data in it to plot".format(in_args.file)
