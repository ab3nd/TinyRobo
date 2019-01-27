#!/usr/bin/python

#Given a bag file, convert all the touches in it into Kivy Event touches
#This also strips out Kivy's idea of something being a doubletap/tripletap

import rosbag
import rospy
import yaml
import argparse
import os.path
import math
from user_interface.msg import Kivy_Event

parser = argparse.ArgumentParser(description="Convert a bagfile from Abe's experiment into per-task inputs")
parser.add_argument('bagfileName', type=str, nargs=1, help='path to the bagfile')
args = parser.parse_args()

#Get the file name and load the bagfile
name = args.bagfileName[0]
bag = rosbag.Bag(name)

#Save all the messages into a long list as kivy events
all_events = []
event_times = []

#Store start times for updating messages
started_touches = {}

for topic, msg, t in bag.read_messages():
	# Check the message type and topic		 	
	if topic == "/touches":
		newMsg = Kivy_Event()
		newMsg.uid = int(msg.header.frame_id)
		newMsg.point.x = msg.point.x
		newMsg.point.y = 1050 - msg.point.y #Flip to pixel coords
		newMsg.point.z = 0.0 #Should this be different? It's on a screen.

		#Start time is based on whether this touch has started already
		if msg.header.frame_id not in started_touches.keys():
			#This message is the first of its kind, update its start and update values
			# sec_f = rospy.Time.to_sec(t)
			# sec = int(math.floor(sec_f))
			# nsec = rospy.Time.to_nsec(t) - 1000000000 * sec_f

			newMsg.start.secs = t.secs
			newMsg.start.nsecs = t.nsecs

			#Update time is same as start time
			newMsg.update.secs = t.secs
			newMsg.update.nsecs = t.nsecs

			#Store start time for later messages
			started_touches[msg.header.frame_id] = t
		else:
			#This message is part of a sequence that already started
			#Use the stored start time
			# sec_f = rospy.Time.to_sec(started_touches[msg.header.frame_id])
			# sec = int(math.floor(sec_f))
			# nsec = rospy.Time.to_nsec(started_touches[msg.header.frame_id]) - 1000000000 * (sec_f
			newMsg.start.secs = started_touches[msg.header.frame_id].secs
			newMsg.start.nsecs = started_touches[msg.header.frame_id].nsecs

			#Use the new time as update time
			#sec_f = rospy.Time.to_sec(t)
			newMsg.update.secs = t.secs #int(math.floor(sec_f))
			newMsg.update.nsecs = t.nsecs #rospy.Time.to_nsec(t) - 1000000000 * sec_f
		
		#The end time and ended state of this message will be set in the reverse pass
		newMsg.ended = False
		newMsg.end = newMsg.update #Kivy uses -1, which isn't a valid time

		#We're removing these values so we can use doubletap as end-of-command
		newMsg.isTripletap = newMsg.isDoubletap = False

		#Store the message and its time
		all_events.append(newMsg)
		event_times.append(t)

#Run through the list in reverse, when you hit a new message, it's the last of that UID
#so set that it has ended and when it ended
ended_touches = set()
for msg in reversed(all_events):
	if msg.uid not in ended_touches:
		#We haven't seen this one yet, so it's the last one that arrived with that UID
		#End it
		msg.ended = True
		msg.end = msg.update
		#Record that we ended it
		ended_touches.add(msg.uid)

#So now all the messages are in all_events and are properly ended if they should be
#Generate a new rosbag with the Kivified touch events in it
outBag = os.path.basename(name).split('.')[0] + "_asKivy.bag"

newBagfile = rosbag.Bag(outBag, 'w')
	
for msg, t in zip(all_events, event_times):
	
	newBagfile.write('/touches', msg, t)	
			
newBagfile.close()