#!/usr/bin/python

#Given a bag file, convert all the touches in it into Kivy Event touches
#This also strips out Kivy's idea of something being a doubletap/tripletap

import rosbag
import rospy
import yaml
import argparse
import os.path
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
		newMsg.uid = msg.header.frame_id
		newMsg.point.x = msg.point.x
		newMsg.point.y = 1050 - msg.point.y #Flip to pixel coords
		newMsg.point.z = 0.0 #Should this be different? It's on a screen.

		#Start time is based on whether this touch has started already
		if msg.header.frame_id not in started_touches.keys():
			#This message is the first of its kind, update its start and update values
			newMsg.start = msg.header.stamp
			newMsg.update = msg.header.stamp
			started_touches[msg.header.frame_id] = msg.header.stamp
		else:
			#This message is part of a sequence that already started
			#Use the stored start time
			newMsg.start = started_touches[msg.header.frame_id]
			#Use the new update time
			newMsg.update = msg.header.stamp
		
		#The end time and ended state of this message will be set in the reverse pass
		newMsg.ended = False
		newMsg.end = -1

		#We're removing these values so we can use doubletap as end-of-command
		newMsg.isTripleTap = newMsg.isDoubleTap = False

		#Store the message and its time
		all_events.append(newMsg)
		event_times.append(t)

#Run through the list in reverse, when you hit a new message, it's the last of that UID
#so set that it has ended and when it ended
ended_touches = set()
for msg in reversed(all_events)
	if msg.uid not in ended_touches:
		#We haven't seen this one yet, so it's the last one that arrived with that UID
		#End it
		msg.ended = True
		msg.end = msg.update
		#Record that we ended it
		ended_touches.add(msg.uid)

#So now all the messages are in all_events and are properly ended if they should be
#Generate a new rosbag with the Kivified touch events in it
outBag = os.path.basename(name) + "asKivy.bag"
rosbag.Bag(outBag)
	
for msg, t in zip(all_events, event_times)
	outBag.write('/touches', msg, t)	
			
outBag.close()