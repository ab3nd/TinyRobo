#!/usr/bin/python

#ROS wrapper for gesture stroke consolidation, mostly getting gestures that are broken up 
#by stuttering contact with the touchscreen into single sets of events

import rospy
from user_interface.msg import Kivy_Event

import touch_cleaner

class TouchCollector(object):

	def __init__(self):
		self.strokes = {}
		self.commands = []

	def touch_event_callback(self, event):
		if event.uid not in self.strokes.keys():
			#If this event is just starting, create a new entry for it in strokes
			self.strokes[event.uid] = touch_cleaner.GestureStroke(event)
		else:
			#Event already started, extend its entry
			self.strokes[event.uid].addEvent(event)

		#Process all of the events which have ended
		for stroke_id in self.strokes.keys():
			if self.strokes[stroke_id].isEnded:
				print "Stroke {0} ended".format(stroke_id)
				touch_cleaner.dumpStroke(self.strokes[stroke_id])
				del self.strokes[stroke_id]

# #Now strokes contains every touch event series
# for command in commands:
# 	dumpCommand(command)

#Do an all pairs comparison to figure out whether any of the strokes in each of the commands should be merged,
# This is to determine what the time threshold is between commands that are deliberately 
# seperate from each other, and commands that are the same, but seperated by finger stutter on the screen
# for cmd in commands:
# 	#Each command has strokes, each stroke is a canidate for merging with each other stroke
# 	for strokeA in cmd.strokes.values():
# 		for strokeB in cmd.strokes.values():
# 			if strokeA.overlaps(strokeB):
# 				#The events overlap. Either A starts first or B does
# 				overlapTime = min(strokeA.endTime, strokeB.endTime) - max(strokeA.startTime, strokeB.startTime)

# 				if strokeA.startTime < strokeB.startTime:
# 					print "{0} overlaps {1} by {2}".format(strokeA.id, strokeB.id, overlapTime)
# 				else:
# 					print "{0} overlaps {1} by {2}".format(strokeA.id, strokeB.id, overlapTime)
# 			else:
# 				#The events don't overlap
# 				#TODO this doesn't need to be all pairs, I only care about sequences
# 				if strokeA.endTime < strokeB.startTime:
# 					if (strokeB.id - strokeA.id) == 1:
# 						print "{0} ends {1} before {2} begins".format(strokeA.id, strokeB.startTime - strokeA.endTime, strokeB.id)
# 	print "---"		

topic = "/touches"
rospy.init_node('touch_destutter')
tc = TouchCollector()
touchSub = rospy.Subscriber(topic, Kivy_Event, tc.touch_event_callback)
rospy.spin()