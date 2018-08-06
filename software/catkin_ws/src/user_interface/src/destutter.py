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

		# #Get a list of all the ended strokes and see if any need to be merged
		# ended_strokes = []
		# for stroke_id in self.strokes.keys():
		# 	if self.strokes[stroke_id].isEnded:
		# 		ended_strokes.append(stroke_id)

		# #Stutter removal. For each stroke and the stroke after it, if they are seperated by less than 
		# #the stutter thresholds in both time and space, they are a product of a poor finger tracking over 
		# #the screen, not actually intended to be seperate events. This merges some strokes.
		# #TODO may need a spatial as well as chronological component for properly resolving multitouch membership
		# for index, item in enumerate(ended_strokes):
		# 	if index > 0: 
		# 		current = self.strokes[ended_strokes[index]]
		# 		prev = self.strokes[ended_strokes[index - 1]]

		# 		if prev.overlaps(current):
		# 			#The events overlap. Either the current one started first or the previous one did
		# 			overlapTime = min(prev.endTime, current.endTime) - max(prev.startTime, current.startTime)

		# 			# if prev.startTime < current.startTime:
		# 			# 	print "{0} overlaps {1} by {2}".format(prev.id, current.id, overlapTime)
		# 			# else:
		# 			# 	print "{0} overlaps {1} by {2}".format(prev.id, current.id, overlapTime)
		# 			if overlapTime < 0.01:
		# 				current.merge(prev)
		# 				#TODO Is this going to give me a bad day? I'm not mutating the thing I'm enumerating...
		# 				del(self.strokes[ended_strokes[index-1]])
		# 		else:
		# 			#The events don't overlap
		# 			if prev.endTime < current.startTime:
		# 				#print "{0} ends {1} before {2} begins".format(prev.id, current.startTime - prev.endTime, current.id)
		# 				if current.startTime - prev.endTime < 0.099:
		# 					current.merge(prev)
		# 					del(self.strokes[ended_strokes[index-1]])
		
		#dump for debugging
		for stroke_id in self.strokes.keys():
			if self.strokes[stroke_id].isEnded:		
				touch_cleaner.dumpStroke(self.strokes[stroke_id])
				del self.strokes[stroke_id]

topic = "/touches"
rospy.init_node('touch_destutter')
tc = TouchCollector()
touchSub = rospy.Subscriber(topic, Kivy_Event, tc.touch_event_callback)
rospy.spin()