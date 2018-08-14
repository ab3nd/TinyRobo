#!/usr/bin/python

#ROS wrapper for gesture stroke consolidation, mostly getting gestures that are broken up 
#by stuttering contact with the touchscreen into single sets of events

import rospy
from user_interface.msg import Kivy_Event, Stroke
from geometry_msgs.msg import Point
import math
import touch_cleaner

class TouchCollector(object):

	def __init__(self):
		self.strokes = {}
		self.endedStrokes = []
		self.strokePub = rospy.Publisher('strokes', Stroke, queue_size=10)
		#In the original destutter code, empirically, events that overlapped by less than 0.01s
		#or didn't overlap but were more than 0.099s apart could be merged. 
		self.merge_overlap_threshold = rospy.Duration.from_sec(0.01)
		self.merge_gap_threshold = rospy.Duration.from_sec(0.099)
		#On the 3M screen, a fingertip is about 70px wide, because a finger is about 2cm, 
		#and the screen is 47.5cm wide or 1680px wide, so 2*1680/47.5 = ~70.
		self.merge_distance_threshold = 80

	def distance(self, e1, e2):
		return math.sqrt(math.pow(e1.point.x - e2.point.x,2) + math.pow(e1.point.y - e2.point.y,2) + math.pow(e1.point.z - e2.point.z,2))

	def touch_event_callback(self, event):
		if event.uid not in self.strokes.keys():
			#If this event is just starting, create a new entry for it in strokes
			self.strokes[event.uid] = touch_cleaner.GestureStroke(event)
		else:
			#Event already started, extend its entry
			self.strokes[event.uid].addEvent(event)


		#If the stroke just ended, try merging it with the other ended strokes, if any
		if self.strokes[event.uid].isEnded:

			just_ended = self.strokes[event.uid]

			#The arriving event just ended a stroke
			#All other strokes are either ended, or have not ended yet
			#If the stroke that just ended can be merged with an ended event, merge it
			for ended_stroke in [x for x in self.strokes.values() if x.isEnded]:
				if just_ended == ended_stroke:
					continue
				#Check time and space distances
				if just_ended.overlaps(ended_stroke):
					overlapTime = min(just_ended.endTime, ended_stroke.endTime) - max(just_ended.startTime, ended_stroke.startTime)
					if overlapTime < self.merge_overlap_threshold:
						#Overlap is small enough, check distance
						d = 0
						if just_ended.endTime < ended_stroke.startTime:
							#Distance from just_ended end to ended_stroke start
							d = self.distance(just_ended.events[-1], ended_stroke.events[0])
						else:
							#Distance from ended stroke end to just_ended start
							d = self.distance(ended_stroke.events[-1], just_ended.events[0])
						if d < self.merge_distance_threshold:
							#Overlap is small enough and distance is small enough
							ended_stroke.merge(just_ended)
							#Have to delete this way instead of through the reference
							del self.strokes[event.uid]
				else:
					#They don't overlap, so check that the ends are close enough in time and space
					gap = 0
					d = 0
					if just_ended.endTime < ended_stroke.startTime:
						#Gap between just_ended end and e2 start
						gap = ended_stroke.startTime - just_ended.endTime
						#Distance from just_ended end to e2 start
						d = self.distance(just_ended.events[-1], ended_stroke.events[0])
					else:
						#Gap between e2 end and just_ended start
						gap = just_ended.startTime - ended_stroke.endTime
						#Distance from e2 end to just_ended start
						d = self.distance(ended_stroke.events[-1], just_ended.events[0])
					if gap < self.merge_gap_threshold and d < self.merge_distance_threshold:
						#Gap in time and distance is small enough
						ended_stroke.merge(just_ended)
						#Have to delete this way instead of through the reference
						del self.strokes[event.uid]

	def timer_callback(self, timerEvent):
		#For all the currently ended strokes, if they might be able to merge with an ongoing or future event, 
		#hang on to them and wait, otherwise publish them 
		for ended_stroke in [x for x in self.strokes.values() if x.isEnded]:
			could_merge = False
			for active_stroke in [x for x in self.strokes.values() if not x.isEnded]:
				#If the end of the ended stroke and the beginning of the active stroke are close enough
				d = self.distance(ended_stroke.events[-1], active_stroke.events[0])
				
				if d < self.merge_distance_threshold:
					#Close enough, check time difference
					if ended_stroke.overlaps(active_stroke):
						overlapTime = ended_stroke.endTime - active_stroke.startTime
						if overlapTime < self.merge_overlap_threshold:
							#They overlap by a small enough amount to merge
							could_merge = True
							break
					else:
						gap = active_stroke.startTime - ended_stroke.endTime
						if gap < self.merge_gap_threshold:
							could_merge = True
							break

			if not could_merge:
				#This stroke can't be merged with an existing active stroke
				if rospy.Time.now() - ended_stroke.endTime > self.merge_gap_threshold:
					#No future stroke can get start soon enough in time to merge with this event
					#so publish it
					strokeMsg = Stroke()
					strokeMsg.events = ended_stroke.events
					strokeMsg.isDoubletap = ended_stroke.events[-1].isDoubletap
					strokeMsg.isTripletap = ended_stroke.events[-1].isTripletap
					strokeMsg.start = ended_stroke.startTime
					strokeMsg.end = ended_stroke.endTime
					strokeMsg.uid = ended_stroke.id
					strokeMsg.centroid = Point()
					strokeMsg.centroid.x = ended_stroke.centroid[0] 
					strokeMsg.centroid.y = ended_stroke.centroid[1]
					strokeMsg.width = ended_stroke.height()
					strokeMsg.height = ended_stroke.width()
					self.strokePub.publish(strokeMsg)
					#Delete the stroke so it doesn't get republished
					if ended_stroke.events[0].uid in self.strokes.keys():
						del self.strokes[ended_stroke.events[0].uid]
		
topic = "/touches"
rospy.init_node('touch_destutter')
tc = TouchCollector()
#Subscribe to touch events that may have stutter from fingers skipping on the screen
touchSub = rospy.Subscriber(topic, Kivy_Event, tc.touch_event_callback)
#Check for finished strokes and send them
strokeTimer = rospy.Timer(rospy.Duration(0.1), tc.timer_callback);

rospy.spin()
