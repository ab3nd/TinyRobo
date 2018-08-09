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

		
topic = "/touches"
rospy.init_node('touch_destutter')
tc = TouchCollector()
touchSub = rospy.Subscriber(topic, Kivy_Event, tc.touch_event_callback)
rospy.spin()


		#So canidates for merging because of stutter are strokes that:
		# - Have ended
		# - Have their start and end points overlapping by < 0.01s
		#                     - OR - 
		# - Have their start and end points not overlapping but < 0.099s apart
		#                     - AND - 
		# - Have their start and end points less than 70 or so pixels apart

		# #First, try to merge any ended events that are canidates for merging
		# ended = [uid for uid in self.strokes.keys() if self.strokes[uid].isEnded]

		# for e1 in ended:
		# 	for e2 in ended:
		# 		#Don't merge event with itself
		# 		if e1 == e2:
		# 			continue
		# 		if self.strokes[e1].overlaps(self.strokes[e2]):
		# 			overlapTime = min(self.strokes[e1].endTime, self.strokes[e2].endTime) - max(self.strokes[e1].startTime, self.strokes[e2].startTime)
		# 			if overlapTime > merge_overlap_threshold:
		# 				#Don't merge events if overlap is too high
		# 				continue
		# 			else:
		# 				d = 0
		# 				if e1.endTime < e2.startTime:
		# 					#Distance from e1 end to e2 start
		# 					d = self.distance(self.strokes[e1].events[-1], self.strokes[e2].events[0])
		# 				else
		# 					#Distance from e2 end to e1 start
		# 					d = self.distance(self.strokes[e2].events[-1], self.strokes[e1].events[0])
		# 				if d > merge_distance_threshold:
		# 					#Don't merge overlapping events if start and end are too far apart
		# 					continue:
		# 		else:
		# 			gap = 0
		# 			d = 0
		# 			if e1.endTime < e2.startTime:
		# 				#Gap between e1 end and e2 start
		# 				gap = e2.startTime - e1.endTime
		# 				#Distance from e1 end to e2 start
		# 				d = self.distance(self.strokes[e1].events[-1], self.strokes[e2].events[0])
		# 			else
		# 				#Gap between e2 end and e1 start
		# 				gap = e1.startTime - e2.endTime
		# 				#Distance from e2 end to e1 start
		# 				d = self.distance(self.strokes[e2].events[-1], self.strokes[e1].events[0])
		# 			if gap > merge_gap_threshold or d > merge_distance_threshold:
		# 				#Don't merge non-overlapping events if the gap is too big in space or time
		# 				continue


		# 		#Didn't manage to disqualify this merger, merge them



		#Canidates for publishing as complete strokes are strokes that are not a canidate 
		#for merging with any other stroke, which means they:
		# - Have ended
		# - Their endpoints in space are more than 70px or so from the start of any active stroke
		#                   - OR - 
		# - Their endpoints in time are more than overlapping all other strokes starts by > 0.01
		#                   - OR - 
		# - Their endpoints in time are > 0.1s from the start of any other stroke

		# #Get a list of the ended strokes and the still-active ones
		# ended = [uid for uid in self.strokes.keys() if self.strokes[uid].isEnded]
		# active = [uid for uid in self.strokes.keys() if not self.strokes[uid].isEnded]

		# for ended_id in ended:
		# 	min_dist = float('inf')
		# 	min_gap = float('inf')
		# 	min_overlap = float('inf')

		# 	for active_id in active:
		# 		#Distance between ended stroke's end and active stroke's start
		# 		d = self.distance(self.strokes[ended_id].events[-1], self.strokes[active_id].events[0])
		# 		if d < min_dist:
		# 			min_dist = d

		# 		#Check any overlaps with the starts of active strokes
		# 		if self.strokes[ended_id].overlaps(self.strokes[active_id]):
		# 			overlapTime = min(self.strokes[ended_id].endTime, self.strokes[active_id]) - max(self.strokes[ended_id].startTime, self.strokes[active_id].startTime)
		# 			if overlapTime < min_overlap:
		# 				min_overlap = overlapTime
		# 		#If they don't overlap, check if the active stroke started very soon after the ended stroke ended
		# 		else:
		# 			if self.strokes[ended_id].endTime < self.strokes[active_id].startTime:
		# 				gap = self.strokes[active_id].startTime - self.strokes[ended_id].endTime
		# 				if gap < min_gap:
		# 					min_gap = gap

		# 	if min_dist > merge_distance_threshold:
		# 		#The ended stroke doesn't end close enough to any active stroke to merge with them
		# 		#So publish the ended stroke
		# 	else:
		# 		#The ended stroke does end close enough to an active stroke to merge with it
		# 		#Check that that the timing is good to go for the merge
		# 		if min_gap < merge_gap_threshold or min_overlap < merge_gap_threshold:
		# 			#Don't publish the ended stroke, it will need to be merged with the active stroke when that stroke finishes
		# 			pass
		# 		else:
		# 			#The location was OK, but the timing is off to merge with any active stroke
		# 			#So publish this stroke

		#At this point, any finished strokes that couldn't be merged with any active strokes have been published
		#Now go through and merge any 

		# #dump for debugging
		# for stroke_id in self.strokes.keys():
		# 	if self.strokes[stroke_id].isEnded:		
		# 		touch_cleaner.dumpStroke(self.strokes[stroke_id])
		# 		del self.strokes[stroke_id]

# def checkStack():
	# 	#In the original destutter code, empirically, events that overlapped by less than 0.01s
	# 	#or didn't overlap but were more than 0.099s apart could be merged. 
	# 	merge_overlap_threshold = 0.01
	# 	merge_gap_threshold = 0.099
	# 	#On the 3M screen, a fingertip is about 70px wide, because a finger is about 2cm, 
	# 	#and the screen is 47.5cm wide or 1680px wide, so 2*1680/47.5 = ~70.
	# 	merge_distance_threshold = 80

	# 	if len(endedStrokes) == 0:
	# 		#There are no ended strokes to deal with, so we're done
	# 		return
	# 	elif len(endedStrokes) == 1:
	# 		#If the one stroke could not be merged with any active strokes
	# 		#publish and remove the ended stroke
	# 		return
	# 	else:
	# 		#Check if the oldest finished stroke in the stack and the second-oldest can be merged
	# 		oldest = self.strokes[self.endedStrokes[0]]
	# 		second = self.strokes[self.endedStrokes[1]]

	# 		#If the oldest is still a canidate for 

	# 		if oldest.overlaps(second):
	# 			#The events overlap. Either the current one started first or the previous one did
	# 			overlapTime = min(oldest.endTime, second.endTime) - max(oldest.startTime, second.startTime)

	# 			if overlapTime < 0.01:
	# 				#Close enough in time to merge, check that they are also close enough in space
	# 				d = 0
	# 				if oldest.endTime < second.startTime:
	# 					d = self.distance(oldest.events[-1], second.events[0])
	# 				else:
	# 					d = self.distance(second.events[-1], oldest.events[0])
	# 				if d < merge_distance_threshold:
	# 					#Close enough in both time and space to merge
	# 					oldest.merge(second)
	# 				else:
	# 					self.publish(oldest)
	# 					del oldest
	# 					#Recursive call to see if we can clear any more old messages
	# 					self.checkStack()
	# 		else:
	# 			#The events don't overlap, see if they are close enough to merge
	# 			gap = 0
	# 			d = 0
	# 			if oldest.endTime < second.startTime:
	# 				#Gap between oldest end and second start
	# 				gap = second.startTime - oldest.endTime
	# 				#Distance from oldest end to second start
	# 				d = self.distance(self.strokes[oldest].events[-1], self.strokes[second].events[0])
	# 			else:
	# 				#Gap between second end and oldest start
	# 				gap = oldest.startTime - second.endTime
	# 				#Distance from second end to oldest start
	# 				d = self.distance(self.strokes[second].events[-1], self.strokes[oldest].events[0])
	# 			if gap < merge_gap_threshold and d < merge_distance_threshold:
	# 				#TODO Merge
	# 				#TODO publish and delete oldest
	# 				self.checkStack()


