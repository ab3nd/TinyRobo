#!/usr/bin/python


import all_data_handler
import re
from pprint import pprint

#Given the object list of a gesture from the coding, return a list of letters representing normalized 
#values for the objects of the gesture
def tag_object(original):
	#This use of strip is to prevent quotes from messing up regex matches
	original = " ".join(original).strip("\"")
	tags = []
	#Matches robots, robot, bot, bots, etc. 
	robots = re.compile("bot|group|swarm|orange|red", re.I)
	crate = re.compile("crate", re.I)
	targetA = re.compile("area a|box a| a$|to a,|^a$", re.I)
	targetB = re.compile("area b|box b| b$|to b,|^b$", re.I)
	whitespace = re.compile("whitespace|ground|screen|white area", re.I)
	
	toCheck = [(robots, "r"), (crate, "c"), (targetA, "a"), (targetB, "b"), (whitespace, "w")]
	for compiled, tag in toCheck:
		if re.search(compiled, original):
			tags.append(tag)
	return tags


#Looking at lengths of sequences of taps
def tap_sequences(participant):
	#Dict of task id to count of taps on robots
	counts = {}
	for task_id in participant["tasks"].keys():
		#Set up an empty list for this task
		counts[task_id] = []
		events = participant["tasks"][task_id]
		#Flip through all the events
		task_count = 0

		# #Conditional breakpoint for debugging
		# if task_id == "11" and participant["participant"] == 1:
		# 	import pdb; pdb.set_trace()

		for event in events:   
			if event["event_type"] == "memo":
				#This event is a memo, not a user gesture
				continue
			elif "example" in event.keys() and event["example"] == True:
				#This event is an example, don't count it
				continue
			else:
				#We have an event, check if it's a tap on a robot
				if event["event_type"] == "tap":
					#Taps always have an object
					tags = tag_object(event["objects"])
					if 'r' in tags:
						#This is a tap on a robot, add it to the temp list
						task_count += 1
					else:
						counts[task_id].append(task_count)
						#Reset the counter
						task_count = 0
				else:
					#This is the end of the sequence, push the count into the event list
					counts[task_id].append(task_count)
					#Reset the counter for the next sequence
					task_count = 0
		if task_count > 0:
			#The last gestures were part of a tap sequence, so include them
			counts[task_id].append(task_count)
			task_count = 0
	return counts

def count_select_taps(participant):
	#Dict of task id to count of taps on robots
	counts = {}
	for task_id in participant["tasks"].keys():
		#Default to no counts of select taps
		counts[task_id] = 0
		events = participant["tasks"][task_id]

		#Flip through all the events
		for event in events:
			if event["event_type"] == "memo":
				#This event is a memo, not a user gesture
				continue
			elif "example" in event.keys() and event["example"] == True:
				#This event is an example, don't count it
				continue
			else:
				#We have an event, check if it's a tap on a robot
				if event["event_type"] == "tap":
					#Taps always have an object
					tags = tag_object(event["objects"])
					if 'r' in tags:
						counts[task_id] += 1
	return counts


#Load the user data
adh = all_data_handler.UserData()

#Get the taps
all_select_taps = adh.apply(count_select_taps)

#Get the tap sequences 
seqs = adh.apply(tap_sequences)


sum_all = sum_seq = 0
for uid in seqs.keys():
	for tid in seqs[uid].keys():
		sum_all += all_select_taps[uid][tid]
		sum_seq += sum(seqs[uid][tid])
		print "[{}, {}] {} {}".format(uid, tid, all_select_taps[uid][tid], sum(seqs[uid][tid]))
print sum_all, sum_seq