#!/usr/bin/python

#Print data from two runs, registered, in format for diagonal view

import json
import argparse
import sys

def time_err(a1, a2):
	total_error = 0
	#These arrays are the same length
	for event1, event2 in zip(a1,a2):
		if event1 is None or event2 is None:
			continue
		else:
			total_error += abs(event1['time'] - event2['time'])
	return total_error

def register_lists(list1, list2):
	#Create new event lists, keeping which coder did what consistent
	new_list_1 = []
	new_list_2 = []

	for task1, task2 in zip (list1, list2):
		#Make sure we're comparing the same tasks
		if task1 != task2:
			print "Error: cannot compare task {0} with task {1}".format(task2, task1)
			sys.exit(1)

		#Discard memos, they don't contribute to K
		events_1 = [x for x in data_1['tasks'][task1] if x['event_type'] != 'memo']
		events_2 = [x for x in data_2['tasks'][task2] if x['event_type'] != 'memo']
		
		#At this point we have two lists of events, describing the same real-world happenings
		#Sort the lists by time
		events_1 = sorted(events_1, key=lambda event: event['time'])
		events_2 = sorted(events_2, key=lambda event: event['time'])
	 	
	 	#The lists are sorted, but may not be aligned. Alignment in this case means that 
	 	#each item in the longer list is paired  with an item in shorter list, or with 
	 	#no item, because the item is something one coder saw and the other coder missed. 
	 	#The pairing should be such that it minimizes the total time difference between 
	 	#paired items (so that each pair consists of items as close together as possible)

	 	#Can get away with brute forcing this because it's usually small sets
	 	import itertools
	 	
		unmatched = abs(len(events_1) - len(events_2))
		
		#Keep track of which coder had the short list
		coder_1_short = False
	 	if len(events_1) < len(events_2):
	 		coder_1_short = True
	 		shortlist = events_1
	 		longlist = events_2
	 	else:
	 		shortlist = events_2
	 		longlist = events_1

	 	#For all combinations of the shorter list and some non-events, check the time error
	 	#Save the one with minimum time error
	 	min_err_t = float('inf')
	 	best_match = None

	 	#For all possible combinations of indicies mapping the sort list to the long one
	 	for idx in itertools.combinations(range(len(longlist)), len(shortlist)):
	 		#Make a list of Nones as long as the longer of the two lists
			with_gaps = [None for _ in longlist]

			#Copy the items from the short list to the indexed locations
			#This ends up leaving some locations still set to None
			for from_idx, to_idx in enumerate(idx):
				with_gaps[to_idx] = shortlist[from_idx] 

			#Check if the time error is less than the minimum observed so far	
			t = time_err(with_gaps, longlist) 
			if t < min_err_t:
				#We got a lower time error, so keep this list
				min_err_t = t
				best_match = with_gaps

		#Now the long list and best_match are the original long list, and the
		#version of the short list with Nones added that had the least bad time
		#difference from the long list. 

		#Replace the Nones with nonevents
		for idx, event in best_match:
			if event is None:
 				best_match[idx] = {"time":0.0, "event_type":"MISS"}

		#Assign the lists back to the proper coder
		if coder_1_short:
			new_list_1 = best_match
			new_list_2 = longlist
		else:
			new_list_2 = best_match
			new_list_1 = longlist	

	return new_list_1, new_list_2


#The only two arguments are the file names
parser = argparse.ArgumentParser()
parser.add_argument("first", nargs = 1, help="First file, from one coder")
parser.add_argument("second", nargs = 1, help="Second file, from other coder")
args = parser.parse_args()

#Load both files, the [0] is because they're lists
data_1 = load_file(args.first[0])
data_2 = load_file(args.second[0])

#Get the participant number and check we're comparing the same participants
participant = data_1["participant"]
assert data_1["participant"] == data_2["participant"]

#get the coder names out of the file names
coder_1 = args.first[0].split('_')[-1].split('.')[0]
coder_2 = args.second[0].split('_')[-1].split('.')[0]

#For each task, get a the list of events
tasks_1 = sorted(data_1['tasks'].keys(), key=int)
tasks_2 = sorted(data_2['tasks'].keys(), key=int)

#They had better be the same length
if len(tasks_2) != len(tasks_1):
	print "Error: sets of tasks in files being compared are not the same length"
	sys.exit(1)

list1, list2 = register_lists(tasks_1, tasks_2)

#Set up a dictionary of counts
names = ['drag', 'voice_command', 'tap', 'lasso', 'pinch', 'box_select', 'ui', 'other', "MISS"]

counts = {}
	for firstName in names:
		counts[firstName] = {}
		for secondName in names:
			counts[firstName][secondName] = 0

#For each event, add it to the counts
for ev1, ev2 in zip(list1, list2):
	counts[ev1['event_type']][ev2['event_type']] += 1

#Print the counts out in a way that's useful for copy/paste to excel for calculation
#First the top row
print "," + ",".join(names)
#Then each row with its values
for firstName in names:
	line = firstName + ","
	values = []
	for secondName in names:
		values.append(str(counts[firstName][secondName]))
	line += ",".join(values)
	print line


	

