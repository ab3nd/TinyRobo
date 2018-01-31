#!/usr/bin/python

#Compare two .json files coding for the same events, calculate Krippendorff's alpha for them
import json
import argparse
import sys
import krippendorff
import numpy as np

def load_file(fname):
	with open(fname, "r") as infile:
		data = json.loads(infile.read())
		return data

def print_event_list(event_list):
	for event in event_list:
		print "\t{0}".format(event['event_type']),
		if 'description' in event.keys():
			print "({0})".format((" ".join(event['description'])).encode("utf-8")),
		if event['example']:
			print "<=== EXAMPLE",
		print ""
	print "--"


def print_2d_array(arr):
	h = len(arr)
	w = len(arr[0])
	for y in range(h):
		for x in range(w):
			print "{:6,.4}".format(float(arr[y][x])),
		print " "

def print_2d_str_array(arr):
	h = len(arr)
	w = len(arr[0])
	for y in range(h):
		for x in range(w):
			print " {0}".format(arr[y][x]),
		print " "

def time_err(a1, a2):
	total_error = 0
	#These arrays are the same length
	for event1, event2 in zip(a1,a2):
		if event1 is None or event2 is None:
			continue
		else:
			total_error += abs(event1['time'] - event2['time'])
	return total_error

#The only two arguments are the file names
parser = argparse.ArgumentParser()
parser.add_argument("first", nargs = 1, help="First file, from one coder")
parser.add_argument("second", nargs = 1, help="Second file, from other coder")
args = parser.parse_args()

#Load both files, the [0] is because they're lists
data_1 = load_file(args.first[0])
data_2 = load_file(args.second[0])

#For each task, get a the list of events
tasks_1 = sorted(data_1['tasks'].keys(), key=int)
tasks_2 = sorted(data_2['tasks'].keys(), key=int)

#They had better be the same length
if len(tasks_2) != len(tasks_1):
	print "Error: sets of tasks in files being compared are not the same length"
	sys.exit(1)

#Save all the codes in one array, gets reshaped in R
codes = []

#Get the participant number
participant = data_1["participant"]
print participant

#Generate an empty confusion matrix
confusion = {} 
names = ['drag', 'voice_command', 'tap', 'lasso', 'pinch', 'box_select', 'ui', 'other', "MISS"]
for firstName in names:
	confusion[firstName] = {}
	for secondName in names:
		confusion[firstName][secondName] = 0

for task1, task2 in zip (tasks_1, tasks_2):
	#Make sure we're comparing the same tasks
	if task1 != task2:
		print "Error: cannot compare task {0} with task {1}".format(task2, task1)
		sys.exit(1)

	#Discard memos, they don't contribute to K
	events_1 = [x for x in data_1['tasks'][task1] if x['event_type'] != 'memo']
	events_2 = [x for x in data_2['tasks'][task2] if x['event_type'] != 'memo']
	
	#At this point we have two lists of events, of the same length, describing the same real-world happenings
	#Sort the lists by time
	events_1 = sorted(events_1, key=lambda event: event['time'])
	events_2 = sorted(events_2, key=lambda event: event['time'])
 	
 	#The lists are sorted, but may not be aligned. Alignment in this case means that each item in the longer list is paired 
 	#with an item in shorter list, or with no item, because the item is something one coder saw and the other coder missed. 
 	#The pairing should be such that it minimizes the total time difference between paired items (so that each pair consists
 	#of items as close together as possible)

 	#Dynamic time warping doesn't work because it can map multiple points to one point, but I need actual pairs
 	#Needleman-Wunsch doesn't work because it can insert arbitrary numbers of indels, and I have a bounded number

 	#Can get away with brute forcing this because it's usually small sets
 	import itertools
 	import copy
	
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

 	for idx in itertools.combinations(range(len(longlist)), len(shortlist)):
		with_gaps = [None for _ in longlist]

		for from_idx, to_idx in enumerate(idx):
			with_gaps[to_idx] = shortlist[from_idx] 

		#Check if the time error is less than the minimum observed so far	
		t = time_err(with_gaps, longlist) 
		if t < min_err_t:
			min_err_t = t
			best_match = with_gaps #TODO is this a potential scoping problem?

	#Print out which task
	print task1

 	#They're the same length because we put some gaps in
 	for event1, event2 in zip(best_match, longlist):
 		#Handle missing events by creating a dummy event for the miss
 		if event1 is None:
 			event1 = {"time":0.0, "event_type":"MISS"}
 		if event2 is None:
 			event2 = {"time":0.0, "event_type":"MISS"}

 		isExample = " "
 		if "example" in event1.keys() and event1["example"]:
				isExample = "X"
 		event1_summary = "\t{0}\t{1} {2}".format(event1['time'], event1['event_type'], isExample)

		isExample = " "
 		if "example" in event2.keys() and event2["example"]:
				isExample = "X"
 		event2_summary = "\t{0}\t{1} {2}".format(event2['time'], event2['event_type'], isExample)

 		delta_T = event1["time"] - event2["time"]
 		#Swap print order based on who had the short codes
 		#This keeps the event columns over the whole file in the same order
 		if coder_1_short:
 			print "\t{}\t{}\t{}".format(event1_summary, event2_summary, delta_T)
 			confusion[event1["event_type"]][event2["event_type"]] += 1
 		else:
 			print "\t{}\t{}\t{}".format(event2_summary, event1_summary, delta_T)
 			confusion[event2["event_type"]][event1["event_type"]] += 1

#Print the confusion matrix
#First the top row of names
print "\nConfusion"
print "\t",
for firstName in names:
	print "{}\t".format(firstName[:7]),
print ""

for firstName in names:
	values = "{}\t".format(firstName[:7])
	for secondName in names:
		values += "{}\t".format(confusion[firstName][secondName])
	print values
