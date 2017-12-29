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

for task1, task2 in zip (tasks_1, tasks_2):
	#Make sure we're comparing the same tasks
	if task1 != task2:
		print "Error: cannot compare task {0} with task {1}".format(task2, task1)
		sys.exit(1)

	#Discard memos, they don't contribute to K
	events_1 = [x for x in data_1['tasks'][task1] if x['event_type'] != 'memo']
	events_2 = [x for x in data_2['tasks'][task2] if x['event_type'] != 'memo']

	#if len(events_1) != len(events_2):
		#print "Error: the event lists are different lengths for task {0}".format(task1)	
		#TODO this should probably be fatal, leaving non-fatal for debugging
		#continue

	#print "{0}\n".format(args.first[0]),
	#print_event_list(events_1)
	#print "{0}\n".format(args.second[0]),
	#print_event_list(events_2)
	
	#At this point we have two lists of events, of the same length, describing the same real-world happenings
	#Sort the lists by time
	events_1 = sorted(events_1, key=lambda event: event['time'])
	events_2 = sorted(events_2, key=lambda event: event['time'])

 	event_length = max(len(events_1), len(events_2))
 	#This is totally arbitrary, the krippendorf library just can't use strings
 	translate_to_int = {'drag':9, 'voice_command':1, 'tap':2, 'lasso':3, 'pinch':4, 'box_select':5, 'ui':6, 'memo':7, 'other':8}

 	#Set up the codings, with np.nan for missing events
 	#This isn't accurate, since the missing events don't have to be at the _end_ of the list...
 	coder_1 = []
 	coder_2 = []
 	for ii in range(event_length):
 		try:
 			coder_1.append(translate_to_int[events_1[ii]['event_type']])
 		except IndexError:
 			coder_1.append(np.nan)

 		try:
 			coder_2.append(translate_to_int[events_2[ii]['event_type']])
 		except IndexError:
 			coder_2.append(np.nan)

 	k_data = [coder_1, coder_2]
 	#print k_data
 	print "---"
 	print krippendorff.alpha(reliability_data=k_data, level_of_measurement='nominal')
 		