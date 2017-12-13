#!/usr/bin/python

#Compare two .json files coding for the same events, calculate Cohen's Kappa for the two coders

import json
import argparse
import sys

def load_file(fname):
	with open(fname, "r") as infile:
		data = json.loads(infile.read())
		return data

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

for tasks in zip (tasks_1, tasks_2):
	#Make sure we're comparing the same tasks
	if tasks[0] != tasks[1]:
		print "Error: cannot compare task {0} with task {1}".format(tasks[0],tasks[1])
		sys.exit(1)

	#Discard memos, they don't contribute to K
	events_1 = [x for x in data_1['tasks'][tasks[0]] if x['event_type'] != 'memo']
	events_2 = [x for x in data_2['tasks'][tasks[1]] if x['event_type'] != 'memo']

	if len(events_1) != len(events_2):
		print "Error: the event lists are different lengths for task {0}".format(tasks[0])	
		#TODO this should probably be fatal, leaving non-fatal for debugging
		continue

	#At this point we have two lists of events, of the same length, describing the same real-world happenings
	#So calculate Cohen's Kappa on them. 
	

	import pdb; pdb.set_trace()
	
