#!/usr/bin/python

#Compare two .json files coding for the same events, calculate Cohen's Kappa for the two coders

import json
import argparse
import sys

def load_file(fname):
	with open(fname, "r") as infile:
		data = json.loads(infile.read())
		return data

def print_event_list(event_list):
	for event in event_list:
		print event['event_type'],
		if 'description' in event.keys():
			print "({0})".format(" ".join(event['description'])),
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

	if len(events_1) != len(events_2):
		print "Error: the event lists are different lengths for task {0}".format(task1)	
		print "{0}\n\t".format(args.first[0]),
		print_event_list(events_1)
		print "{0}\n\t".format(args.second[0]),
		print_event_list(events_2)
		#TODO this should probably be fatal, leaving non-fatal for debugging
		#continue

	#At this point we have two lists of events, of the same length, describing the same real-world happenings
	#Sort the lists by time
	events_1 = sorted(events_1, key=lambda event: event['time'])
	events_2 = sorted(events_2, key=lambda event: event['time'])

	#We already checked that the event lists are the same length, but zip will end when the 
	#shorter list ends (rather than e.g. spewing tuples like ({event}, None)).
	#This builds a matrix of agreement and disagreement
	agree_matrix = {}
	#These are used to calculate Cohen's Kappa 
	event_counts_1 = {}
	event_counts_2 = {}

	for event1, event2 in zip(events_1, events_2):
		event_type1 = event1['event_type']
		event_type2 = event2['event_type']

		#Store counts of individual events
		if event_type1 in event_counts_1.keys():
			event_counts_1[event_type1] += 1
		else:
			event_counts_1[event_type1] = 1

		if event_type2 in event_counts_2.keys():
			event_counts_2[event_type2] += 1
		else:
			event_counts_2[event_type2] = 1

		#Store agreements or disagreements
		if event_type1 in agree_matrix.keys():
			if event_type2 in agree_matrix[event_type1].keys():
				#This pair has come up before
				agree_matrix[event_type1][event_type2] += 1
			else:
				#Event type 2 hasn't been seen with event type 1 before
				agree_matrix[event_type1][event_type2] = 1
		else:
			#First time event type 1 has been seen, and so clearly the first time event type 2 has been seen with it
			agree_matrix[event_type1] = {}
			agree_matrix[event_type1][event_type2] = 1

	#Now let's calculate Cohen's Kappa
	event_count = len(events_1) #Or len(events_2), they're the same

	#Proportionate agreement = total agreements between two coders divided by number of events
	agreement_count = 0
	for event1 in agree_matrix.keys():
		if event1 in agree_matrix[event1].keys():
			agreement_count += agree_matrix[event1][event1]
	pO = float(agreement_count)/float(event_count)


	#Probability of random agreement = sum over all actions: the chance that the first rater would pick this action at random, 
	# times the probability that the second rater would pick this action at random. 
	# First, convert the number of times each rater picked a specific event into it's proportion of the total events
	for event in event_counts_1.keys():
		event_counts_1[event] = float(event_counts_1[event])/float(event_count)
	for event in event_counts_2.keys():
		event_counts_2[event] = float(event_counts_2[event])/float(event_count)

	#Now convert that into one dictionary of how likely they are to agree, which can include some zero entries
	#because if one rater never picked an event, they have 0 probability of agreeing with the other rater on it
	#print event_counts_1, event_counts_2
	all_events = set(event_counts_2.keys())
	all_events = all_events.union(set(event_counts_1.keys()))
	all_events = list(all_events)
	
	agree_probability = {}
	for event in all_events:
		if event in event_counts_2.keys() and event in event_counts_1.keys():
			agree_probability[event] = event_counts_2[event] * event_counts_1[event]
		else:
			agree_probability[event] = 0

	#The probability of agreement on _any_ coding is the sum of the individual probabiliites of agreement on _each_ coding
	pE = sum(agree_probability.values())

	#Now let's REALLY calculate Cohen's Kappa
	k = 0
	if pE == 1:
		#There was only one event type across both coders
		k = 1
	else:
		k = (pO - pE)/(1-pE)
	print k
	print "---"



	
