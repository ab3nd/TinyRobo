#!/usr/bin/python

#Uses the coding annotation of example gestures to figure out the time ranges 
#for user gestures that are examples, and removes those gestures from the 
#bagfile. 

#Load the all_participant data and get the times of the examples out of it
import json

infile = "/home/ams/TinyRoboData/all_participants.json"

with open(infile, 'r') as all_data:
	data = json.loads(all_data.read())

#Build a lookup table for getting the times of examples in a task
examples = {}

#For each participant
for prt in data.keys():
	#Create a per-participant dict of tasks, indexed by task number
	examples[prt] = {}
	#For each task
	for task in data[prt]['tasks'].keys():
		#For keeping track of spans of multiple examples
		start_time = None
		end_time = None

		print prt, task,
		#Create a list of all the example code times in this task
		examples[prt][task] = []

		gst = data[prt]['tasks'][task]
		for gesture in gst:
			if 'example' in gesture.keys():
				if gesture['example']:
					print ".",
					if start_time is None:
						#We are starting a possible span of examples
						start_time = gesture['time']
					#Update the end of the span of examples
					end_time = gesture['time']

				else:
					if start_time is not None:
						#We have seen examples before this gesture, save 
						#them and reset for the next pass
						examples[prt][task].append((start_time, end_time))
						start_time = end_time = None

					print "|",
		#If at the end of the gestures, the start and end times are set,
		#this sequence ended with a span of examples, so store them too
		if start_time is not None:
			examples[prt][task].append((start_time, end_time))
			start_time = end_time = None		
		print ""
		print examples[prt][task]

