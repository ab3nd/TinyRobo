#!/usr/bin/python

#Uses the coding annotation of example gestures to figure out the time ranges 
#for user gestures that are examples, and removes those gestures from the 
#bagfile. 

#Load the all_participant data and get the times of the examples out of it
import json
import rosbag

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

		#print prt, task,

		#Create a list of all the example code times in this task
		examples[prt][task] = []

		gst = data[prt]['tasks'][task]
		for gesture in gst:
			if 'example' in gesture.keys():
				if gesture['example']:
					#print ".",
					if start_time is None:
						#We are starting a possible span of examples
						start_time = gesture['time']
					#Update the end of the span of examples
					end_time = gesture['time']

				else:
					#print "|",
					if start_time is not None:
						#We have seen examples before this gesture, save 
						#them and reset for the next pass
						examples[prt][task].append((start_time, end_time))
						start_time = end_time = None


		#If at the end of the gestures, the start and end times are set,
		#this sequence ended with a span of examples, so store them too
		if start_time is not None:
			examples[prt][task].append((start_time, end_time))
			start_time = end_time = None		
		#print ""
		#print examples[prt][task]

#Generate bagfile names using the same schema as bag_chop.py, load them up, 
#and only keep the contacts that are not during the example periods

#For mapping participant numbers to conditions
cmap = {0:'X',1:'1',2:'10',3:'100',4:'1000'}

for prt in examples.keys():
	for task in examples[prt].keys():
		cond = cmap[((int(prt)-1) % 5)]

		#Load up the target bag and the bag to clean up
		name = "user-{}_cond-{}_task-{}.bag".format(prt, cond, task)
		outBagName = "user-{}_cond-{}_task-{}_no_example.bag".format(prt, cond, task)
		bag = rosbag.Bag(name)
		outBag = rosbag.Bag(outBagName, 'w')


		for topic, msg, t in bag.read_messages():
			if topic == "/touches":
				import pdb; pdb.set_trace()
				#TODO need to do a time check in here
				#Actually, first pass through, get all the strokes that are occuring in the time range, and get their IDs
				#next pass, remove anything with the ID of a c u r s e d   s t r o k e
				if outBag is not None:
					outBag.write(topic, msg, t)

