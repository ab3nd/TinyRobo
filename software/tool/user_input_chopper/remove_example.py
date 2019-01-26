#!/usr/bin/python

#Uses the coding annotation of example gestures to figure out the time ranges 
#for user gestures that are examples, and removes those gestures from the 
#bagfile. 

#Load the all_participant data and get the times of the examples out of it
import json
import rosbag
import rospy
import os
import fnmatch
import yaml

#From https://stackoverflow.com/questions/1724693/find-a-file-in-python
def find(pattern, path):
    result = []
    for root, dirs, files in os.walk(path):
        for name in files:
            if fnmatch.fnmatch(name, pattern):
                result.append(os.path.join(root, name))
    return result


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

	cond = cmap[((int(prt)-1) % 5)]

	#Get the start time of the bag from the original bagfile
	originalPath = "/home/ams/TinyRoboData/Experiment_Run_Bags/p{0}/".format(prt)
	fnamePattern = "id_{0}_cond_{1}_*.bag".format(prt, cond)

	originalFile = find(fnamePattern, originalPath)
	print originalFile

	if not len(originalFile) > 0:
		continue

	#Load the info from it so we can get the real start time
	info = yaml.load(rosbag.Bag(originalFile[0], 'r')._get_yaml_info())
	
	#get the real start time 
	#originalStart = rospy.Time.from_sec(info['start'])
	originalStart = info['start']

	for task in examples[prt].keys():
		
		#Load up the bag to clean up and a new bag to put the results in 
		name = "user-{}_cond-{}_task-{}.bag".format(prt, cond, task)
		outBagName = "user-{}_cond-{}_task-{}_no_example.bag".format(prt, cond, task)
		bag = rosbag.Bag(name)
		outBag = rosbag.Bag(outBagName, 'w')

		#Get the time ranges that are blocked
		blockedTimes = examples[prt][task]

		purgeStrokes = []
		for topic, msg, t in bag.read_messages():
			if topic == "/touches":

				#Convert the time of this message to a coding time by subtracting
				#the bag start time from it
				asCodeTime = t.to_sec() - originalStart

				#if the message overlaps a blocked time, add its frame id to the 
				#list of strokes to purge
				for blt in blockedTimes:
					start = blt[0]
					end = blt[1]
					if abs(asCodeTime - start) < 0.2 or abs(asCodeTime-end) < 0.2:
						#It's near the start or end time
						purgeStrokes.append(msg.header.frame_id)
					elif start <= asCodeTime <= end:
						#It's between the start and end times
						purgeStrokes.append(msg.header.frame_id)


		for topic, msg, t in bag.read_messages():
			if topic == "/touches":
				if msg.header.frame_id not in purgeStrokes:
					if outBag is not None:
						outBag.write(topic, msg, t)

