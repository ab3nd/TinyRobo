#!/usr/bin/python

#Uses the coding annotation of example gestures to figure out the time ranges 
#for user gestures that are examples, and removes those gestures from the 
#bagfile. 

#Load the all_participant data and get the times of the examples out of it
import json

infile = "~/TinyRoboData/all_participants.json"

with open(infile, 'r') as all_data:
	data = json.loads(all_data.read())
	print data