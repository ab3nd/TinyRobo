#!/usr/bin/python

# General tool for working with the big json file of all participants

import json

class UserData(object):
	def __init__(self):
		self.infileName = "./all_participants.json"
		self.data = None
		with open(self.infileName, 'r') as infile:
			self.data = json.loads(infile.read())

	#Return a dict of participants to the results of running 
	#the function on that participant. Functions should deal with 
	#a single participant
	def apply(self, function):
		ret = {}
		for participant in self.data.keys():
			ret[participant] = function(self.data[participant])
		return ret

	#TODO Apply over a condtion
	#TODO Apply over a task
	#These could be written in terms of a filter on all of the participants, 
	#followed by an application on those participants that pass the filter

	#Map of tasks to task numbers
	# task 								1	10	100	1000	Unknown
	# Move to A							1	1	1	1		1
	# Move to A with wall				2	2	2	2		2
	# Stop the robots					3	3	3	3		3
	# Divide around obstacle				4	4	4		4
	# Orange to B, red to A				4	5	5	5		5
	# Orange to A, red to B				5	6	6	6		6
	# Orange to A, red to B (mixed)		6	7	7	7		7
	# Divide group						7	8	8	8		8
	# Merge group							9	9	9		9
	# Form a line							10	10	10		10
	# Form a square							11	11	11		11
	# Move crate to A					8	12	12	12		12
	# Move crate to A (dispersed)		9	13	13	13		13
	# Mark defective					10	14	13	14		
	# Remove defective					11	15	14	15		
	# Patrol screen						12	16	16	16		14
	# Patrol A							13	17	17	17		15
	# Disperse							14	18	18	18		16