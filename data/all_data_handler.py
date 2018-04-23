#!/usr/bin/python

# General tool for working with the big json file of all participants

import json

class UserData(object):
	def __init__(self):
		self.infileName = "./all_participants.json"
		self.data = None
		with open(self.infileName, 'r') as infile:
			self.data = json.loads(infile.read())
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
		self.taskMap = {
			"move_a":{1:1, 10:1, 100: 1, 1000:1, 'X':1},
			"move_wall":{1:2, 10:2, 100:2, 1000:2, 'X':2},
			"stop":{1:3, 10:3, 100:3, 1000:3, 'X':3},
			"divide":{1:None, 10:4, 100:4, 1000:4, 'X':4},
			"divide_color_1":{1:4, 10:5, 100:5, 1000:5, 'X':5},
			"divide_color_2":{1:5, 10:6, 100:6, 1000:6, 'X':6},
			"divide_color_mix":{1:6, 10:7, 100:7, 1000:7, 'X':7},
			"split":{1:7, 10:8, 100:8, 1000:8, 'X':8},
			"merge":{1:None, 10:9, 100:9, 1000:9, 'X':9},
			"line":{1:None, 10:10, 100:10, 1000:10, 'X':10},
			"square":{1:None, 10:11, 100:11, 1000:11, 'X':11},
			"crate":{1:8, 10:12, 100:12, 1000:12, 'X':12},
			"crate_dispersed":{1:9, 10:13, 100:13, 1000:13, 'X':13},
			"mark":{1:10, 10:14, 100:14, 1000:14, 'X':None},
			"remove":{1:11, 10:15, 100:15, 1000:15, 'X':None},
			"patrol_screen":{1:12, 10:16, 100:16, 1000:16, 'X':14},
			"patrol_a":{1:13, 10:17, 100:17, 1000:17, 'X':15},
			"disperse":{1:14, 10:18, 100:18, 1000:18, 'X':16}
		}

		#The value is the participant number mod 5
		self.conditionMap = {
			"one":2,
			"ten":3,
			"hundred":4,
			"thousand":0,
			"unknown":1
		}
	#Return a dict of participants to the results of running 
	#the function on that participant. Functions should deal with 
	#a single participant
	def apply(self, function):
		ret = {}
		for participant in self.data.keys():
			ret[participant] = function(self.data[participant])
		return ret

	#Apply over a condtion, function takes a participant
	def applyCondition(self, function, condition):
		ret = {}
		for pID in self.data.keys():
			if (self.data[pID]["participant"] % 5) == self.conditionMap[condition]:
				ret[pID] = function(self.data[pID])
		return ret

		
	#Apply over a _task_, not over a participant
	#So the function takes a task
	def applyTask(self, function, task):
		ret = {}
		taskNums = self.taskMap[task]
		for pId in self.data.keys():
			#Convert the task name to a number 
			taskNum = None
			if self.data[pId]['participant'] % 5 == self.conditionMap['one']:
				taskNum = taskNums[1]
			elif self.data[pId]['participant'] % 5 == self.conditionMap['ten']:
				taskNum = taskNums[10]
			elif self.data[pId]['participant'] % 5 == self.conditionMap['hundred']:
				taskNum = taskNums[100]
			elif self.data[pId]['participant'] % 5 == self.conditionMap['thousand']:
				taskNum = taskNums[1000]
			elif self.data[pId]['participant'] % 5 == self.conditionMap['unknown']:
				taskNum = taskNums['X']
			else:
				print "Bad participant number {}".format(pId)

			#This participant had this task, so call the function on it
			if taskNum is not None:
				ret[pId] = function(self.data[pId][taskNum])



