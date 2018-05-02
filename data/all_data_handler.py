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

	def taskNumberToName(self, number, pId):
		cond = self.IdToCondition(pId)[1]
		for name in self.taskMap.keys():
			if self.taskMap[name][cond] == number:
				return name
		#No match, return None
		return None

	def taskNameToNumber(self, task, pId):
		#Convert the task name to a number 
		taskNum = None
		taskNums = self.taskMap[task]
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
		return taskNum

	#Given a PID, return the condition that they were in
	def IdToCondition(self, pId):
		if self.data[pId]['participant'] % 5 == self.conditionMap['one']:
			return ('one', 1)
		elif self.data[pId]['participant'] % 5 == self.conditionMap['ten']:
			return ('ten', 10)
		elif self.data[pId]['participant'] % 5 == self.conditionMap['hundred']:
			return ('hundred', 100)
		elif self.data[pId]['participant'] % 5 == self.conditionMap['thousand']:
			return ('thousand', 1000)
		elif self.data[pId]['participant'] % 5 == self.conditionMap['unknown']:
			return ('unknown', "X")
		else:
			print "Bad participant number {}".format(pId)

	#Apply over a _task_, not over a participant
	#So the function takes a task
	def applyTask(self, function, task):
		ret = {}
		for pId in self.data.keys():
			taskNum = taskNameToNumber(task, pId)
			#This participant had this task, so call the function on it
			if taskNum is not None:
				ret[pId] = function(self.data[pId][taskNum])

	#Get a dictionary, by event type, of counts of that event within that task
	#for the given participant
	def getCounts(self, pId, task):
		gestures = {"drag":0,
				"draw":0,
				"ui":0,
				"tap":0,
				"doubletap":0,
				"tripletap":0,
				"hold":0,
				"pinch":0,
				"rev_pinch":0,
				"lasso":0,
				"box":0,
				"voice":0,
				"other":0}
		taskEvents = self.data[pId]["tasks"][task]
		for event in taskEvents:
		 	if event["event_type"] == "tap":
		 		#Taps need special handling, as they might be double, triple, or hold
		 		if event["hold"]:
		 			gestures["hold"] += 1
		 		elif event["count"] == 2:
		 			gestures["doubletap"] += 1
		 		elif event["count"] == 3:
		 			gestures["tripletap"] += 1
		 		else:
		 			gestures["tap"] += 1
			elif event["event_type"] == "drag":
				#Drags might be drag or might be draw
				if event["draw"] is None:
					gestures["drag"] += 1
				else:
					gestures["draw"] += 1
			elif event["event_type"] == "pinch":
				#pinch can be pinch or reverse
				if event["reverse"]:
					gestures["rev_pinch"] += 1
				else:
					gestures["pinch"] += 1
			elif event["event_type"] == "voice_command":
				gestures["voice"] += 1
			elif event["event_type"] == "ui":
				gestures["ui"] += 1
			elif event["event_type"] == "memo":
				#Don't do anything with memos
				pass
			elif event["event_type"] == "lasso":
				gestures["lasso"] += 1
			elif event["event_type"] == "box_select":
				gestures["box"] += 1
			elif event["event_type"] == "other":
				gestures["other"] += 1		
			else:
				#This is an error, some event type wasn't handled
				print event["event_type"]	
		return gestures

	#returns a dict of column names to lists of values representing the whole data set
	def toPandas(self):
		ret={}
		users = []
		task = []
		condition = []
		taps = []
		holds = []
		doubletaps = []
		tripletaps = []
		drags = []
		draws = []
		pinches = []
		reverse_pinches = []
		voices = []
		uis = []
		lassos = []
		boxes = []
		others = []
		for pId in self.data.keys():
			for taskID in self.data[pId]["tasks"].keys():
				#Get a task number from the task name
				taskName = self.taskNumberToName(int(taskID), pId)
				if taskID is not None:
					#Get the count of each gesture for this task
					counts = self.getCounts(pId, str(taskID))

					#Fill in all the fields
					users.append(pId)
					condition.append(self.IdToCondition(pId)[0])
					task.append(taskName)
					taps.append(counts["tap"])
					holds.append(counts["hold"])
					doubletaps.append(counts["doubletap"])
					tripletaps.append(counts["tripletap"])
					drags.append(counts["drag"])
					draws.append(counts["draw"])
					pinches.append(counts["pinch"])
					reverse_pinches.append(counts["rev_pinch"])
					voices.append(counts["voice"])
					uis.append(counts["ui"])
					lassos.append(counts["lasso"])
					boxes.append(counts["box"])
					others.append(counts["other"])
		
		#We've loaded all the data, now build the dict
		ret["user"] = users
		ret["task"] = task
		ret["condition"] = condition
		ret["box"] = boxes
		ret["doubletap"] = doubletaps
		ret["drag"] = drags
		ret["draw"] = draws
		ret["hold"] = holds
		ret["lasso"] = lassos
		ret["other"] = others
		ret["pinch"] = pinches
		ret["rev_pinch"] = reverse_pinches
		ret["tap"] = taps
		ret["tripletap"] = tripletaps
		ret["ui"] = uis
		ret["voice"] = voices

		return ret