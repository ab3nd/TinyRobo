#!/usr/bin/python

#Normalize use of words like "crate", "whitespace", "robots", etc. in object declarations
#Possibly through tagging gestures with a normalized tag rather than changing description

#Given a set of user gestures with normalized tags, build a stack representing the data

#Possibly reduce the stack by compacting neighboring matching gestures with the same object

#Calculate edit distance on stacks to de-duplicate strategies (based on observation)
#This is a point where I can encode my heuristics

#Cluster stacks based on edit distance. How does it correlate with my tagged strategies
#This is a point where I can try to resist my interest in machine learning


import all_data_handler

users = all_data_handler.UserData()


def Ngrammer(object):
	def __init__(self):
		self.texts = []
		#memoization of n-grams, keyed by N
		self.grams = {}

	#For all the texts, build the n-grams
	def get_n_grams(n):
		if n in self.grams.keys():
			return self.grams[n]
		else:
			#This is where we do the actual N-gramming

	#Add a single line of text to this grammer
	def add_text(t):
		self.texts.append(t)
		#Adding text invalidates any stored n-grams
		self.grams = {}


#Debugging code, set up a grammer, feed it, reap its innards
testGrammer = Ngrammer()
testGrammer.add_text("Let them eat cake")
testGrammer.add_text("The cake is a lie.")
testGrammer.add_text("Now it begins")
testGrammer.add_text("Red robot to area A")
testGrammer.add_text(None)
testGrammer.add("Short")

t1 = testGrammer.get_n_grams(1)
t2 = testGrammer.get_n_grams(2)
t3 = testGrammer.get_n_grams(3)

# #Build seperate sets of N-grams for objects, description, and draw commands
# objGrams = Ngrammer()
# drawGrams = Ngrammer()
# descGrams = Ngrammer()

# for participant in users.data.keys():
# 	for task in users.data[participant]["tasks"].keys():
# 		for event in task:
# 			#Skip memos
# 			if event["event_type"] == "memo":
# 				continue

# 			#Get objects for most events
# 			if "objects" in event.keys():
# 				obj = event["objects"]
# 				objGrams.add_text(obj)

# 			#Get description for "other" events, memo was already skipped
# 			if "description" in event.keys():
# 				desc = event["description"]
# 				descGrams.add_text(desc)

# 			#Get what was drawn for drag-draw events
# 			if "draw" in event.keys():
# 				draw = event["draw"] #TODO can be None, handle gracefully
# 				drawGrams.add_text(draw)

# #Loaded everything, generate word list (1-grams), 2-grams, and 3-grams
# for grammer, name in [(objGrams, "objects"), (drawGrams, "draw"), (descGrams, "desc")]:
# 	oneGrams = grammer.get_n_grams(1)
# 	twoGrams = grammer.get_n_grams(2)
# 	threeGrams = grammer.get_n_grams(3)

	#TODO print them out nicely