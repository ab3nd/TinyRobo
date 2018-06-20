#!/usr/bin/python

# Script for classifying gestures at a higher level
# For now just does whether gesture is a selection or is a position (of robots) gesture
# These are stored as is_selection and is_position flags on the action in the all user data json file

import cmd
import all_data_handler
import re

def tag_object(original):
	tags = []
	#Matches robots, robot, bot, bots, etc. 
	robots = re.compile("bot|group|swarm|orange|red", re.I)
	crate = re.compile("crate", re.I)
	targetA = re.compile("area a|box a| a$|to a,|^a$", re.I)
	targetB = re.compile("area b|box b| b$|to b,|^b$", re.I)
	whitespace = re.compile("whitespace|ground|screen|white area", re.I)
	
	toCheck = [(robots, "robot"), (crate, "crate"), (targetA, "area a"), (targetB, "area b"), (whitespace, "whitespace")]
	for compiled, tag in toCheck:
		if re.search(compiled, original):
			tags.append(tag)
	return tags

if __name__ == '__main__':

	#Get a dict, by participant, to a list of all their actions
	ud = all_data_handler.UserData()

	#Make a list of tasks when passed a participant
	def task_list(p):
		return p["tasks"].keys()
	
	# Create a dict of users and their tasks
	user_tasks = ud.apply(task_list)

	selections = ['tap', 'lasso', 'box_select']
	objects = ['robot', 'crate']
	places = ['whitespace', 'area a', 'area b']
	other_cmds = ['voice_command', 'other']

	#Store the user, task, event, and whether it is selection or position (or both or neither)
	results = []

	for user in user_tasks.keys():
		for task in user_tasks[user]:
			#Prettyprint each event
			for event in ud.data[user]["tasks"][task]:
				#Skip all example events
				if "example" in event.keys() and event["example"]:
					continue
				#Handle as many of the events as possible without human intervention
				if event["event_type"] in selections:
					tags = tag_object(" ".join(event["objects"]).strip("\""))
					#See if any of the objects of this selection are interactable
					if any(t for t in tags if t in objects):
						#This is a selection gesture
						is_selection = True
						is_position = False
					elif any(t for t in tags if t in places):
						#This is a position gesture
						is_selection = False
						is_position = True
					else:
						#Can't tell, user will have to classify this one
						desc_str = "{} {}".format(event["event_type"], " ".join(event["objects"]))
						is_selection, is_position = classify(desc_str)
				elif event["event_type"] == "drag":
					tags = tag_object(" ".join(event["objects"]).strip("\""))
					if event["draw"] is None:
						#This is a simple drag
						if any(t for t in tags if t in objects):
							#This is moving a thing, so position
							is_selection = False
							is_position = True
						elif any(t for t in tags if t in places):
							#This is creating a path, so position
							is_selection = False
							is_position = True
						else:
							#Can't tell, user will have to classify this one
							desc_str = "{} {}: {}".format(event["event_type"], " ".join(event["objects"]).strip("\""), tags)
							is_selection, is_position = classify(desc_str)
					else:
						#The user drew something
						#Dalton used draw when the user wasn't drawing anything, so I have to classify all of these
						desc_str = "draw {} over {}".format(" ".join(event["draw"]).strip("\""), " ".join(event["objects"]).strip("\""))
						is_selection, is_position = classify(desc_str)
				elif event["event_type"] == "pinch":
					t = "pinch"
					if event["reverse"]:
						t = "reverse pinch"
					# print "{} {}".format(event["event_type"], " ".join(event["objects"]).strip("\""))
					is_selection, is_position = classify(desc_str)
				elif event["event_type"] == "ui":
					#There are few enough UI events that I can just classify them by hand
					desc_str = "{} {}".format(event["event_type"], " ".join(event["description"]).strip("\""))
					is_selection, is_position = classify(desc_str)
				elif event["event_type"] == "voice_command":
					#There are few enough voice commands that I can just classify them by hand
					desc_str = "{} {}".format(event["event_type"], " ".join(event["command"]).strip("\""))
					is_selection, is_position = classify(desc_str)
				elif event["event_type"] == "other":
					#There are few enough other interactions that I can just classify them by hand
					desc_str = "{} {}".format(event["event_type"], " ".join(event["description"]).strip("\""))
					is_selection, is_position = classify(desc_str)
				elif event["event_type"] == "memo":
					#Skip it
					pass
				else:
					print event #Shouldn't happen
				
				#Kind of poor form to write to another object's data...
				event["is_selection"] = is_selection
				event["is_position"] = is_position