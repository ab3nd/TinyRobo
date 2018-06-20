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
	for user in user_tasks.keys():
		for task in user_tasks[user]:
			#Prettyprint each event
			for event in ud.data[user]["tasks"][task]:
				#Handle as many of the events as possible without human intervention
				if event["event_type"] in selections:
					#import pdb; pdb.set_trace()
					tags = tag_object(" ".join(event["objects"]).strip("\""))
					#See if any of the objects of this selection are interactable
					if any(t for t in tags if t in objects):
						#This is a selection gesture
						#print "{} selection of {}".format(event["event_type"], tags)
						pass
					elif any(t for t in tags if t in places):
						#This is a selection gesture
						#print "{} position to {}".format(event["event_type"], tags)
						pass
					else:
						#import pdb; pdb.set_trace()
						print " ".join(event["objects"])
						print "----{}, {} {}".format(event["event_type"], tags, " ".join(event["objects"]))