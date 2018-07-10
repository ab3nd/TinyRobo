#!/usr/bin/python
# Tag all the user actions based on class
# Most of this is automatable, since e.g. a box select with robots as the target is selection
# Final pass will also use cmd to let user set unclear tags

import json
import pprint
import re
import cmd

class EventTagger(cmd.Cmd):

	#Given the object list of a gesture from the coding, return a list of letters representing normalized 
	#values for the objects of the gesture
	def tag_object(self, original):
		#This use of strip is to prevent quotes from messing up regex matches
		original = " ".join(original).strip("\"")
		tags = []
		#Matches robots, robot, bot, bots, etc. 
		robots = re.compile("bot|group|swarm|orange|red", re.I)
		crate = re.compile("crate|wood", re.I)
		targetA = re.compile("area a|box a| a$|to a,|^a$", re.I)
		targetB = re.compile("area b|box b| b$|to b,|^b$", re.I)
		whitespace = re.compile("whitespace|ground|screen|white area", re.I)
		
		toCheck = [(robots, "r"), (crate, "c"), (targetA, "a"), (targetB, "b"), (whitespace, "w")]
		for compiled, tag in toCheck:
			if re.search(compiled, original):
				tags.append(tag)
		return tags



	def preloop(self):
		#Load the participant data
		infileName = "./all_participants_updated.json"
		self.data = None
		with open(infileName, 'r') as infile:
			self.data = json.loads(infile.read())

		#Load all the events into a big list
		self.events = []

		#They are strings, but sort by numerical value
		for user in sorted(self.data.keys(), key=lambda x: int(x)):
			for task in sorted(self.data[user]["tasks"].keys(), key=lambda x: int(x)):
				for event in self.data[user]["tasks"][task]:
					#Tag everything that has objects
					if "objects" in event.keys():
						event["obj_tags"] = self.tag_object(event["objects"])

					#Don't care about memos
					if event["event_type"] == "memo":
						continue
					#Check selection gestures for whether they are on robots or the crate
					elif event["event_type"] == "box_select" or event["event_type"] == "lasso":
						if 'r' in event["obj_tags"] or 'c' in event["obj_tags"]:
							event["is_select"] = True
					#Check selection gestures for whether they are on robots or the crate
					elif event["event_type"] == "tap":
						#If there's an r in tags, this was a selection gesture on a robot
						if 'r' in event["obj_tags"] or 'c' in event["obj_tags"]:
							event["is_select"] = True
						#If there was no other object, but was whitespace, this is a waypoint
						elif 'w' in event["obj_tags"]:
							event["is_position"] = True
					#Dragging robots or the crate is a position command
					elif event["event_type"] == "drag":
						if 'r' in event["obj_tags"] or 'c' in event["obj_tags"]:
							event["is_position"] = True
						


					elif "is_position" in event.keys() or "is_select" in event.keys() or "is_ui" in event.keys() or "is_meta" in event.keys():
						continue
					#Load the non-automatable ones
					else:
						self.events.append(event)
		#Set up for first prompt
		self.current_event = self.events.pop(0)
		pprint.pprint(self.current_event)


	def do_s(self, line):
		#mark as a selection event
		self.current_event["is_select"] = True

	def do_p(self, line):
		#mark as a position event
		self.current_event["is_position"] = True

	def do_u(self, line):
		#Mark as a ui event
		self.current_event["is_ui"] = True

	def do_m(self, line):
		#For things like cut gestures
		self.current_event["is_meta"] = True

	def do_q(self, line):
		#Quit
		return True

	def do_x(self, line):
		#Unknown, don't do anything
		pass

	def postcmd(self, stop, line):
		#Load the next event and prettyprint it
		self.current_event = self.events.pop(0)
		pprint.pprint(self.current_event)
		return stop
						
	def postloop(self):
		#Save the whole thing out to a new file
		outfileName = "./all_participants_updated.json"
		with open(outfileName, 'w') as outFile:
			outFile.write(json.dumps(self.data, sort_keys=True, indent=3))


    
if __name__ == '__main__':
    EventTagger().cmdloop()
