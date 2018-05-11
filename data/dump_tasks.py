#!/usr/bin/python

# Dump all of the JSON data, broken down by task, to try to figure out what 
# gestures were used per task, percentage-wise by users. 

import all_data_handler

users = all_data_handler.UserData()

def convertTime(time):
	#Timecodes are in seconds, convert to minutes/seconds
	minutes = int(time/60)
	seconds = time - (minutes*60)
	return "{0}:{1}".format(minutes, seconds)

for task in users.taskMap.keys():
	#Flip through all the conditions 
	for condition in [1,10,100,1000,'X']:	
		#Only get the tasks where that condition actually had the task
		#e.g. 1-robot doesn't have "disperse"
		if users.taskMap[task][condition] is not None:
			#Print a nice header and underline it with = signs
			header = "{0} - {1} robot(s)".format(task, condition)
			print "\n{0}".format(header)
			print "=" * len(header)

			#Work only with the users in this task and condition
			for pID in users.data.keys():
				if users.inCondition(pID, condition):
					print "\n{0}".format(pID)
					print "-" * len(str(pID))
					gestures = users.data[pID]["tasks"][str(users.taskMap[task][condition])]
					for g in gestures:
						#print g["event_type"]
						if g["event_type"] == "voice_command":
							print "\tvoice {0} {1}".format(" ".join(g["command"]), convertTime(g["time"]))
						elif g["event_type"] == "memo":
							print "\t\tmemo -> {0}".format(" ".join(g["description"]))
						elif g["event_type"] == "ui":
							print "\tui {0} {1}".format(" ".join(g["description"]), convertTime(g["time"]))
						elif g["event_type"] == "other":
							print "\tother {0} {1} {2}".format(" ".join(g["description"]), " ".join(g["objects"]), convertTime(g["time"]))
						elif g["event_type"] == "drag" and g["draw"] is not None:
							print "\t{0} draw {1} {2} {3}".format(g["event_type"], " ".join(g["draw"]), " ".join(g["objects"]), convertTime(g["time"]))
						elif g["event_type"] == "pinch" and g["reverse"]:
							print "\treverse {0} {1} {2}".format(g["event_type"], " ".join(g["objects"]), convertTime(g["time"]))
						elif g["event_type"] == "tap" and g["count"] > 1:
							print "\t{0} ({3}) {1} {2}".format(g["event_type"], " ".join(g["objects"]), convertTime(g["time"]), g["count"])
						else:
							print "\t{0} {1} {2}".format(g["event_type"], " ".join(g["objects"]), convertTime(g["time"]))
