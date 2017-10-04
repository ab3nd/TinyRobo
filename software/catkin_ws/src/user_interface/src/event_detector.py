#!/usr/bin/python

# Script for graphing and otherwise handling the output from the UI test drive program

# Events are either events of the form:
# event = {"time": event.time_update,
#          "uid": event.uid,
#          "start_time" : event.time_start,
#          "end_time" : event.time_end,
#          "event_x" : event.x,
#          "event_y" : event.y,
#          "update_time" : event.time_update,
#          "shape" : event.shape}
# or metaevents of the form:
# event = {"time": time.time(),
#          "desc": desc}


# TODO
# Render images based on the points
# Try to detect events
#

import pickle

infile = "26-08-17_17_10_15_s555_c10.pickle"

def isMeta(event):
	if "desc" in event.keys():
		return True
	return False

with open(infile, 'r') as inputData:
	try:
		event = pickle.load(inputData)
		while event is not None:
			if isMeta(event):
				print "MetaEvent", event["desc"]
			else:
				print "Event", event
			event = pickle.load(inputData)
	except EOFError:
		print "All done"
	
