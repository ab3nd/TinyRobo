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

import pickle

infile = "./stoups_pilot.pickle"

with open(infile, 'r') as inputData:
	pickle.load(inputData)
	import pdb; pdb.set_trace()