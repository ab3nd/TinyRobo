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

import Image, ImageDraw
import uuid


class imageLogger():
	def __init__(self):
		#Create a new image
		#TODO base this on the size of the screen or the output available
		self.img = Image.new("RGB", (1000,750))
		self.draw = ImageDraw.Draw(self.img)

	def finish(self):
		#Write the image 
		#TODO do something smarter with file names
		self.img.save("{0}.png".format(uuid.uuid4()))
		
	def addEvent(self, event):
		if isMeta(event):
			pass #Ignore it
		else:
			x = int(event["event_x"])
			y = int(event["event_y"])
			print x, y
			self.draw.ellipse([(x-2,y-2),(x+2,y+2)], fill=(255,255,255))

import pickle

infile = "17-08-17_16:57:56_s666_c10.pickle"

def isMeta(event):
	if "desc" in event.keys():
		return True
	return False

with open(infile, 'r') as inputData:
	imlog = imageLogger()

	try:
		event = pickle.load(inputData)
		while event is not None:
			if isMeta(event):
				print "MetaEvent", event["desc"]
				if event["desc"].startswith("Advanced"):
					imlog.finish()
					imlog = imageLogger()
			else:
				print "Event"
				imlog.addEvent(event)
			event = pickle.load(inputData)
	except EOFError:
		print "All done"
	
	imlog.finish()
