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
import numpy as np

class imageLogger():
	def __init__(self, path=None, suffix=None):
		#Create a new image
		#TODO base this on the size of the screen or the output available
		if path is None:
			self.img = Image.new("RGB", (1000,750))
		else:
			self.img = Image.open(path)

		
		#Set up a draw object for the image
		self.draw = ImageDraw.Draw(self.img)

		#Set up a list to store events
		self.logged_events = []

		#File name to store output in
		if path is None and suffix is None:
			self.outfile = "{0}.png".format(uuid.uuid4())
		elif path is None:
			self.outfile = "{0}_{1}.png".format(uuid.uuid4(), suffix)
		elif suffix is None:
			#The find-based cut throws away a relative path so the image ends up in this directory
			self.outfile = "{0}_{1}.png".format(uuid.uuid4(), path[path.find('-')+2:-4])
		else:
			self.outfile = "{0}_{1}.png".format(path[path.find('-')+2:-4], suffix)
		print self.outfile

	def finish(self):
		#TODO This is where we'd do something clever for event detection

		#Set up the colors of the logged events by time
		color_length = len(self.logged_events)/3
		reds = np.linspace(0,255, color_length, dtype=int)
		reds = np.append(reds[:-1], np.fliplr([reds])[0])
		reds = np.append(reds, np.zeros(2*color_length, dtype = int))
		greens = np.roll(reds, color_length)
		blues = np.roll(greens, color_length)
		#Combine them all
		colors = zip(reds, greens, blues)

		#Draw all of the logged events as dots
		for event in self.logged_events:
			x = event["event_x"]
			y = event["event_y"]
			#Draw rectangular events as rectangles instead of circles
			if event["shape"] is not None:
				h = event["shape"]["height"]
				w = event["shape"]["width"]
				self.draw.rectangle([(x-(w/2), y-(h/2)), (x+(w/2), y+(h/2))], fill=colors.pop(0))
			else:
				self.draw.ellipse([(x-2,y-2),(x+2,y+2)], fill=colors.pop(0))

		#Write the image 
		#TODO do something smarter with file names
		self.img.save(self.outfile)

	#Kivy coordinates put (0,0) at the bottom left corner of the window
	#PIL puts (0,0) in the top left		
	def kivyToPIL(self, coords):
		x = coords[0]
		y = coords[1] #TODO I guess I could have a point class...
		maxX, maxY = self.img.size
		return (x, maxY - y) #Flip around horizontal center of image
		
	def addEvent(self, event):
		if isMeta(event):
			pass #Ignore it
		else:
			#Convert event coordinates from Kivy to PIL coordinates
			x = int(event["event_x"])
			y = int(event["event_y"])
			event["event_x"], event["event_y"] = self.kivyToPIL((x,y))
			self.logged_events.append(event)

import pickle

infile = "26-08-17_16_43_57_s5_c10.pickle"

def isMeta(event):
	if "desc" in event.keys():
		return True
	return False

with open(infile, 'r') as inputData:

	#Cut the extension off the input file name
	outfile_suffix = infile[:-7]
	imlog = None

	try:
		event = pickle.load(inputData)
		while event is not None:
			if isMeta(event):
				#print "MetaEvent", event["desc"]
				if event["desc"].startswith("Loaded"):
					#Close the existing image logger, if any
					if imlog is not None:
						imlog.finish()
					
					#Get the path to the slide out of the event
					fpath = event['desc'].split()[1]
					#Start a new logger with that path
					imlog = imageLogger(path=fpath, suffix=outfile_suffix)
			else:
				#print "Event"
				imlog.addEvent(event)
			event = pickle.load(inputData)
	except EOFError:
		print "All done"
	
	imlog.finish()
