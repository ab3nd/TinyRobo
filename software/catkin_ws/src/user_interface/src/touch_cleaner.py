#!/usr/bin/python

# Script for recognizing gestures from a recording of user input
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


import Image, ImageDraw, ImageFont
import uuid
import numpy as np
import pickle
import math

#infile = "26-08-17_17-10-15_s555_c10.pickle"
#infile = "26-08-17_17_10_15_s555_c10.pickle"

def isMeta(event):
	if "desc" in event.keys():
		return True
	return False

def distanceEvents(eventA, eventB):
	x1 = eventA.point.x
	y1 = eventA.point.y
	x2 = eventB.point.x
	y2 = eventB.point.y

	return math.sqrt(pow(x1 - x2, 2) + pow(y1 -y2, 2))

def distance(x1, y1, x2, y2):
	return math.sqrt(pow(x1 - x2, 2) + pow(y1 -y2, 2))

def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

def dumpCommand(cmd):

	#Generate a file name from the IDs of all the strokes in this command
	fname = "{0}.png".format("_".join([str(x) for x in cmd.strokes.keys()]))

	#This is hardcoded because this is how big my images are. Stylistically not great. 
	width = 1680 #int(maxX - minX)
	height = 1050 #int(maxY - minY)

	#Create a new image with the required shape
	image = Image.new("RGB", (width, height))
	
	for stroke in cmd.strokes.keys():
		dumpStroke(cmd.strokes[stroke], fname, image)

#Debug function, dumps strokes to image files for viewing
def dumpStroke(stroke, fname=None, image=None):
	pxlPad = 16
	
	#Load parameters for the image, pad the width and height
	if fname == None:
		fname = "stroke_{0}.png".format(stroke.id)

	# If image is None, we're not dumping to an image that got created already
	if image is None:
		#Set up widths and heights for image
		width = 1680 #int(stroke.width()) + pxlPad
		height = 1050 #int(stroke.height()) + pxlPad
	
		#Create a new image with the required shape
		image = Image.new("RGB", (width, height))
	
	draw = ImageDraw.Draw(image)
	
#Set up the colors of the logged events by time
	color_length = (len(stroke.events) + 3)/3
	reds = np.linspace(0,255, color_length, dtype=int)
	reds = np.append(reds[:-1], np.fliplr([reds])[0])
	reds = np.append(reds, np.zeros(2*color_length, dtype = int))
	greens = np.roll(reds, color_length)
	blues = np.roll(greens, color_length)
	#Combine them all
	colors = zip(reds, greens, blues)

	#The event didn't have enoug points to bother with colors, it's probably a tap or doubletap
	#So do it all in black
	#if len(colors) == 0:
	#	colors = [(0,0,0)] * 3

	for event in stroke.events:
		#Convert to image coordinates
		x = int(event.point.x)
		#This flips the image, kivy coordinates are upside-down relative to PIL coordinates
		y = 750 - int(event.point.y)
		#Then move them from full screen space to the smaller image, including padding
		#x = x - stroke.minX + (pxlPad/2)
		#y = y - stroke.minY + (pxlPad/2)

		#Draw the event
		draw.ellipse([(x-3,y-3),(x+3,y+3)], fill=colors.pop(0))

	#Draw the stroke centroid
	#Convert to image coordinates
	x = int(stroke.centroid[0])
	#This flips the image, kivy coordinates are upside-down relative to PIL coordinates
	y = 750 - int(stroke.centroid[1])
	#Then move them from full screen space to the smaller image, including padding
	#x = x - stroke.minX + pxlPad/2
	#y = y - stroke.minY + pxlPad/2

	#Draw an X
	draw.line([(x-3,y-3),(x+3,y+3)], fill=(255, 0, 0))
	draw.line([(x-3,y+3),(x+3,y-3)], fill=(255, 0, 0))

	#font = ImageFont.truetype("DejaVuSansMono.ttf", 16)
	#Specify font path precisely if just the file name doesn't work
	font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 10)
		
	#Can't have an angle between less than two events
	if len(stroke.events) > 2:
		#Get the angle between the end points around the centroid
		#Start by getting the distances between the start and end events and the centroid
		x1 = stroke.events[0].point.x
		y1 = stroke.events[0].point.y
		x2, y2 = stroke.centroid
		d1 = distance(x1, y1, x2, y2)
		x1 = stroke.events[-1].point.x
		y1 = stroke.events[-1].point.y
		d2 = distance(x1, y1, x2, y2)
		#Distance between the start and end events
		d3 = distanceEvents(stroke.events[0], stroke.events[-1])
		#From the law of cosines and https://stackoverflow.com/questions/1211212/how-to-calculate-an-angle-from-three-points
		#This should be in radians, math.acos is in radians, according to the docs
		#value is capped at 1 because numerical imprecision was causing math domain errors by feeding
		#acos values like -1.00000000002, which is, technically, out of ranges
		textLocation = (stroke.centroid[0] + 5, 750 - (stroke.centroid[1] - 5))
		if (d1 * d2) == 0:
			draw.text(textLocation,"d1 or d2 was zero",(255,255,255),font=font)
			return
		capped = clamp((pow(d1, 2) + pow(d2, 2) - pow(d3, 2))/(2 * d1 * d2), -1, 1)
		angle = math.acos(capped)
		if angle < 1:
			draw.text(textLocation, "Circle ID:{0} ({1})".format(stroke.id, len(stroke.events)) ,(255,255,255),font=font)
		elif angle < 2.5:
			draw.text(textLocation, "Arc ID:{0} ({1})".format(stroke.id, len(stroke.events)) ,(255,255,255),font=font)
		else:
			draw.text(textLocation, "Line ID:{0} ({1})".format(stroke.id, len(stroke.events)) ,(255,255,255),font=font)		
	else:
		textLocation = (stroke.centroid[0] + 5, 750-(stroke.centroid[1] - 5))
		draw.text(textLocation, "Point ID:{0} ({1})".format(stroke.id, len(stroke.events)) ,(255,255,255),font=font)
	#Write the file
	image.save(fname)

class GestureStroke():
	def __init__(self, event):
		self.id = event.uid
		self.startTime = event.start
		self.endTime = None
		self.centroid = [event.point.x, event.point.y]
		self.avg_center_dist = None #Don't calculate until needed
		self.isEnded = False
		self.events = [event]
		#Lucky me, Kivy doesn't use negative coordinates or put the origin in the middle of the screen
		self.maxX = self.minX = self.maxY = self.minY = 0


	def addEvent(self, event):
		#Events have ends, and stuff can't occur after the end of the event
		#It does happen, there appear to be some duplicate events, but not often
		if self.isEnded:
			#print "Attempt to add event to ended stroke {0}".format(self.id)
			return None

		self.events.append(event)

		#Update the running centroid total
		#The centroid isn't correct until the event is ended
		self.centroid[0] += event.point.x
		self.centroid[1] += event.point.y

		#Update the bounding box
		if event.point.x > self.maxX:
			self.maxX = event.point.x
		if event.point.y > self.maxY:
			self.maxY = event.point.y
		if event.point.x < self.minX:
			self.minX = event.point.x
		if event.point.y < self.minY:
			self.minY = event.point.y

		#This was the last event of this stroke, so end it and calculate the centroid
		if event.ended:
			self.endTime = event.end
			self.isEnded = True
			self.centroid[0] = self.centroid[0]/len(self.events)
			self.centroid[1] = self.centroid[1]/len(self.events)

	def merge(self,otherEvent):
		#This is multitouch, so instead of deleting multiple overlapping events, 
		#we just slap the other stroke's events into this one and then clean up 
		self.events.extend(otherEvent.events)

		#Sort the events by time, increasing?
		self.events.sort(key=lambda evt: evt.update)

		#Fix the IDs and event end time, recalculate centroid
		centroidX = centroidY = 0
		for index, event in enumerate(self.events):
			
			event['uid'] = self.id #Normalize ID
			
			if index != len(self.events)-1:
				event['end_time'] = -1 #No event except the end knows the end time

			#Acuumulate the centroid position
			centroidX += event.point.x
			centroidY += event.point.y

		#AvgCenterDist and centroid have changed
		self.avg_center_dist = None #Triggers recalculation
		self.centroid[0] = centroidX/len(self.events)
		self.centroid[1] = centroidY/len(self.events)


	def overlaps(self, otherEvent):
		if otherEvent.startTime < self.startTime < otherEvent.endTime:
			#This event started while the other event was going on
			return True
		if self.startTime < otherEvent.startTime < self.endTime:
			#The other event started while this event was going on
			return True
		return False

	def width(self):
		return self.maxX - self.minX

	def height(self):
		return self.maxY - self.minY

	def avgCenterDist(self):
		if self.avg_center_dist == None:
			self.avg_center_dist = 0
			for event in self.events:
				x1 = event.point.x
				y1 = event.point.y
				x2, y2 = self.centroid
				self.avg_center_dist += distance(x1, y1, x2, y2)
			self.avg_center_dist = self.avg_center_dist/len(self.events)
		return self.avg_center_dist

	def isTap(self):
		return (len(self.events) <= 10) and (self.avgCenterDist() < 20)

class GestureCommand():
	def __init__(self):
		self.strokes = {}

	#Expects a dictionary of strokes
	def addStrokes(self, strokes):
		
		self.strokes = strokes
		
		#Clean up the strokes

		#Stutter removal. For each stroke and the stroke after it, if they are seperated by less than 
		#the stutter thresholds in both time and space, they are a product of a poor finger tracking over 
		#the screen, not actually intended to be seperate events. This merges some strokes.
		#TODO may need a spatial as well as chronological component for properly resolving multitouch membership
		ids = self.strokes.keys() #TODO is this ordered? I want them ordered by time...
		for index, item in enumerate(ids):
			if index > 0: 
				current = self.strokes[ids[index]]
				prev = self.strokes[ids[index - 1]]

				if prev.overlaps(current):
					#The events overlap. Either the current one started first or the previous one did
					overlapTime = min(prev.endTime, current.endTime) - max(prev.startTime, current.startTime)

					# if prev.startTime < current.startTime:
					# 	print "{0} overlaps {1} by {2}".format(prev.id, current.id, overlapTime)
					# else:
					# 	print "{0} overlaps {1} by {2}".format(prev.id, current.id, overlapTime)
					if overlapTime < 0.01:
						current.merge(prev)
						#TODO Is this going to give me a bad day? I'm not mutating the thing I'm enumerating...
						del(self.strokes[ids[index-1]])
				else:
					#The events don't overlap
					if prev.endTime < current.startTime:
						#print "{0} ends {1} before {2} begins".format(prev.id, current.startTime - prev.endTime, current.id)
						if current.startTime - prev.endTime < 0.099:
							current.merge(prev)
							del(self.strokes[ids[index-1]])

		#TODO: Any stroke with less than ~10 points is a tap
		#Detect taps, and reduce to one point at the centroid
		#Detect double taps and triple taps 1 or three taps, within a small distance, within a small time
		#TODO: Stutter detection will combine some taps into other strokes if it's not configured right
		
		ids = self.strokes.keys() #TODO is this ordered? I want them ordered by time...
		for index, item in enumerate(ids):
			if index > 0:
				current = self.strokes[ids[index]]
				prev = self.strokes[ids[index - 1]]
				if current.isTap() and prev.isTap():
					#Distance between previous starting and next ending
					timeDistance = current.startTime - prev.endTime
					#Distance between start locations
					spaceDistance = distanceEvents(current.events[0], prev.events[0])
					if timeDistance < 0.5 and spaceDistance < 10:
						#print "{0} -> {1}: {2} {3}".format(prev.id, current.id, timeDistance, spaceDistance)
						print "{0} -> {1} is a double-click".format(prev.id, current.id)


			
		

	def coalesceStrokes():
		#Convert any lines of fewer than 3 points to their centroid, consider them a point
		#Combine any strokes that overlap by a very small amount, or don't start far enough apart into a single stroke
		#But only if they are also long enough that they aren't points themselves?
		#Convert sets of two or three points into double or triple-tap events
		pass




