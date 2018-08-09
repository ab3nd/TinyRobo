#!/usr/bin/python

import rospy
from user_interface.msg import Kivy_Event, Stroke
from geometry_msgs.msg import Point
import math
from apriltags_ros.msg import *

#For debugging
import Image, ImageDraw, ImageFont
import numpy as np

# Receives the locations of the robots and the strokes made by the user, and determines if the 
# resulting configuration could be a box selection. Generally, this means that the user drew a line
# and that the bounding box of the line has robots inside of it. 
# This is actually too permissive of a criterion for box selection, but higher level nodes will
# accept or reject the box selection canidates based on state and previous actions

def distanceEvents(eventA, eventB):
	x1 = eventA.point.x
	y1 = eventA.point.y
	x2 = eventB.point.x
	y2 = eventB.point.y

	return distance(x1, y1, x2, y2)

def distance(x1, y1, x2, y2):
	return math.sqrt(pow(x1 - x2, 2) + pow(y1 -y2, 2))

def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

#Debug function, dumps strokes to image files for viewing
def dumpStroke(stroke, fname=None, image=None):
	pxlPad = 16
	#Set up widths and heights for image
	width = 1680 
	height = 1050

	#Load parameters for the image, pad the width and height
	if fname == None:
		fname = "stroke_{0}.png".format(stroke.uid)

	# If image is None, we're not dumping to an image that got created already
	if image is None:
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

	for event in stroke.events:
		#Convert to image coordinates
		x = int(event.point.x)
		y = int(event.point.y)
		
		#Draw the event
		draw.ellipse([(x-3,y-3),(x+3,y+3)], fill=colors.pop(0))

	#Draw the stroke centroid
	#Convert to image coordinates
	x = int(stroke.centroid.x)
	y = int(stroke.centroid.y)
	
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
		x2 = stroke.centroid.x
		y2 = stroke.centroid.y
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
		textLocation = (stroke.centroid.x + 5, stroke.centroid.y - 5)
		if (d1 * d2) == 0:
			draw.text(textLocation,"d1 or d2 was zero",(255,255,255),font=font)
			return
		capped = clamp((pow(d1, 2) + pow(d2, 2) - pow(d3, 2))/(2 * d1 * d2), -1, 1)
		angle = math.acos(capped)
		if angle < 1:
			draw.text(textLocation, "Circle ID:{0} ({1})".format(stroke.uid, len(stroke.events)) ,(255,255,255),font=font)
		elif angle < 2.5:
			draw.text(textLocation, "Arc ID:{0} ({1})".format(stroke.uid, len(stroke.events)) ,(255,255,255),font=font)
		else:
			draw.text(textLocation, "Line ID:{0} ({1})".format(stroke.uid, len(stroke.events)) ,(255,255,255),font=font)		
	else:
		textLocation = (stroke.centroid.x + 5,  stroke.centroid.y - 5)
		draw.text(textLocation, "Point ID:{0} ({1})".format(stroke.uid, len(stroke.events)) ,(255,255,255),font=font)
	#Write the file
	image.save(fname)

class BoxSelectDetector(object):
	def __init__(self):
		self.currentTags = {}

	def update_robot_points(self, msg):
		#Just saves the detections
		self.currentTags = {}
		for ii in range(len(msg.detections)):
			self.currentTags[int(msg.detections[ii].id)] = msg.detections[ii]

	def check_stroke(self, msg):
		#For debugging
		dumpStroke(msg)
		
		selected_tags = []
		if len(self.currentTags) > 0:
			#Get the bounding box of the stroke
			xs = [event.point.x for event in msg.events]
			minX = min(xs)
			maxX = max(xs)
			ys = [event.point.y for event in msg.events]
			minY = min(ys)
			maxY = max(ys)
			#Get all the tags that have at least one corner in the box
			for tag in self.currentTags.values():
				print "minX: {0} tagCenterX: {1} maxX: {2}".format(minX, tag.tagCenterPx.x, maxX)
				print "minY: {0} tagCenterY: {1} maxY: {2}".format(minY, tag.tagCenterPx.y, maxY)
				print "---"
				if (minX < tag.tagCenterPx.x < maxX) and (minY < tag.tagCenterPx.y < maxY):
					selected_tags.append(tag.id)

			if len(selected_tags) > 0:
				#This is possibly a box select, pack it up and publish it
				rospy.loginfo("{0} selects {1}".format(msg.uid, selected_tags))
		
rospy.init_node('box_select_detect')
bsd = BoxSelectDetector()
strokeSub = rospy.Subscriber("/strokes", Stroke, bsd.check_stroke)
tagSub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, bsd.update_robot_points)

rospy.spin()
