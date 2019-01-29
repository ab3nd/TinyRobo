#!/usr/bin/python

#Annotate the debug graphics with the detected images to get a better idea of what I'm looking at

import fnmatch
import rosbag
import rospy
import yaml
import json
import sys
import os

from PIL import Image
from PIL import ImageDraw, ImageFont

#From https://stackoverflow.com/questions/1724693/find-a-file-in-python
def find(pattern, path):
    result = []
    for root, dirs, files in os.walk(path):
        for name in files:
            if fnmatch.fnmatch(name, pattern):
                result.append(os.path.join(root, name))
    return result

#Given a directory, get the image path and the gesture bag path
imagefile = find("*.png", sys.argv[1])[0]
bagfile = find("*.bag", sys.argv[1])[0]


#Name of this event is in msg.eventName as a string
#Stamp of this event is in msg.stamp as a rospy timestamp (secs and nsecs)
#msg.strokes is a list of lists of events, which have points in fractional pixels
#msg.centroid is point in fractional pixels
#msg strokes have a uid, message itself doesn't, but can safely use msg.strokes[0].uid
#because messages don't overlap
def draw_gesture(imgDraw, gesture, color):
	for stroke in gesture.strokes:
		for event in stroke.events:
			x = event.point.x
			y = event.point.y
			imgDraw.ellipse((x-4, y-4, x+4, y+4), outline=color)


def draw_gestures(imgPath, gestures):

	if len(gestures.keys()) == 0:
		#Nothing to do here, we didn't actually get any gestures
		return
	#Load the image and get an imagedraw object on it
	img = Image.open(imgPath)
	draw = ImageDraw.Draw(img)

	#Load up a font to use
	fnt = ImageFont.truetype('/usr/lib/python2.7/dist-packages/kivy/data/fonts/Roboto-Regular.ttf', 20)

	#Calculate spacing for text and colors for drawing for all gestures
	height = img.size[1]
	
	placementStep = height/len(gestures.keys())
	colorStep = 255/len(gestures.keys())
	colorR = 0
	colorG = 0
	colorB = 255

	#Correction because the images were smaller than the screen, and 
	#were displayed centered in the screen, so the logged location of the user
	#touches are in screen coordinates, and the detected robot location is in 
	#image coordinates. 
	fudge_x = 340 # (width of screen (1680) - width of image (1000))/2
	fudge_y = 150 # (height of screen (1050) - height of image (750))/2

	#Index for placement calculations
	idx = 0

	for g in gestures.keys():
		gesture = gestures[g]
		#Walk from blue to red without dropping below 0
		#Green is ignored
		colorR = max(0, colorR + colorStep)
		colorB = max(0, colorB - colorStep)

		#Draw the gesture event type near the l side of the image
		draw.text((10, idx*placementStep), gesture.eventName, font=fnt, fill=(colorR, colorG, colorB))
		txt_width, txt_height = draw.textsize(gesture.eventName, font=fnt)
		#Draw a thin line from the text to the starting point of the stroke
		#Height inversion compensates for kivy fail
		draw.line((12 + txt_width, (idx*placementStep)+txt_height/2, gesture.strokes[0].events[0].point.x, gesture.strokes[0].events[0].point.y), fill=(colorR, colorG, colorB))

		#Draw all the gesture points
		draw_gesture(draw, gesture, (colorR, colorG, colorB))

		#Update index
		idx += 1
	del draw
	
	img.save(imgPath)

#All gestures go in the dict, drawing gets called on each one later
all_gestures = {}

#Open the bagfile and read all the gesture points and IDs into a dictionary by gesture
bag = rosbag.Bag(bagfile)
for topic, msg, t in bag.read_messages():
	if topic == "/gestures":
		all_gestures[msg.strokes[0].uid] = msg

draw_gestures(imagefile, all_gestures)