#!/usr/bin/python

#Given a list of bagfiles, render the path of each tag detected in each bagfile on a picture of the arena

from PIL import Image, ImageDraw
import sys
import rosbag
import os.path
import randomcolor #sudo -H pip install randomcolor, because I'm done doing it with numpy rotating matrix hax


#For random path colors
rand_color = randomcolor.RandomColor()

def getPoints(bagfile):
	#Ends up being lists of sorted timestamps and points, indexed by tag id
	tag_points = {}

	for topic, msg, t in bagfile.read_messages():
		if topic == "/tag_detections":
			if len(msg.detections) > 0:
				for tag in msg.detections:
					#Get the center point, we already have the timestamp
					#Round is because PIL doesn't like subpixels
					tag_x = round(tag.tagCenterPx.x)
					tag_y = round(tag.tagCenterPx.y)
					
					#Put it in the hash by ID
					if tag.id in tag_points.keys():
						tag_points[tag.id].append([t,tag_x, tag_y])
					else:
						tag_points[tag.id] = []
						tag_points[tag.id].append([t, tag_x, tag_y])

	#Sort each of the lists of points by time (the first element)
	for tag in tag_points.keys():
		tag_points[tag].sort(key= lambda x: x[0])
		
	return tag_points
			

#Log the robots seen for file name
robots = []

#Lists of points, indexed by robot id
all_tracks = {}

#Collect all tracks, indexed per robot
#Some very stupid command line processing here
files = sys.argv[1:]
for file in files:
	#If it's a file, load it
	if os.path.isfile(file):
		bag = rosbag.Bag(file)
		
		points = getPoints(bag)
		for robot_id in points.keys():
			if robot_id in all_tracks.keys():
				all_tracks[robot_id].append(points[robot_id])
			else:
				all_tracks[robot_id] = []
				all_tracks[robot_id].append(points[robot_id])
	else:
		print "*** Warning: {0} is not a file".format(file)

#Render each robot's tracks to a different file
for robot in all_tracks.keys():
	#Load the picture background
	bgnd = Image.open("arena.png").convert('RGB')
	bg_draw = ImageDraw.Draw(bgnd)

	for path in all_tracks[robot]:
		#Pick a color
		color = rand_color.generate()[0] #It returns an array

		#Draw the lines
		for pointIdx in range(1,len(path)):
			x1, y1 = path[pointIdx-1][1:]
			x2, y2 = path[pointIdx][1:]

			#Draw a line in that color between the points
			bg_draw.line([(x1+43, y1+34), (x2+43, y2+34)], fill=color, width=5)
		
	#Create file name
	fname = "robot_{}.png".format(robot)
	bgnd.save(fname)