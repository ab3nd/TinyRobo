#!/usr/bin/python

#Given a list of bagfiles, render the path of each tag detected in each bagfile on a picture of the arena

from PIL import Image
import sys

#Load the picture
bg = Image.open("arena.png")

#Some very stupid command line processing here
files = sys.argv[1:]
for file in files:
	print file
	
#For each bagfile
	#Load the bagfile
	#Get a list of the tag center points
	#Pick a color
	#Draw a line in that color
	#Line end finials?