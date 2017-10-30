#!/usr/bin/python

import os
from PIL import Image, ImageDraw
import math
import re 

#From https://stackoverflow.com/questions/2669059/how-to-sort-alpha-numeric-set-in-python
def sorted_nicely( l ): 
    """ Sort the given iterable in the way that humans expect.""" 
    convert = lambda text: int(text) if text.isdigit() else text 
    alphanum_key = lambda key: [ convert(c) for c in re.split('([0-9]+)', key) ] 
    return sorted(l, key = alphanum_key)


#Generate pages of aruco tags from images of them, for printing on sticker paper

#Some assumptions
pxPerMM = 2.8346 #This is ~72px/inch

#We don't resize the images, just put spacing around them for printing
imgPadding = 7 #Px per side of the image, so this gets doubled

#Load the image list from this directory
imgDir = "./aruco_tags"

#gets calculated in composition pass, used in page setup pass
sizeX = sizeY = 0

for dirName, dirNames, fileNames in os.walk(imgDir):
	#There shouldn't be any subdirs
	for file in fileNames:
		fullName = os.path.join(dirName, file)

		#Load the file with PIL, get its size
		tagImg = Image.open(fullName)
		sizeX, sizeY = tagImg.size

		#Calculate the new size
		sizeX += 2*imgPadding
		sizeY += 2*imgPadding

		#Create a new image
		padImg = Image.new("RGB", (sizeX, sizeY), 'white')

		#Outline it in black (cut line guide)
		padDraw = ImageDraw.Draw(padImg)
		padDraw.rectangle([(0,0),(sizeX-1,sizeY-1)], fill='white', outline='black')

		#Put the tag image in the middle of it
		padImg.paste(tagImg, (imgPadding, imgPadding))

		#Get rid of the draw, save the file
		del padDraw
		newName = "./padded_{0}".format(file)
		padImg.save(newName)

#Now that we have all the images padded, lets compose them into images that 
#are less than 8x10.5, and so will fit on an 8.5x11 sheet of sticker paper
inchInMM = 25.4 #Do not change unless you work for NIST or CGPM!

#get the size of a padded image in inches
inSize = (sizeX/pxPerMM)/inchInMM

#Figure out how many of them can fit across and down a page
across = int(math.floor(8/inSize))
down = int(math.floor(10.5/inSize))

#Look at the current directory
for dirName, dirNames, fileNames in os.walk("./"):
	#only look at the file names that are for the padded images
	fileNames = sorted_nicely(filter(lambda x: x.startswith("padded_"), fileNames))
	#Calculate the number of pages needed
	pageCount = math.ceil(float(len(fileNames))/float(across*down))

	#define a new image that has enough pixels for the padded images
	pageImg = Image.new("RGB", (across*sizeX, down*sizeY), 'white')

	placed = 0
	for index, fName in enumerate(fileNames):

		#Get the position on the page
		xPos = placed % across
		yPos = placed / across #Finally a use for integer division

		#Convert from position on page to pixels
		xPos *= sizeX
		yPos *= sizeY

		#Paste from the file into the new image
		fullName = os.path.join(dirName, fName)
		print "placed {0} at ({1},{2})".format(fullName, xPos, yPos)
		pageImg.paste(Image.open(fullName), (xPos, yPos))
		placed += 1

		#See if we're done with this page
		if placed == (across * down):
			placed = 0
			#Write the old pageImg
			outFile = "{0}_page.png".format(index)
			print "---Wrote {0}".format(outFile)
			pageImg.save(outFile)
			#Create a new pageImg
			pageImg = Image.new("RGB", (across*sizeX, down*sizeY), 'white')

	if placed > 0:
		#There were leftover images
		outFile = "{0}_page.png".format(len(fileNames))
		print "---Wrote {0}".format(outFile)
		pageImg.save(outFile)




