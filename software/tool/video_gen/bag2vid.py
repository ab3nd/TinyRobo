#!/usr/bin/python

# Given a bag file, write a video file using the images and points, with accompanying audio 

import rosbag
import rospy
import yaml

from sensor_msgs.msg import Image as ImageMsg
from sensor_msgs.msg import CompressedImage as CompImageMsg

#for building the frame
from PIL import Image as PILImage
from PIL import ImageDraw
import StringIO

#For writing video
import cv2
import numpy as np

#For working with command line and ffmpeg
import subprocess
import argparse
import os.path

class VideoGenerator(object):

	def __init__(self, frameRate):
		#self.frameDelay = rospy.Duration(frameRate)
		self.lastMsgTime = None
		#This is the sum of the video images I'm working with
		self.frameSize = (1800,1246)
		self.frameImage = PILImage.new("RGB", self.frameSize)
		
		#Set up a video writer instance
		self.fourcc = cv2.VideoWriter_fourcc(*'FMP4')
		#TODO set the fps correctly
		self.vidWriter = cv2.VideoWriter("./temp.avi", self.fourcc, frameRate * 2, self.frameSize, True)
		
		#Write audio to a file
		self.audioFile = open('temp.mp3', 'w')

	def updateImage(self, imgMsg, msgTime, isCompressed = False):
		if isCompressed:
			stio = StringIO.StringIO(imgMsg.data)
			img = PILImage.open(stio)
			#TODO this is specific to the image sizes I'm working with because I'm not writing
			#a custom compositor just to put these in place
			offset = (0,0)
			if img.size[0] == 800:
				#The 800x600 image, do nothing
				pass
			else:
				#All the way over, under the UI view
				offset = (554, 750)
			#Stick it in the frame image at the right spot
			self.frameImage.paste(img, offset)	
		else:
			#These are 1000x750, like the display images
			#Convert to a PIL image, mapping is from https://github.com/CURG-archive/ros_rsvp/blob/master/image_converter.py
			_ENCODINGMAP_ROS_TO_PY = {'mono8': 'L', 'rgb8': 'RGB','rgba8': 'RGBA', 'yuv422': 'YCbCr'}
			encoding = _ENCODINGMAP_ROS_TO_PY[imgMsg.encoding]
			img = PILImage.frombytes(encoding, (imgMsg.width, imgMsg.height), imgMsg.data)
			#Insert into the frame image
			offset=(800,0)
			self.frameImage.paste(img, offset)

		self.drawFrame(msgTime)

	def updateAudio(self, audMsg, msgTime):
		self.audioFile.write(audMsg.data)

	def addPoint(self, pointMsg, msgTime):
		# Draw a new point on the UI Image
		#Scale the points from the full screen to the active area
		#Magic numbers are half of the difference in screen sizes...
		x = pointMsg.point.x - 340
		y = pointMsg.point.y - 150
		# the UI image is at 800 px off from the corner of the frame
		x += 800
		# Kivy points are upside down from PIL points
		y = max(750 - y, 0)

		#Draw a dot on the screen at the touch point
		draw = ImageDraw.Draw(self.frameImage)
		dotSize = 2
		draw.ellipse([(x-dotSize, y-dotSize), (x+dotSize, y+dotSize)], fill='blue')
		del draw

	def drawFrame(self, msgTime):
		#Convert the whole frame to an OpenCV image in the right color space
		data = self.frameImage.tobytes()
		cols, rows = self.frameImage.size
		#We can get away with the 3 here because we know it's RGB (or BGR, apparently)
		img = np.fromstring(data, dtype=np.uint8).reshape(rows,cols,3)
		#Make Robots Red Again!
		img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
		self.vidWriter.write(img)

	def finalizeVideo(self, outfileName):
		#Release the video writer
		self.vidWriter.release()
		#Close the audio file
		self.audioFile.close()

		#Add the audio file to the video file
		subprocess.call(['ffmpeg', '-y', '-i', 'temp.mp3', '-i', 'temp.avi', '-acodec', 'copy', '-vcodec', 'copy', '-shortest', outfileName])
		print "wrote {}".format(outfileName)


parser = argparse.ArgumentParser(description="Convert a bagfile from Abe's experiment into a video with audio")
parser.add_argument('bagfileName', type=str, nargs=1, help='path to the bagfile')
args = parser.parse_args()

#Get the file name and load the bagfile
name = args.bagfileName[0]
bag = rosbag.Bag(name)

#Create a name for the output video
#Get the name of the directory it was in
participant = name.split('/')[-2]
path = os.path.split(name)[0]
#outputFile = path + "/" + participant + ".avi"
outputFile = "./" + participant + ".avi"

#Get info about the bag
bagInfo = yaml.load(bag._get_yaml_info())

for topic in bagInfo['topics']:
	print "{0} \t{1} \tfrequency:{2} \tmsg count:{3}".format(topic['topic'], topic['type'], topic['frequency'], topic['messages'])

#Get the number of video frames that arrived, divided by the duration of the bagfile
#The frequency from the bagfile is something like 5fps higher, for some reason
msg_count_1 = bag.get_type_and_topic_info()[1]['/experiment/c09/camera/image/compressed'][1]
msg_count_2 = bag.get_type_and_topic_info()[1]['/experiment/c35/camera/image/compressed'][1]
avg_msgs = float(msg_count_2 + msg_count_1)/2.0
framerate = avg_msgs/bagInfo['duration']

print "Calculated framerate: {}".format(framerate)  

vg = VideoGenerator(framerate)

for topic, msg, t in bag.read_messages():
	if topic.startswith("/experiment/c"):
		#Output from one of the cameras
		vg.updateImage(msg, t, isCompressed=True)
	if topic == "/ui_image":
		#New UI screen
		vg.updateImage(msg, t)
	if topic == "/audio":
		#New sound
		vg.updateAudio(msg, t)
	if topic == "/touches":
		vg.addPoint(msg, t)

vg.finalizeVideo(outputFile)
