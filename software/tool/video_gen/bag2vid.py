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

bag = rosbag.Bag('./id_2_cond_1_2017-10-16-15-00-48.bag')


class VideoGenerator(object):

	def __init__(self, frameDelay):
		self.frameDelay = rospy.Duration(frameDelay)
		self.lastMsgTime = None
		#This is the sum of the video images I'm working with
		self.frameSize = (1800,1246)
		self.frameImage = PILImage.new("RGB", self.frameSize)
		
		#Set up a video writer instance
		self.fourcc = cv2.cv.FOURCC('m','j','p','g')
		#TODO set the fps correctly
		self.vidWriter = cv2.VideoWriter("./test.avi", self.fourcc, 30, self.frameSize, True)

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
			#print imgMsg.width, imgMsg.height
			#Convert to a PIL image, mapping is from https://github.com/CURG-archive/ros_rsvp/blob/master/image_converter.py
			_ENCODINGMAP_ROS_TO_PY = {'mono8': 'L', 'rgb8': 'RGB','rgba8': 'RGBA', 'yuv422': 'YCbCr'}
			encoding = _ENCODINGMAP_ROS_TO_PY[imgMsg.encoding]
			img = PILImage.fromstring(encoding, (imgMsg.width, imgMsg.height), imgMsg.data)
			#Insert into the frame image
			offset=(800,0)
			self.frameImage.paste(img, offset)

		self.drawFrame(msgTime)

	def updateAudio(self, audMsg, msgTime):
		# Extend the audio track
		pass

	def addPoint(self, pointMsg, msgTime):
		# Draw a new point on the UI Image
		#Scale the points from the full screen to the active area
		#xConv = 1000.0/1680.0
		#yConv = 750.0/1050.0
		#Magic numbers are half of the difference in screen sizes...
		x = pointMsg.point.x - 340#* xConv
		y = pointMsg.point.y - 150#* yConv
		# the UI image is at 800 px off from the corner of the frame
		x += 800
		# Kivy points are upside down from PIL points
		y = max(750 - y, 0)

		#Draw a dot on the screen at the touch point
		draw = ImageDraw.Draw(self.frameImage)
		dotSize = 2
		draw.ellipse([(x-dotSize, y-dotSize), (x+dotSize, y+dotSize)], fill='red')
		del draw
		self.drawFrame(msgTime)

	def drawFrame(self, msgTime):
		# If a new frame is generated for each image, the framerate of the video will be absurd. At worst,
		# it could be the sum of the framerate of all the video streams in the bag. To prevent this, the
		# framerate is precalculated, and frames are generated when enough time in the bag has passed to 
		# keep the frame rate under the precalculated rate. 
		#Check if this message is beyond self.frameDelay from self.lastMsgTime,
		#If it is, draw another frame and update the last message time
		if self.lastMsgTime is None:
			self.lastMsgTime = msgTime
		else:
			if msgTime - self.lastMsgTime > self.frameDelay:
				#Convert the whole frame to an OpenCV image in the right color space
				data = self.frameImage.tostring()
				cols, rows = self.frameImage.size
				#We can get away with the 3 here because we know it's RGB (or BGR, apparently)
				img = np.fromstring(data, dtype=np.uint8).reshape(rows,cols,3)
				#Make Robots Red Again!
				img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
				self.vidWriter.write(img)

	def finalizeVideo(self):
		#Clean up, possibly involving combining audio with video
		self.vidWriter.release()
		pass

#Get info about the bag
bagInfo = yaml.load(bag._get_yaml_info())

for topic in bagInfo['topics']:
	print "{0} \t{1} \tfrequency:{2} \tmsg count:{3}".format(topic['topic'], topic['type'], topic['frequency'], topic['messages'])

#Precalculate the video framerate
#For now, it's just the same as the greater of the two compressed streams
rate1 = bag.get_type_and_topic_info()[1]['/experiment/c09/camera/image/compressed'][3]
rate2 = bag.get_type_and_topic_info()[1]['/experiment/c35/camera/image/compressed'][3]
framerate = max(rate1, rate2)
secPerFrame = framerate/bagInfo['duration']

vg = VideoGenerator(secPerFrame)

for topic, msg, t in bag.read_messages():
	if topic.startswith("/experiment/c"):
		#Output from one of the cameras
		vg.updateImage(msg, t, isCompressed=True)
	if topic == "/ui_image":
		#New UI screen
		vg.updateImage(msg, t)
	if topic == "/audio":
		#New sound
		pass #vg.updateAudio(msg, t)
	if topic == "/touches":
		vg.addPoint(msg, t)

vg.finalizeVideo()

#print "Topics: {0}".format(topics)
#print "Types:{0}".format(types)

#print bagInfo