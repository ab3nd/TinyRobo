#!/usr/bin/python

# Given a bag file, write a video file using the images and points, with accompanying audio 

import rosbag
import yaml

from sensor_msgs.msg import Image as ImageMsg
from sensor_msgs.msg import CompressedImage as CompImageMsg

from PIL import Image as PILImage
import StringIO

bag = rosbag.Bag('./id_2_cond_1_2017-10-16-15-00-48.bag')


class VideoGenerator(object):

	def __init__(self, frameDelay):
		self.frameDelay = frameDelay
		self.lastMsgTime = None
		#This is the sum of the video images I'm working with
		self.frameImage = PILImage.new("RGB", (1800,1246))

	def updateImage(self, imgMsg, msgTime, isCompressed = False):
		if isCompressed:
			pass
			stio = StringIO.StringIO(imgMsg.data)
			img = PILImage.open(stio)
			#TODO this is specific to the image sizes I'm working with because I'm not writing
			#a custom compositor just to put these in place
			offset = (0,0)
			if img.size[0] == 800:
				#The 800x600 image, do nothing
				pass
			else:
				offset = (30, 750)
			#Stick it in the frame image at the right spot
			self.frameImage.paste(img, offset)	
		else:
			#These are 1000x750, like the display images
			print imgMsg.width, imgMsg.height
			#Convert to a PIL image
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
		pass

	def drawFrame(self, msgTime):
		#self.frameImage.save("test{0}.png".format(msgTime))
		self.frameImage.save("test.png".format(msgTime))
		# If a new frame is generated for each image, the framerate of the video will be absurd. At worst,
		# it could be the sum of the framerate of all the video streams in the bag. To prevent this, the
		# framerate is precalculated, and frames are generated when enough time in the bag has passed to 
		# keep the frame rate under the precalculated rate. 
		#Check if this message is beyond self.frameDelay from self.lastMsgTime,
		#If it is, draw another frame and update the last message time
		#if self.lastMsgTime is None:
		#	self.lastMsgTime = msgTime
		#	return

	def finalizeVideo(self):
		#Clean up, possibly involving combining audio with video
		pass


#Get the topics and types from the bag
#topics = bag.get_type_and_topic_info()[1].keys()
#types = []
#for i in range(0,len(bag.get_type_and_topic_info()[1].values())):
#    types.append(bag.get_type_and_topic_info()[1].values()[i][0])

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
		vg.updateAudio

vg.finalizeVideo()

#print "Topics: {0}".format(topics)
#print "Types:{0}".format(types)

#print bagInfo