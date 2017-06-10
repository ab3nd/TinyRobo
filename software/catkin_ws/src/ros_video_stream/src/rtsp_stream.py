#!/usr/bin/python

# Use OpenCV and python rather than C++, because as of this writing, the codecs available
# on Ubuntu can't deal with H264. 

import rospy
import sys
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class imgMsgConverter:
	#Set up a publisher and a bridge for conversion
	def __init__(self):
		self.bridge = CvBridge()
		self.imagePub = rospy.Publisher("rtsp_stream", Image, queue_size=2)
	
	#Attempt to convert the image to a ros message and publish it
	def convertImage(self, cvImage):
		try:
			rosImg = self.bridge.cv2_to_imgmsg(cvImage, "bgr8")
		except CvBridgeError as e:
			rospy.logerr(e)
			pass
		self.imagePub.publish(rosImg)


def main(args):
	rospy.init_node('video_streamer', anonymous = True)
	converter = imgMsgConverter()
	resource = "http://localhost:8888/videostream.cgi"
	#Get an openCV image from the camera
	cap = cv2.VideoCapture(resource)
	if not cap.isOpened():
		rospy.logerr("Error opening resource: {0}".format(resource))
		sys.exit(1)
	rospy.loginfo("Opened {0}, streaming now...".format(resource))

    #Read an initial frame
	rval, frame = cap.read()
	while not rospy.is_shutdown() and rval:
		#Convert to a ros image and publish it
		converter.convertImage(frame)
		#cv2.imshow("Stream: " + resource_name, frame)
		#get the next one
		rval, frame = cap.read()
		#TODO how would we rectify this? Can ROS do it?
		

if __name__ == '__main__':
	main(sys.argv)