#!/usr/bin/python

#Connect to all the ARGOS simulated robots, and synthesize the top-down camera image of all of the robots

import rospy
import re
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image as ImageMsg
from PIL import Image, ImageDraw
import yaml
from sensor_msgs.msg import CameraInfo
import rospkg
import math
from tf import transformations as transf

class ImageConverter(object):
    """
    Based on https://github.com/CURG-archive/ros_rsvp/blob/master/image_converter.py
    Convert images/compressedimages to and from ROS
    """

    _ENCODINGMAP_PY_TO_ROS = {'L': 'mono8', 'RGB': 'rgb8',
                              'RGBA': 'rgba8', 'YCbCr': 'yuv422'}
    _ENCODINGMAP_ROS_TO_PY = {'mono8': 'L', 'rgb8': 'RGB',
                              'rgba8': 'RGBA', 'yuv422': 'YCbCr'}
    _PIL_MODE_CHANNELS = {'L': 1, 'RGB': 3, 'RGBA': 4, 'YCbCr': 3}

    @staticmethod
    def to_ros(img):
        """
        Convert a PIL/pygame image to a ROS compatible message (sensor_msgs.Image).
        """

        # Everything ok, convert PIL.Image to ROS and return it
        if img.mode == 'P':
            img = img.convert('RGB')

        rosimage = ImageMsg()
        rosimage.encoding = ImageConverter._ENCODINGMAP_PY_TO_ROS[img.mode]
        (rosimage.width, rosimage.height) = img.size
        rosimage.step = (ImageConverter._PIL_MODE_CHANNELS[img.mode]
                         * rosimage.width)
        rosimage.data = img.tobytes()
        return rosimage

    @staticmethod
    def from_ros(rosMsg):
        """
        Converts a ROS sensor_msgs.Image to a PIL image
        :param rosMsg: The message to convert
        :return: an alpha-converted pygame Surface
        """
        return PILImage.frombytes(ImageConverter._ENCODINGMAP_ROS_TO_PY[rosMsg.encoding], (rosMsg.width, rosMsg.height), rosMsg.data)


class ROSImageSynth(object):
	def __init__(self):
		#Subscribe to all of the robot position messages
		self.subs = {}
		self.topicRE = re.compile("\/bot([0-9]*)\/position")
		self.poses = {}
		self.tags = {}

		#This image will get displayed by Kivy, which will do poorly if it's not the same
		#aspect ratio as the screen 
		self.imWidth = 1680
		self.imHeight = 1050
		self.arenaWidth = 8 #in meters
		self.tagSize = int((self.imWidth/self.arenaWidth) * 0.15625) #px/m times size in m
		self.mToPx = (self.imWidth/self.arenaWidth)

		self.imgPub = rospy.Publisher("/overhead_cam/image", ImageMsg, queue_size=10)
		self.infoPub = rospy.Publisher("/overhead_cam/camera_info", CameraInfo, queue_size=10)

		#Load up our bogus camera information
		self.camera_info = None
		self.loadCamInfo()

		#Sign up for Argos position updates
		self.checkSubs()

		# Find the path to myself, for loading png files
		rospack = rospkg.RosPack()
		self.pkg_path = rospack.get_path('user_interface')


	def loadCamInfo(self):
		cal_file = rospy.get_param("/sim_cam/camera_info_url")
		with open(cal_file, 'r') as cal_in:
			cal_data = yaml.load(cal_in)

		camera_info_msg = CameraInfo()
		camera_info_msg.width = cal_data["image_width"]
		camera_info_msg.height = cal_data["image_height"]
		camera_info_msg.K = cal_data["camera_matrix"]["data"]
		camera_info_msg.D = cal_data["distortion_coefficients"]["data"]
		camera_info_msg.R = cal_data["rectification_matrix"]["data"]
		camera_info_msg.P = cal_data["projection_matrix"]["data"]
		camera_info_msg.distortion_model = cal_data["distortion_model"]
		self.camera_info = camera_info_msg


	def checkSubs(self):
		#Check for new position topics and subscribe to them
		for topic in rospy.get_published_topics():
			#If we're not already subscribed to it and it is a bot position
			if topic[0] not in self.subs.keys() and self.topicRE.match(topic[0]):
				self.subs[topic[0]] = rospy.Subscriber(topic[0], Pose, callback = self.updateRobot, callback_args=topic[0])

	def updateRobot(self, poseMsg, topic):
		#Get the robot number out of the topic
		robot_id = self.topicRE.match(topic).group(1)
		#Store the new pose
		self.poses[robot_id] = poseMsg
		#If we haven't loaded the tag for this ID before, do it now
		if robot_id not in self.tags.keys():
			tag_path="{}/src/tag36h11/tag36_11_{:05d}.png".format(self.pkg_path, int(robot_id))
			tag_image = Image.open(tag_path).convert('RGBA')
			#Resize to the size specified
			tag_image = tag_image.resize((self.tagSize, self.tagSize), Image.BICUBIC)
			self.tags[robot_id] = tag_image

	def publishNewFrame(self):
		#Called by a timer, updates the image and publishes it
		#Create a new white image
		image = Image.new('RGBA', (self.imWidth, self.imHeight), color='white')

		#robotDraw = ImageDraw.Draw(image)
		#Draw a dot for each robot 
		for robot_id in self.poses.keys():
			#Robot positions are in meters, convert to px to draw
			#Argos has origin in center of arena
			#Origin of PIL is in top left
			pX = self.imWidth/2 + (self.poses[robot_id].position.x * self.mToPx)
			pY = self.imHeight/2 - (self.poses[robot_id].position.y * self.mToPx)

			#PIL is not having any of these fractional pixels
			pX = int(pX)
			pY = int(pY)

			#robotDraw.ellipse([pX-10, pY-10, pX+10, pY+10], fill='red')
			#robotDraw.text((pX-2, pY-5), robot_id, fill='black')
			if robot_id not in self.tags.keys():
				rospy.logwarn("Have pose but not tag for {}".format(robot_id))
			else:
				#Rotate the tag image to the appropriate angle
				tag_img = self.tags[robot_id]

				w = self.poses[robot_id].orientation.w
				x = self.poses[robot_id].orientation.x
				y = self.poses[robot_id].orientation.y
				z = self.poses[robot_id].orientation.z

				#Update the robot's heading.
				(roll, pitch, yaw) = transf.euler_from_quaternion([w, x, y, z]) 
				#Roll is the robot's heading, but I want to change it 
				#to degrees for PIL to rotate the image
				roll = roll * (180/math.pi)
				tag_img = tag_img.rotate(roll, expand=True, resample=Image.BICUBIC)
				image.paste(tag_img, box=(pX-self.tagSize/2, pY-self.tagSize/2), mask=tag_img)

		#del robotDraw

		#Publish the image on an image topic
		rosImg = ImageConverter.to_ros(image)
		self.imgPub.publish(rosImg)

		#Publish some camera info (it's all boooooggguussss!)
		if self.camera_info is not None:
			self.infoPub.publish(self.camera_info)

		#See if we need to update
		self.checkSubs()

rospy.init_node("sky_eye", anonymous=True)
image_synth = ROSImageSynth()

r = rospy.Rate(20) #Units are Hz
while not rospy.is_shutdown():
	image_synth.publishNewFrame()
	r.sleep()
