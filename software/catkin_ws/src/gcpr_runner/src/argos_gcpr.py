#!/usr/bin/python

import random
import json

from time import sleep

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from network_service.msg import *
import math
from tf import transformations as transf
from argos_bridge.msg import Proximity
from argos_bridge.msg import ProximityList


class ProgramLoader(object):
	def __init__(self):
		# Example program, will get replaced
		# Default is to not go anywhere
		self.program = [("True", "self.stop()", 1.0)]

	#This is how programs get deployed to this runner instance
	#Currently just a string containing a set of GCPR tuples
	#This isn't anything like secure, since eval is getting used. 
	#There also may be thread safety concerns
	def replaceProgram(self, msg):
		rospy.logwarn("Got {0}".format(msg.data))
		self.program = json.loads(msg.data)

	def getProgram(self):
		return self.program


class GCPR_driver(object):
	def __init__(self):
		self.programLoader = ProgramLoader()
		self.laser_readings = []
		self.max_range = 0.6 #TODO TOTALLY ARBITARY THRESHOLD FIXME

		self.twistPub = rospy.Publisher('cmd_vel', Twist, queue_size=0)
		self.laser_msg = None

		#For ARGoS sensors
		self.proxReadings = []
		self.heading = 0.0
		self.traveled_x = 0.0
		self.traveled_y = 0.0
		self.lastPosition = None

		#For GCPR program counter, default to 0
		self.prog_ctr = 0


	def update_laser(self, laserMsg):
		self.laser_readings = laserMsg.ranges
		self.laser_msg = laserMsg

	def update_pose(self, poseMsg):
		if self.lastPosition is not None:
			#Update travel distances
			self.traveled_x += self.lastPosition.position.x - poseMsg.position.x
			self.traveled_y += self.lastPosition.position.y - poseMsg.position.y
			w = poseMsg.orientation.w
			x = poseMsg.orientation.x
			y = poseMsg.orientation.y
			z = poseMsg.orientation.z

			#Update the robot's heading.
			#The roll direction is what I'd call yaw.
			#RPY are ambiguious, nothing to be done for it.
			(roll, pitch, yaw) = transf.euler_from_quaternion([w, x, y, z]) 
			self.heading = roll

		#For future distance calculation
		self.lastPosition = poseMsg

	def update_prox(self, proxMsg):
		#Sort prox messages by angle
		self.proxReadings = proxMsg.proximities
		self.proxReadings.sort(key=lambda item:item.angle)
		
	def recv_msg(self, msg):
		rospy.logwarn("Got a message {0}".format(msg.data))
		pass

	def replace_program(self, msg):
		self.programLoader.replaceProgram(msg)

	def run_gcpr(self):
		#Nothing to do yet
		todo_list = []

		#Get all the rules and see if any of them have satisfied guards
		for rule in self.programLoader.getProgram():
			if eval(rule[0]):
				#Check the rate
				if random.random() < rule[2]:
					#Add to the list of things to do
					todo_list.append(rule[1])
		
		random.shuffle(todo_list)

		#This needs some smarter method to unify the drive commands than just FIFO
		#Or does it? FIFO fast enough, plus a short-cycle time would be a sort of
		#stochastic PWM effect
		for item in todo_list:
			#TODO, add "self." to calls in the program...
			#https://stackoverflow.com/questions/1911281/how-do-i-get-list-of-methods-in-a-python-class
			#might be of help for figuring out what substrings are callable
			eval(item)

	#Functions for handling heading
	def set_desired_heading(self, value):
		self.desired_heading = value

	#Within threshold of heading
	def on_heading(self):
		threshold = 1.0
 		if (self.desired_heading < self.heading + threshold)  or (self.heading > self.current_heading - threshold):
 			return True
 		return False

	#Functions for handling a program counter for sequential state changes
	def set_pc(self,value):
		self.prog_ctr = value

	def pc_is(self, value):
		return self.prog_ctr == value

	# Basic action of moving along an arc
	def move_arc(self, rot_speed, trans_speed):
		rTwist = Twist()
		#only two params are used for robots on a table
		rTwist.linear.x = trans_speed
		rTwist.angular.z = rot_speed
		#The rest are not used
		rTwist.linear.y = rTwist.linear.z = 0
		rTwist.angular.x = rTwist.angular.y = 0

		#publish it and wait
		self.twistPub.publish(rTwist)
		
	# Moving straight is moving in an arc with no rotational speed
	def move_fwd(self, speed):
		self.move_arc(0, speed)

	# Turning is moving in an arc with no translational speed
	def move_turn(self, speed):
		self.move_arc(speed, 0)

	# Stopping is moving with no velocity. Have you ever, like, REALLY, looked at your hands, man?
	def stop(self):
		self.move_arc(0,0)

	# This is for doing printouts from inside the GCPR code and having it get out to ROS
	def dbg_print(self, message):
		rospy.logwarn(message)
		
	# Define the guards that can be sensed. Guards are boolean, so they should return either a boolean
	# or something that python is going to treat as a boolean. I've decided to name the guards as 
	# questions so that it reads better

	def is_near_anything(self):
		return self.is_near_left() and self.is_near_right() and self.is_near_center()

	def is_near_right(self):
		start = -3.14
		end = 0.0 #Pi/4
		for reading in self.proxReadings:
			if reading.angle > start and reading.angle < end:
				if reading.value > 0:
					return True
		return False


	def is_near_left(self):
		start = 0.0
		end = 3.14
		for reading in self.proxReadings:
			if reading.angle > start and reading.angle < end:
				if reading.value > 0:
					return True
		return False

	def is_near_center(self):
		start = -0.7854
		end = 0.7854
		for reading in self.proxReadings:
			if reading.angle > start and reading.angle < end:
				if reading.value > 0:
					return True
		return False


rospy.init_node("gcpr_driver", anonymous=True)

#Get the ID of the robot that this instance of the driver is driving
#robot_id = rospy.get_param("/gcpr/robot_id")
#Argos sets these as namespaces, so I may just be able to get the proximity from the 
#namespace, which saves me dealing with the robot ID in the parameters

#Set up an instance of the GCPR driver
gDriver = GCPR_driver()

#Subscribe to laser for this robot
#laser_sub = rospy.Subscriber("/laser_driver_{0}".format(robot_id), LaserScan, gDriver.update_laser)
#Argos proximity sensor
proxSub = rospy.Subscriber("proximity", ProximityList, gDriver.update_prox)

#Subscribe to the position for this robot
#We're not actually using the absolute position, just the distance traveled
#and the heading, as if we had odom and a compass/IMU
poseSub = rospy.Subscriber("position", Pose, gDriver.update_pose)

#Listen for new programs to load
prog_sub = rospy.Subscriber("robot_prog", String, gDriver.replace_program)

#Cleanup code to make sure the robot stops moving when ros is shut down
#Not guaranteed, because the message might not make it out
rospy.on_shutdown(gDriver.stop)	

#TODO figure out what a good rate for the driver to run at is,
# and update at that rate
r = rospy.Rate(10) # 10hz
while not rospy.is_shutdown():
    #Update the GCPR driver
    gDriver.run_gcpr()
    r.sleep()