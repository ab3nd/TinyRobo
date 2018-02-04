#!/usr/bin/python

import random
import json

from time import sleep

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from network_service.msg import *

# Ties together "laser" scanner, gcpr programs, and a physical robot
# There should be an instance of this for each robot

class ProgramLoader(object):
	def __init__(self):
		# Example program, will get replaced
		# What this should do is make the robot avoid obstacles and stop when
		# it isn't near anything.
		self.program = [("not(self.is_near_anything())", 1.0, "self.move_fwd(0.1)"),
				   ("self.is_near_left() and not self.is_near_center()", 1.0, "self.move_arc(-0.5, 0.2)"),
				   ("self.is_near_right() and not self.is_near_center()", 1.0, "self.move_arc(-0.5, 0.2)"),
				   ("self.is_near_center()", 1.0, "self.move_turn(0.3)")]

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
	def __init__(self, robot_id):
		self.programLoader = ProgramLoader()
		self.laser_readings = []
		self.max_range = 0.2 #TODO TOTALLY ARBITARY THRESHOLD FIXME
		#TODO probably should name this topic better
		self.twistPub = rospy.Publisher('/gcpr_drive_{}'.format(robot_id), Twist, queue_size=0)

	def update_laser(self, laserMsg):
		self.laser_readings = laserMsg.ranges

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
				if random.random() < rule[1]:
					#Add to the list of things to do
					todo_list.append(rule[2])
		
		random.shuffle(todo_list)

		#This needs some smarter method to unify the drive commands than just FIFO
		#Or does it? FIFO fast enough, plus a short-cycle time would be a sort of
		#stochastic PWM effect
		for item in todo_list:
			#TODO, add "self." to calls in the program...
			#https://stackoverflow.com/questions/1911281/how-do-i-get-list-of-methods-in-a-python-class
			#might be of help for figuring out what substrings are callable
			eval(item)

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
		span = len(self.laser_readings)/3
		#Right side of readings
		for reading in self.laser_readings[span+span:]:
			#If there's a close object, return true
			if reading < self.max_range / 2:
				return True
		return False

	def is_near_left(self):
		span = len(self.laser_readings)/3
		#Left side of readings
		for reading in self.laser_readings[:span]:
			#If there's a close object, return true
			if reading < self.max_range / 2:
				return True
		return False

	def is_near_center(self):
		span = len(self.laser_readings)/3
		#Center of readings
		for reading in self.laser_readings[span:span+span]:
			#If there's a close object, return true
			if reading < self.max_range / 2:
				return True
		return False


rospy.init_node("gcpr_driver", anonymous=True)



#Get the ID of the robot that this instance of the driver is driving
robot_id = rospy.get_param("/gcpr/robot_id")

#Set up an instance of the GCPR driver
gDriver = GCPR_driver(robot_id)

#Subscribe to laser for this robot
laser_sub = rospy.Subscriber("/laser_driver_{0}".format(robot_id), LaserScan, gDriver.update_laser)

#Recieve messages over the simulated network
net_sub = rospy.Subscriber("/messages_to_{0}".format(robot_id), NetMsgToRobot, gDriver.recv_msg)

#Listen for new programs to load
prog_sub = rospy.Subscriber("/robot_prog/{0}".format(robot_id), String, gDriver.replace_program)

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