#!/usr/bin/python

import random
from time import sleep

import rospy
from std_msgs.msg import String
import json

# Load and run GCPR files

# Define the actions that can be run with the GCPR actions and the conditionals that can be detected

# Basic action of moving along an arc
def move_arc(rot_speed, trans_speed):
	#TODO this is where we'd actually have ROS send the message to the robot
	print "move_arc({0}, {1})".format(rot_speed, trans_speed)
	pass

# Moving straight is moving in an arc with no rotational speed
def move_fwd(speed):
	move_arc(0, speed)

# Turning is moving in an arc with no translational speed
def move_turn(speed):
	move_arc(speed, 0)

# Stopping is moving with no velocity. Have you ever, like, REALLY, looked at your hands, man?
def stop():
	move_arc(0,0)

def dbg_print(message):
	print message
	
# Define the guards that can be sensed. Guards are boolean, so they should return either a boolean
# or something that python is going to treat as a boolean. I've decided to name the guards as 
# questions so that 

def is_near_anything():
	return is_near_left() and is_near_right() and is_near_center()

def is_near_right():
	span = len(laser_readings)/3
	#Right side of readings
	for reading in laser_readings[span+span:]:
		#If there's a close object, return true
		if reading < max_range / 2:
			return True
	return False

def is_near_left():
	span = len(laser_readings)/3
	#Left side of readings
	for reading in laser_readings[:span]:
		#If there's a close object, return true
		if reading < max_range / 2:
			return True
	return False

def is_near_center():
	span = len(laser_readings)/3
	#Center of readings
	for reading in laser_readings[span:span+span]:
		#If there's a close object, return true
		if reading < max_range / 2:
			return True
	return False

# Some state 
max_range = 10
laser_readings = [10, 10, 10, 10, 10, 10, 4, 10, 10]

class ProgramLoader(object):
	def __init__(self):
		rospy.Subscriber("/robot_prog/{0}".format(7), String, self.replaceProgram)
		# Example program, will get replaced
		# What this should do is make the robot avoid obstacles and stop when
		# it isn't near anything.
		self.program = [("not(is_near_anything())", 1.0, "stop()"),
				   ("is_near_left() and not is_near_center()", 1.0, "move_arc(-5, 2)"),
				   ("is_near_right() and not is_near_center()", 1.0, "move_arc(5, 2)"),
				   ("is_near_center()", 1.0, "move_turn(8)")]

	#This is how programs get deployed to this runner instance
	#Currently just a string containing a set of GCPR tuples
	#This isn't anything like secure, since eval is getting used. 
	#There also may be threads afety concerns
	def replaceProgram(self, msg):
		rospy.logwarn("Got {0}".format(msg.data))
		self.program = json.loads(msg.data)

	def getProgram(self):
		return self.program

rospy.init_node("gcpr_runner")
pl = ProgramLoader()


random.seed()

while(True):
	todo_list = []

	for rule in pl.getProgram():
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
		eval(item)

	sleep(2)
	print "----"