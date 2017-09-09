#!/usr/bin/python

import random
from time import sleep

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

# Example program, this will be loaded from a file in later versions
# What this should do is make the robot avoid obstacles and stop when
# it isn't near anything.
program = [("not(is_near_anything())", 1.0, "stop"),
		   ("is_near_left() and not is_near_center()", 1.0, "move_arc(-5, 2)"),
		   ("is_near_right() and not is_near_center()", 1.0, "move_arc(5, 2)"),
		   ("is_near_center()", 1.0, "move_turn(8)")]

# Loader and runner

# This is where we'd load the program

random.seed()

while(True):
	todo_list = []

	for rule in program:
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