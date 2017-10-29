#!/usr/bin/python

import random
from time import sleep

import rospy

# Ties together "laser" scanner, gcpr programs, and a physical robot
# There should be an instance of this for each robot

class ProgramLoader(object):
	def __init__(self):
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
	#There also may be thread safety concerns
	def replaceProgram(self, msg):
		rospy.logwarn("Got {0}".format(msg.data))
		self.program = json.loads(msg.data)

	def getProgram(self):
		return self.program


class GCPR_driver(object):
	def __init__(self):
		self.programLoader = ProgramLoader()
		pass

	def update_laser(self, laserMsg):
		pass

	def recv_msg(self, msg):
		pass

	def replace_program(self, msg):
		self.programLoader.replaceProgram(msg)

	def run_gcpr(self)
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
			eval(item)


	# Basic action of moving along an arc
	def move_arc(self, rot_speed, trans_speed):
		#TODO this is where we'd actually have ROS send the message to the robot
		print "move_arc({0}, {1})".format(rot_speed, trans_speed)
		pass

	# Moving straight is moving in an arc with no rotational speed
	def move_fwd(self, speed):
		move_arc(0, speed)

	# Turning is moving in an arc with no translational speed
	def move_turn(self, speed):
		move_arc(speed, 0)

	# Stopping is moving with no velocity. Have you ever, like, REALLY, looked at your hands, man?
	def stop(self):
		move_arc(0,0)

	def dbg_print(self, message):
		print message
		
	# Define the guards that can be sensed. Guards are boolean, so they should return either a boolean
	# or something that python is going to treat as a boolean. I've decided to name the guards as 
	# questions so that 

	def is_near_anything(self):
		return is_near_left() and is_near_right() and is_near_center()

	def is_near_right(self):
		span = len(laser_readings)/3
		#Right side of readings
		for reading in laser_readings[span+span:]:
			#If there's a close object, return true
			if reading < max_range / 2:
				return True
		return False

	def is_near_left(self):
		span = len(laser_readings)/3
		#Left side of readings
		for reading in laser_readings[:span]:
			#If there's a close object, return true
			if reading < max_range / 2:
				return True
		return False

	def is_near_center(self):
		span = len(laser_readings)/3
		#Center of readings
		for reading in laser_readings[span:span+span]:
			#If there's a close object, return true
			if reading < max_range / 2:
				return True
		return False


rospy.init_node("gcpr_driver", anonymous=True)

gDriver = GCPR_driver()

#Get the ID of the robot that this instance of the driver is driving
robot_id = rospy.get_param("~robot_id")

#Subscribe to laser for this robot
laser_sub = rospy.Subscriber("/laser_driver_{0}".format(robot_id), Laser, gDriver.update_laser)

#TODO support for passing messages in and out
net_sub = rospy.Subscriber("/messages_to_{0}".format(robot_id), String, gDriver.recv_msg)

#Load new programs
prog_sub = rospy.Subscriber("/robot_prog/{0}".format(robot_id), String, gDriver.replaceProgram)
#TODO publish twist messages


#TODO figure out what a good rate for the driver to run at is,
# and update at that rate
r = rospy.Rate(10) # 10hz
while not rospy.is_shutdown():
    #TODO update the GCPR driver
    r.sleep()