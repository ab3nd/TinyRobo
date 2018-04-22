#!/usr/bin/python

#Uses the space decomposer and a set of points to generate a GCPR program that 
#steers robots along a path

import math
import json
import rospy
from std_msgs.msg import String
import decompose_space
from geometry_msgs.msg import Pose
import random

class PositionHandler(object):
	def __init__(self, id):
		self.id = id
		self.position = None

	def updatePosition(self, msg):
		self.position = msg

	def getPosition(self):
		return self.position

#Subscribe to the positions of all the robots
#This instantiates a subscriber for anything whose topic ends in "position"
class PositionListener(object):
	def __init__(self):
		self.handlers = {}

		#Get all the topics that are called "position"
		topics = rospy.get_published_topics()
		pos_topics = [t for t in topics if t[0].endswith("position")]
		#Set up handlers for all of them
		for topic in pos_topics:
			#Assumes the names are of the form "/botX/position"
			botID = topic[0].split("/")[1][3:]
			handler = PositionHandler(botID)
			sub = rospy.Subscriber(topic[0], Pose, handler.updatePosition)
			self.handlers[botID] = [sub, handler]

	def getPosition(self, botID):
		return self.handlers[botID][1].getPosition()

	#Not great form to have this here, but I have the info...
	def getRobotList(self):
		return self.handlers.keys()

def pickPositions(tl, br, count):
	positions = []
	#TODO these might end up close to each other, or the walls
	for ii in range(count):
		x = random.uniform(tl[0], br[0])
		y = random.uniform(tl[1], br[1])
		positions.append((x,y))
	return positions


if __name__ == "__main__":

	#tl and br corners of space
	space = [(-4,2),(4,-2)]

	rospy.init_node("program_sender", anonymous=True)

	#Start up a position listener to get the positions of all of the robots
	pl = PositionListener()

	#wait for all the positions to be non-None
	positions = [None]
	while not all(positions):
		positions = [pl.getPosition(x) for x in pl.getRobotList()]


	#Pick new dispersed positions for all the robots
	new_positions = pickPositions(space[0], space[1], len(positions))

	#Pair up ids and current positions
	#Frankly, I'm not sure this is more readable than a for loop...
	new_pos = {botID:(pl.getPosition(botID), new) for (new, botID) in zip(new_positions, pl.getRobotList())}

	#Build a list and keep it around so messages have time to get out
	pubs = []

	for bot in new_pos.keys():
		program = []
		#Reactive obstacle avoidance
		program.append(("self.is_near_left() and not(self.is_near_right()) and not(self.is_near_center())", "self.move_turn(-0.9)", 0.9))
		program.append(("self.is_near_right() and not(self.is_near_left()) and not(self.is_near_center())", "self.move_turn(0.9)", 0.9))
		#Back and turn away
		program.append(("self.is_near_left() and self.is_near_right() and not(self.is_near_center())", "self.move_arc(2.0, -0.5)", 0.8))
		program.append(("self.is_near_center() and not(self.is_near_right()) and not(self.is_near_left())", "self.move_arc(2.0, -0.5)", 1.0))
		#Move away from stuff behind
		program.append(("self.is_near_anything() and not(self.is_near_left()) and not(self.is_near_center()) and not(self.is_near_right())", "self.move_fwd(0.4)", 1.0))

		#Get the point the robot is at and the point it should go to
		points = [(new_pos[bot][0].position.x, new_pos[bot][0].position.y), new_pos[bot][1]]

		print "Move is from {} to {}".format(points[0], points[1])

		dec = decompose_space.get_decomposition(space[0], space[1], points, 0.5)
		for square in dec:
			program.append(("self.is_in({0}, {1})".format(square.tl, square.br), "self.set_desired_heading({0})".format(math.pi - square.heading), 0.9))

		#Add motion commands to turn to bearing and move forward
		program.append(("self.on_heading() and not(self.is_near_anything())", "self.move_fwd(0.3)", 1.0))
		#TODO make turning to heading less awful
		program.append(("not(self.on_heading()) and not(self.is_near_anything())", "self.turn_heading(1)", 1.0))

		#Create a publisher to send the program to the robot
		print "Sending to /bot{}/robot_prog".format(bot)
		pub = rospy.Publisher('/bot{}/robot_prog'.format(bot), String, queue_size=10)
		message = json.dumps(program)
		print json.dumps(program, indent=4)
		pub.publish(message)
		pubs.append(pub)

		#May not really be needed... cargo cult
		rospy.sleep(0.5)

	#Done, spin and wait
	rospy.spin()



