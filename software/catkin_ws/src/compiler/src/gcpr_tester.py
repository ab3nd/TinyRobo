#!/usr/bin/python

#Script for testing a GCPR program by hand before adding it to the compiler
import rospy
import re
import json
from std_msgs.msg import String

class ProgSender(object):
	def __init__(self):
		self.robotPubs = {}
		self.topicRE = re.compile("\/bot([0-9]*)\/cmd_vel")

	def updatePubs(self):
		#We can't get a list via rospy.get_published_topics(), because this node
		#is what's supposed to be publishing the topics, so get the IDs by looking at the 
		#other /botN/* topics
		for topic in rospy.get_published_topics():
			if self.topicRE.match(topic[0]):
				robot_id = self.topicRE.match(topic[0]).group(1)
				if robot_id not in self.robotPubs.keys():
					self.robotPubs[robot_id] = rospy.Publisher('/bot{}/robot_prog'.format(robot_id), String, queue_size=10)
		#Wait for publishers to be ready, and yes, this can be a problem. 
		rospy.sleep(0.5)

	def getRobots(self):
		return self.robotPubs.keys()

	def pubProg(self, robot, program):
		#If we don't have a publisher for this robot yet, create one
		if robot not in self.robotPubs.keys():
			self.robotPubs[robot] = rospy.Publisher('/bot{}/robot_prog'.format(robot), String, queue_size=10)
			#Wait for publisher to be ready, and yes, this can be a problem. 
			rospy.sleep(0.5)

		msg = json.dumps(program)
		self.robotPubs[robot].publish(msg)
		print "Sent {0} the program:\n {1}".format(robot, msg)

# #Start everything up and then just spin
rospy.init_node('tangent_bug')

program_sender = ProgSender()

program = []

#Very simple program, move forward if not near anything
program.append(("self.get_l_front() == self.get_r_front() == 0 and self.proxReadings[6].value == 0", "self.move_fwd(0.4)", 1.0))

#If there is something in the way, rotate (always the same direction, setting up for edge follow)
program.append(("self.get_l_front() > 0 or self.get_r_front() > 0", "self.move_turn(-1.2)", 1.0))
#If there's something in the way and we haven't set the hit point yet, set it. It needs to get unset when we leave the object, though
program.append(("self.hit_point is None and (self.get_l_front() > 0 or self.get_r_front() > 0)", "self.set_hit_point()", 1.0))

#We're in the right range, work on alignment
program.append(("self.get_l_front() == self.get_r_front() == 0 and ((0.1 < self.proxReadings[5].value < 0.2) and (0.1 < self.proxReadings[6].value < 0.2)) and self.proxReadings[5].value > self.proxReadings[6].value", "self.move_arc(-0.3, 0.2)", 1.0))
program.append(("self.get_l_front() == self.get_r_front() == 0 and ((0.1 < self.proxReadings[5].value < 0.2) and (0.1 < self.proxReadings[6].value < 0.2)) and self.proxReadings[5].value < self.proxReadings[6].value", "self.move_arc(0.3, 0.2)", 1.0))

#Fix range, we're too close or too far
program.append(("self.get_l_front() == self.get_r_front() == 0 and self.proxReadings[5].value > 0.2 and self.proxReadings[6].value > 0.2", "self.move_arc(-1.3, 0.3)", 1.0))
program.append(("self.get_l_front() == self.get_r_front() == 0 and 0.0 < self.proxReadings[5].value < 0.1 and 0.0 < self.proxReadings[6].value < 0.1", "self.move_arc(1.3, 0.3)", 1.0))

#Attempt to turn corners of things when wall following, 5 is forward of 6 and so should go off first
program.append(("self.get_l_front() == self.get_r_front() == 0 and self.proxReadings[5].value == 0 and self.proxReadings[6].value > 0.0", "self.move_turn(1.9)", 1.0))

#TODO hack, just explicitly set the goal if it isn't set
program.append(("self.goal is None", "self.set_goal((3,0))", 1.0))

# update checks for leave conditions for tangent bug with range:
# dFollow - shortest distance from a point the robot has detected to be occupied while wall following to the goal
# dReach - shortest distance from a point the robot can sense is free to the goal
program.append(("self.get_l_front() == self.get_r_front() == 0 and ((0.1 < self.proxReadings[5].value < 0.2) and (0.1 < self.proxReadings[6].value < 0.2))", "self.update_distances()", 1.0))

# Check the tangent bug leave condition: We can see a point closer to the goal than anywhere along the edge of the 
# object that we're following, so head for that point instead
program.append(("(self.dReach < self.dFollow) and (self.get_l_front() == self.get_r_front() == 0 and ((0.1 < self.proxReadings[5].value < 0.2) and (0.1 < self.proxReadings[6].value < 0.2)))", "rospy.logwarn(\"{} would leave.\".format(self.ns))", 1.0))

#TODO eventually we're going to want to unset the hit point

#Send to either one robot or all of them
one_robot = False
program_sender.updatePubs()
robots = program_sender.getRobots()

if one_robot:
	program_sender.pubProg(robots[0], program)
else:
	for robot in robots:
	 	program_sender.pubProg(robot, program)



rospy.spin()

# Wedge-free travel, for my own future reference
# #Very simple program, move forward if not near anything
# program.append(("self.get_l_front() == self.get_r_front() == 0", "self.move_fwd(0.4)", 1.0))
# #Mutually exclusve and symmetry-breaking turning away from obstacles
# program.append(("self.get_l_front() > 0 and self.get_r_front() == 0", "self.move_arc(-1.2, 0.1)", 1.0))
# program.append(("self.get_l_front() == 0 and self.get_r_front() > 0", "self.move_arc(1.2, 0.1)", 1.0))
# program.append(("self.get_l_front() > 0 and self.get_r_front() > 0", "self.move_turn(-1.2)", 1.0))


#NEGATIVE ANGLES ARE TO THE ROBOT'S LEFT, DON'T FUCK UP
