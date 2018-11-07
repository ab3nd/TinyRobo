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

goal = (3,0)

#Initially, pick a random heading to travel on
program.append(("self.pc_is(0)", "self.set_desired_heading((random.random() * math.pi * 2)-math.pi)", 1.0))
program.append(("self.pc_is(0)", "self.set_pc(1)", 1.0))

#Move towards heading if not near anything
program.append(("len(self.neighbors(d=4)) > 1 and not(self.is_near_anything()) and self.on_heading()", "self.move_fwd(0.4)", 1.0))
program.append(("len(self.neighbors(d=4)) > 1 and not(self.is_near_anything()) and not(self.on_heading())", "self.turn_heading(0.8)", 1.0))

# Too few neighbors, turn around		
program.append(("len(self.neighbors(d=4)) < 1", "self.set_desired_heading(self.add_headings(self.current_heading, math.pi))", 1.0))
#Just right neighbors, stop
program.append(("len(self.neighbors(d=4)) == 1", "self.stop()", 1.0))

#Avoid objects, by turning away from the object
#Symmetry breaking
program.append(("self.is_near_left_f_quarter() and self.is_near_right_f_quarter()", "self.move_turn(-0.8)", 1.0))
#Avoid object
program.append(("self.is_near_left_f_quarter() and not(self.is_near_right_f_quarter())", "self.move_turn(0.8)", 1.0))
program.append(("not(self.is_near_left_f_quarter()) and self.is_near_right_f_quarter()", "self.move_turn(-0.8)", 1.0))
program.append(("self.is_near_anything() and not(self.is_near_left_f_quarter()) and not(self.is_near_right_f_quarter())", "self.move_fwd(0.4)", 1.0))

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

#NEGATIVE ANGLES ARE TO THE ROBOT'S LEFT, DON'T FUCK UP

# #Stop at the goal 
# program.append(("self.at({})".format(goal), "self.stop()", 1.0))

# #Just go forwards
# program.append(("not(self.is_near_left_f_quarter()) and not(self.is_near_right_f_quarter()) and not(self.is_near_right_r_quarter()) and not (self.is_near_left_r_quarter())", "self.move_fwd(0.3)", 1.0))

# #Symmetry breaking
# program.append(("self.is_near_left_f_quarter() and self.is_near_right_f_quarter()", "self.move_turn(-0.8)", 1.0))

# #Turn to have the right side near the object, 18 is just forward of 17
# program.append(("not(self.is_near_left_f_quarter()) and self.is_near_right_f_quarter() and (self.proxReadings[19].value == 0 and self.proxReadings[17].value== 0)", "self.move_turn(-0.8)", 1.0))
# #Keep the object to the side
# program.append(("not(self.is_near_left_f_quarter()) and self.is_near_right_f_quarter() and ((0.0 < self.proxReadings[19].value < 0.3) or (0.0 < self.proxReadings[17].value < 0.3)) and self.proxReadings[17].value > self.proxReadings[18].value", "self.move_arc(-0.3, 0.2)", 1.0))
# program.append(("not(self.is_near_left_f_quarter()) and self.is_near_right_f_quarter() and ((0.0 < self.proxReadings[19].value < 0.3) or (0.0 < self.proxReadings[17].value < 0.3)) and self.proxReadings[17].value < self.proxReadings[18].value", "self.move_arc(0.3, 0.2)", 1.0))# if 17 > 18, turn r
# #Try to get around corners
# program.append(("not(self.is_near_right_f_quarter()) and self.is_near_right_r_quarter()", "self.move_turn(0.8)", 1.0))
# #Keep the range better
# program.append(("self.proxReadings[19].value > 0.3 or self.proxReadings[17].value > 0.3", "self.move_arc(-1.3, 0.2)", 1.0))

# #Turn to have the left side to the object, 5 is forward of 6
# program.append(("self.is_near_left_f_quarter() and not(self.is_near_right_f_quarter()) and (self.proxReadings[4].value == 0 and self.proxReadings[6].value == 0)", "self.move_turn(0.8)", 1.0))
# #Keep the object to one side
# program.append(("self.is_near_left_f_quarter() and not(self.is_near_right_f_quarter()) and ((0.0 < self.proxReadings[4].value < 0.3) or (0.0 < self.proxReadings[6].value < 0.3)) and self.proxReadings[5].value > self.proxReadings[6].value", "self.move_arc(-0.3, 0.2)", 1.0))
# program.append(("self.is_near_left_f_quarter() and not(self.is_near_right_f_quarter()) and ((0.0 < self.proxReadings[4].value < 0.3) or (0.0 < self.proxReadings[6].value < 0.3)) and self.proxReadings[5].value < self.proxReadings[6].value", "self.move_arc(0.3, 0.2)", 1.0))
# #Attempt to turn sharply when off the edge of something the robot is following
# program.append(("not(self.is_near_left_f_quarter()) and self.is_near_left_r_quarter()", "self.move_turn(-0.8)", 1.0))
# #Fix range
# program.append(("self.proxReadings[4].value > 0.3 or self.proxReadings[6].value > 0.3", "self.move_arc(1.3, 0.2)", 1.0))


# #Add motion commands to turn to bearing and move forward
# #program.append(("not(self.is_near_front())", "self.set_desired_heading(self.get_heading({}))".format(goal), 1.0))
# program.append(("not(self.proxReadings[4].value != 0 or self.proxReadings[6].value != 0 or self.proxReadings[19].value != 0 or self.proxReadings[17].value != 0) and self.on_heading()", "self.move_fwd(0.3)", 1.0))
# program.append(("not(self.proxReadings[4].value != 0 or self.proxReadings[6].value != 0 or self.proxReadings[19].value != 0 or self.proxReadings[17].value != 0) and not(self.on_heading())", "self.turn_heading_arc(1,0.2)", 1.0))
# #Keep the object to the side
# program.append(("((0.0 < self.proxReadings[19].value < 0.3) or (0.0 < self.proxReadings[17].value < 0.3)) and self.proxReadings[17].value > self.proxReadings[18].value", "self.move_arc(0.3, 0.2)", 1.0))
# program.append(("((0.0 < self.proxReadings[19].value < 0.3) or (0.0 < self.proxReadings[17].value < 0.3)) and self.proxReadings[17].value < self.proxReadings[18].value", "self.move_arc(-0.3, 0.2)", 1.0))
# program.append(("self.proxReadings[19].value > 0.3 or self.proxReadings[17].value > 0.3", "self.move_arc(1.3, 0.2)", 1.0))

# # #Keep the object to one side
# program.append(("((0.0 < self.proxReadings[4].value < 0.3) or (0.0 < self.proxReadings[6].value < 0.3)) and self.proxReadings[5].value > self.proxReadings[6].value", "self.move_arc(-0.3, 0.2)", 1.0))
# program.append(("((0.0 < self.proxReadings[4].value < 0.3) or (0.0 < self.proxReadings[6].value < 0.3)) and self.proxReadings[5].value < self.proxReadings[6].value", "self.move_arc(0.3, 0.2)", 1.0))
# program.append(("self.proxReadings[4].value > 0.3 or self.proxReadings[6].value > 0.3", "self.move_arc(-1.3, 0.2)", 1.0))

# # #Move towards a free point nearest to the goal
# program.append(("self.proxReadings[4].value == self.proxReadings[6].value == self.proxReadings[17].value == self.proxReadings[19].value== 0.0", "self.set_desired_heading(self.get_heading(self.closest_free_point({})))".format(goal), 1.0))
# program.append(("self.proxReadings[4].value == self.proxReadings[6].value == self.proxReadings[17].value == self.proxReadings[19].value== 0.0 and self.on_heading()", "self.move_fwd(0.3)", 1.0))
# program.append(("self.proxReadings[4].value == self.proxReadings[6].value == self.proxReadings[17].value == self.proxReadings[19].value== 0.0 and not(self.on_heading())", "self.turn_heading(1)", 1.0))

# #Different attempt
# program.append(("not(self.is_near_anything())", "self.set_desired_heading(self.get_heading({}))".format(goal), 1.0))
# program.append(("not(self.is_near_anything()) and self.on_heading()", "self.move_fwd(0.5)", 1.0))
# program.append(("not(self.is_near_anything()) and not(self.on_heading())", "self.turn_heading(0.5)", 1.0))
# program.append(("self.is_near_anything()", "self.wall_follow({})".format(goal), 1.0))
# # program.append(("self.is_near_anything()", "self.stop()", 1.0))
