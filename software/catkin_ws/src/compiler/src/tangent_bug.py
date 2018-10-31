#!/usr/bin/python

#Implementation of tangent bug pathing for swarms

# #Original tangent bug (Kamon, Rimon, Rivlin 1998)

# # 1. Move towards the target in the locally optimal direction in the local 
# # tangent graph (This is a fancy way of saying, of all the ends of edges 
# # of things detected around the robot, move towards the one closest to 
# # the target. This requries some sensor preprocessing and global 
# # localization, but the math is simple polar coordinate stuff). 

# #If the list of tangent points is empty, point towards the goal
# #If the list of tangent points has a member that is closer to the goal than 
# #the robot's current location, point towards it

# # 1a. If the target is reached, stop.  This is success.

# # 1b. If there is no point that is closer to the target and in free space,
# # go to step two. This is the transition to wall following. 

# # 2. Choose a boundary-following direction and record the hit point. 
# # Follow the boundary while recording the minimum distance to the target. 
# #Set up the closest point, if it is set, keep track of the minimum distance to target seen so far
# # 2a. If the target is reached, stop.
# # Doesn't need to be explicit, the statement in condition 1 is redundant with this one
# #(at(self.target_point), stop(success=True), 1.0)

# # 2b. An element of the local tangent graph is closer to the target than 
# # the current minimum distance to the target. This is the leave condition. 

# # 2c. If you return to the hit point. The target is unreachable, stop. 

# # 3. Transition phase. Move directly toward the leaving point until 
# # reaching a point  that is closer to the target than the minimum distance 
# # so far. Return to step 1. 

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


# 1. Move towards the target in the locally optimal direction in the local 
# tangent graph (This is a fancy way of saying, of all the ends of edges 
# of things detected around the robot, move towards the one closest to 
# the target. This requries some sensor preprocessing and global 
# localization, but the math is simple polar coordinate stuff). 


#If the list of tangent points is empty, point towards the goal
#program_sender.append(("self.closest_tangent_point() == None", "self.state = 'drive_goal'", 1.0))
#If the list of tangent points has a member that is closer to the goal than 
#the robot's current location, point towards it
#program_sender.append(("self.closest_tangent_point() != None", "self.state = 'drive_tangent_pt'", 1.0))

# 1a. If the target is reached, stop.  This is success.
#program_sender.append(("self.at({})".format(goal), "self.stop()", 1.0))

# # 1b. If there is no point that is closer to the target and in free space,
# # go to step two. This is the transition to wall following. 

# # 2. Choose a boundary-following direction and record the hit point. 
# # Follow the boundary while recording the minimum distance to the target. 
# #Set up the closest point, if it is set, keep track of the minimum distance to target seen so far
# # 2a. If the target is reached, stop.
# # Doesn't need to be explicit, the statement in condition 1 is redundant with this one
# #(at(self.target_point), stop(success=True), 1.0)

# # 2b. An element of the local tangent graph is closer to the target than 
# # the current minimum distance to the target. This is the leave condition. 

# # 2c. If you return to the hit point. The target is unreachable, stop. 

# # 3. Transition phase. Move directly toward the leaving point until 
# # reaching a point  that is closer to the target than the minimum distance 
# # so far. Return to step 1. 


#Motion towards the goal
program.append(("self.at({})".format(goal), "self.stop()", 1.0))
program.append(("not(self.at({})) and self.on_heading()".format(goal), "self.move_fwd(0.4)", 1.0))
program.append(("not(self.at({})) and not(self.on_heading())".format(goal), "self.turn_heading(0.8)", 1.0))
program.append(("not(self.at({})) and not(self.is_near_anything())".format(goal), "self.set_desired_heading(self.get_heading({}))".format(goal), 1.0))

program.append(("not(self.at({})) and self.is_near_anything()".format(goal), "self.set_desired_heading(self.closest_free_window({}))".format(goal), 1.0))
program.append(("not(self.at({})) and self.is_near_anything() and self.hit_point is None".format(goal), "self.set_hit_point()", 1.0))
# #For debugging
# #program.append(("not(self.at({})) and not(self.hit_point is None)".format(goal),"self.stop()",1.0))
# #----- OK down to here, so far -----
# program.append(("not(self.at({})) and not(self.hit_point is None)".format(goal),"self.set_desired_heading(self.closest_free_window({}))".format(goal),1.0))

program_sender.updatePubs()
robots = program_sender.getRobots()
for robot in robots:
 	program_sender.pubProg(robot, program)

#program_sender.pubProg(robots[0], program)

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
