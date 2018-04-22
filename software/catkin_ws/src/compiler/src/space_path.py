#!/usr/bin/python

#Uses the space decomposer and a set of points to generate a GCPR program that 
#steers robots along a path

import math
import json
import rospy
from std_msgs.msg import String
import decompose_space

# These points are intended to traverse an arena in ARGoS, in simulation, from the bottom left corner
# to the top right corner, on a slightly curved path (actually just 6 segments)

points = [(-3.5,-1.1),(-2.3,0.0),(-1.2,0.2),(0.0,0.2),(3.0,0.2),(3.5,1.0)]

program = []

#Reactive obstacle avoidance
program.append(("self.is_near_left() and not(self.is_near_right()) and not(self.is_near_center())", "self.move_turn(-0.9)", 0.9))
program.append(("self.is_near_right() and not(self.is_near_left()) and not(self.is_near_center())", "self.move_turn(0.9)", 0.9))
#Back and turn away
program.append(("self.is_near_left() and self.is_near_right() and not(self.is_near_center())", "self.move_arc(2.0, -0.5)", 0.8))
program.append(("self.is_near_center() and not(self.is_near_right()) and not(self.is_near_left())", "self.move_arc(2.0, -0.5)", 1.0))
#Move away from stuff behind
program.append(("self.is_near_anything() and not(self.is_near_left()) and not(self.is_near_center()) and not(self.is_near_right())", "self.move_fwd(0.4)", 1.0))

# This is the bounds of the ARGoS arena. For a sytem that made its own coordinate 
# system, using the robot sensors, this could be calculated by having each robot 
# report its own location or the furthest away position that it had heard of, 
# until they converged to find the extremes of the connected group. 
space = [(-4,2),(4,-2)]

dec = decompose_space.get_decomposition(space[0], space[1], points, 0.5)
for square in dec:
	program.append(("self.is_in({0}, {1})".format(square.tl, square.br), "self.set_desired_heading({0})".format(math.pi - square.heading), 0.9))

#Add motion commands to turn to bearing and move forward
program.append(("self.on_heading() and not(self.is_near_anything())", "self.move_fwd(0.3)", 1.0))
#TODO make turning to heading less awful
program.append(("not(self.on_heading()) and not(self.is_near_anything())", "self.turn_heading(1)", 1.0))

print json.dumps(program, sort_keys=True, indent=2, separators=(',', ': '))

#publish the robot program to each of the robots
rospy.init_node("program_sender", anonymous=True)

#Build a list and keep it around so messages have time to get out
pubs = []

for robotID in range(6):
	pubs.append(rospy.Publisher('/bot{}/robot_prog'.format(robotID), String, queue_size=10))

#Wait for my publishers to be ready
rospy.sleep(0.5)

for pub in pubs:
	message = json.dumps(program)
	pub.publish(message)

rospy.spin()
	