#!/usr/bin/python

# Given a set of points describing a gesture, convert it into a GCPR program for controlling robots

import math
import json
import rospy
from std_msgs.msg import String

# These points are intended to traverse an arena in ARGoS, in simulation, from the bottom left corner
# to the top right corner, on a slightly curved path (actually just 6 segments)

points = [(-3.5,-1.1),(-2.3,0.0),(-1.2,0.2),(0.0,0.2),(3.0,0.2),(3.5,1.0)]

#Calculate the desired heading and distance from each point to the next
headings = []
distances = []
for idx in range(len(points)-1):
	p1 = points[idx]
	p2 = points[idx+1]
	#Atan2 is (y,x), not (x,y)
	headings.append(math.atan2(p2[1]-p1[1], p1[0]-p2[0]))
	#Distances are not cartesian, but distance traveled in (signed) x and y directions,
	#which is to say they include direction
	distances.append((abs(p1[0]-p2[0]), abs(p1[1]-p2[1])))

print headings
print distances

program = []

#Build GCPR program for setting the program counter based on distance traveled
pc = 0
for heading, distance in zip(headings, distances):
	program.append(("self.pc_is({0})".format(pc), "self.set_desired_heading({0})".format(heading), 1.0))
	program.append(("self.traveled_x > {0} and self.traveled_y > {1} and self.pc_is({2})".format(distance[0], distance[1], pc), "self.set_pc({0})".format(pc+1), 1.0))
	#Reset the distances traveled when they are over the limits needed to reset the pc
	#program.append(("self.traveled_x > {0} and self.traveled_y > {1}".format(distance[0], distance[1]), "self.reset_travel()".format(pc+1), 1.0))
	pc += 1

#Add a stop condition
program.append(("self.pc_is({0})".format(pc), "self.stop()", 1.0))

#Add motion commands to turn to bearing and move forward
#Don't move after stopping because PC has hit end of program
program.append(("self.on_heading() and not(self.pc_is({0})) and not(self.is_near_anything())".format(pc), "self.move_fwd(0.3)", 1.0))
program.append(("not(self.on_heading()) and not(self.pc_is({0})) and not(self.is_near_anything())".format(pc), "self.move_turn(1)", 1.0))

#Reactive obstacle avoidance
#Could still do reactive obstacle avoidance after ending travel, but could lead to jostling out of goal
#program.append(("not(self.is_near_anything()) and not(self.pc_is({0}))".format(pc), "self.move_fwd(0.3)", 1.0))
program.append(("self.is_near_left() and not self.is_near_center() and not(self.pc_is({0}))".format(pc), "self.move_turn(-1)", 1.0))
program.append(("self.is_near_right() and not self.is_near_center() and not(self.pc_is({0}))".format(pc), "self.move_turn(1)", 1.0))
program.append(("self.is_near_center() and not(self.pc_is({0}))".format(pc), "self.move_turn(1)", 1.0))

print json.dumps(program, sort_keys=True, indent=4, separators=(',', ': '))

#publish the robot program to each of the robots
rospy.init_node("program_sender", anonymous=True)


#Build a list and keep it around so messages have time to get out
pubs = []

for robotID in range(6):
	pubs.append(rospy.Publisher('/bot{}/robot_prog'.format(robotID), String, queue_size=10))

#r = rospy.Rate(10) # 10hz
#while not rospy.is_shutdown():
for pub in pubs:
	message = json.dumps(program)
	pub.publish(message)
	rospy.sleep(0.5)

rospy.spin()
	
# pub = rospy.Publisher('/bot0/robot_prog', String, queue_size=10)
# message = json.dumps(program)

# while not rospy.is_shutdown():
# 	pub.publish(message)
# 	rospy.sleep(1.5)

rospy.spin()
