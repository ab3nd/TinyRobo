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

#GCPR for a vector-based collision avoidance

#Reactive obstacle avoidance
program.append(("self.is_near_left() and not(self.is_near_right()) and not(self.is_near_center())", "self.move_turn(-0.9)", 0.9))
program.append(("self.is_near_right() and not(self.is_near_left()) and not(self.is_near_center())", "self.move_turn(0.9)", 0.9))
#Back and turn away
program.append(("self.is_near_left() and self.is_near_right() and not(self.is_near_center())", "self.move_arc(2.0, -0.5)", 0.8))
program.append(("self.is_near_center() and not(self.is_near_right()) and not(self.is_near_left())", "self.move_arc(2.0, -0.5)", 1.0))
#Move away from stuff behind
program.append(("self.is_near_anything() and not(self.is_near_left()) and not(self.is_near_center()) and not(self.is_near_right())", "self.move_fwd(0.4)", 1.0))

#Move around
program.append(("not(self.is_near_anything())", "self.move_fwd(0.3)", 0.8))
program.append(("not(self.is_near_anything())", "self.move_arc(0.3, 0.5)", 0.2))
program.append(("not(self.is_near_anything())", "self.move_arc(-0.3, 0.5)", 0.2))

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
