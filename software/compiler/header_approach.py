#!/usr/bin/python

# Given a set of points describing a gesture, convert it into a GCPR program for controlling robots

import math

# These points are intended to traverse an arena in ARGoS, in simulation, from the bottom left corner
# to the top right corner, on a slightly curved path (actually just 6 segments)

points = [(-3.5,-1.1),(-2.3,0.0),(-1.2,0.2),(0.0,0.2),(3.0,0.2),(3.5,1.0)]

#Calculate the desired heading and distance from each point to the next
headings = []
distances = []
for idx in range(len(points)-1):
	p1 = points[idx]
	p2 = points[idx+1]
	headings.append(math.atan2(p2[0]-p1[0], p1[1]-p2[1]))
	#Distances are not cartesian, but distance traveled in (signed) x and y directions,
	#which is to say they include direction
	distances.append((p1[0]-p2[0], p1[1]-p2[1]))

print headings
print distances

#Build GCPR program for setting the program counter based on distance traveled
pc = 0
for heading, distance in zip(headings, distances):
	print "self.pc_is({0}), self.set_desired_heading({1}), 1.0".format(pc, heading)
	print "self.distance_x > {0} and self.distance_y > {1}, self.set_pc({2}), 1.0".format(distance[0], distance[1], pc+1)
	pc += 1

#Add a stop condition
print "self.pc_is({0}), self.stop(), 1.0".format(pc)

#Add motion commands to turn to bearing and move forward
#Don't move after stopped because PC has hit end of program
print "self.on_heading() and not(self.pc_is({0})), 1.0, self.move_fwd(0.3)".format(pc)
print "not(self.on_heading()) and not(self.pc_is({0})), 1.0, self.move_turn(0.3)".format(pc)

#Reactive obstacle avoidance
#Could still do reactive obstacle avoidance after ending travel, but could lead to jostling out of goal
print "not(self.is_near_anything()) and not(pc_is({0}))", "1.0", "self.move_fwd(0.3)".format(pc)
print "self.is_near_left() and not self.is_near_center() and not(pc_is({0}))", "1.0", "self.move_arc(-0.3, 0.25)".format(pc)
print "self.is_near_right() and not self.is_near_center() and not(pc_is({0}))", "1.0", "self.move_arc(0.3, 0.25)".format(pc)
print "self.is_near_center() and not(pc_is({0}))", "1.0", "self.move_turn(0.3)".format(pc)
