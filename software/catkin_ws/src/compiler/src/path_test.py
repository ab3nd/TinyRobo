#!/usr/bin/python

#Script to, given a bunch of points, emit a program that uses a pc to move between those points

path = [(1,2),(3,4),(5,6),(7,8),(9,9),(8,7),(6,5)]

for idx, point in enumerate(path):
	if idx == 0:
		#First point, just set it as the goal
		print "self.set_goal({})".format(point)
		print "self.set_pc({})".format(idx)
	elif idx == len(path)-1:
		#Last point
		#If the point is not reachable or we're there, stop
		print "self.pc == {} and self.not_reachable({})".format(idx, point), "self.stop()"
		print "self.pc == {} and self.at({})".format(idx, point), "self.stop()"
	else:
		#If we made it there, move on to the next point
		print "self.pc == {} and self.at({})".format(idx, point), "self.set_pc({})".format(idx+1)
		print "self.at({})".format(point), "self.set_goal({})".format(path[idx+1])
		#If it's not reachable, increment the pc and move on to the next point
		print "self.pc == {} and self.not_reachable({})".format(idx, point), "self.set_pc({})".format(idx+1)
		print "self.not_reachable({})".format(point), "self.set_goal({})".format(path[idx+1])

#Some postamble stuff to do the actual following
print "not(self.at(self.get_goal())", "self.move_goal()"