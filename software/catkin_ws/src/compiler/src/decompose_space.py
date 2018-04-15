#!/usr/bin/python

#Decomposes a space into voroni cells, and figures out the vector for each 
#cell to drive robots in that cell along a specific path

#For visualization
import pygame
import sys
import numpy as np
import random
import math
import copy

def to_pygame(r, width = 1024, height = 768, ppm = 120):
	#Get the width and height of the rectangle in pixels
	rect_w = (abs(r[0][0]) + abs(r[1][0])) * ppm
	rect_h = (abs(r[0][1]) + abs(r[1][1])) * ppm

	#Get the top left corner in pixels
	tl_x = r[0][0] * ppm
	tl_y = -r[0][1] * ppm # negate because increasing y moves down the screen, not up

	#Convert to offset from center
	tl_x += width/2
	tl_y += height/2
	
	return pygame.Rect(tl_x, tl_y, rect_w, rect_h)

def point_to_pygame(pt, width = 1024, height = 768, ppm = 120):
	#Get the top left corner in pixels
	tl_x = pt[0] * ppm
	tl_y = -pt[1] * ppm # negate because increasing y moves down the screen, not up

	#Convert to offset from center
	tl_x += width/2
	tl_y += height/2

	tl_x = int(tl_x)
	tl_y = int(tl_y)

	return (tl_x, tl_y)

def pg_dbg(space, decomp, points):
	pygame.init()
	
	size = width,height = 1024,768
	screen = pygame.display.set_mode(size)
	screen.fill((255,255,255))

	#Draw the space
	green = (0,128,0)
	#Pygame rects are left, top, width, height
	pygame.draw.rect(screen, green, to_pygame(space) ,0)

	for sq in decomp:
		
		#Draw a vector for each decomp grid square
		#Get the center point
		center = sq.get_center()
		pg_center = point_to_pygame(center)

		#Get a point that's half the width away on the heading
		hx = center[0] + (sq.width/2.0 * math.cos(sq.heading))
		hy = center[1] + (sq.width/2.0 * math.sin(sq.heading))
		pg_vector = point_to_pygame([hx, hy])


		#Conditional coloring of vectors
		color = (0,200,0)
		if sq.isAssigned:
			color = (100, 100, 0)

		pygame.draw.line(screen, color, (pg_center[0], pg_center[1]), (pg_vector[0], pg_vector[1]), 2)
		pygame.draw.rect(screen, (0,0,190), pygame.Rect(pg_center[0], pg_center[1], 2, 2))
		#Add the vector as text
		# font = pygame.font.SysFont('freemono', 10)
		# text = font.render("{0:.3f}".format(sq.heading), True, (0,0,0))
		# screen.blit(text, (pg_center[0]-10, pg_center[1] + 5))
	
		#Draw the right and bottom edges of each square
		br = point_to_pygame(sq.br)
		tr = point_to_pygame((sq.br[0], sq.br[1]+sq.height))
		bl = point_to_pygame((sq.br[0]-sq.width, sq.br[1]))
		pygame.draw.line(screen, (0,150,0), br, tr)
		pygame.draw.line(screen, (0,150,0), br, bl)

	#Draw all the points
	for point in points:
		ppt = point_to_pygame(point)
		pygame.draw.circle(screen, (0,0,180), (ppt[0], ppt[1]), 4, 1)


	#Draw lines between the points
	for pIdx in range(len(points)-1):
		pygame.draw.line(screen, (0,0,180), point_to_pygame(points[pIdx]), point_to_pygame(points[pIdx+1]))

	#Draw a circle in the middle of the screen
	#These appear nested if point_to_pygame works right
	# scr_ctr = (0,0)
	# scr_ctr = point_to_pygame(scr_ctr)
	# pygame.draw.circle(screen, (0,0,180), scr_ctr, 5, 1)
	# pygame.draw.circle(screen, (0,0,180), (width/2, height/2), 3, 1)	

	pygame.display.flip()

	while True:
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				pygame.quit(); sys.exit();


def isIn(point, sq):
	if (sq.br[0] >= point[0] and sq.tl[0] <= point[0] ) and (sq.br[1] <= point[1] and sq.tl[1] >= point[1]):
		return True
	return False

def intersection(a1, a2, b1, b2):
	x1 = a1[0]
	y1 = a1[1]
	x2 = a2[0]
	y2 = a2[1]
	x3 = b1[0]
	y3 = b1[1]
	x4 = b2[0]
	y4 = b2[1]
	denominator = (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4)
	if denominator == 0:
		return None #Parallel
	#Get the intersection point
	px = (((x1 * y2) - (y1 * x2)) * (x3 - x4) - (x1 - x2) * ((x3 * y4) - (y3 * x4)))/denominator
	py = (((x1 * y2) - (y1 * x2)) * (y3 - y4) - (y1 - y2) * ((x3 * y4) - (y3 * x4)))/denominator

	return (px, py)

def point_line_distance(end1, end2, point):
	x0 = point[0]
	y0 = point[1]
	x1 = end1[0]
	y1 = end1[1]
	x2 = end2[0]
	y2 = end2[1]

	d = abs((y2-y1) * x0 - (x2-x1) * y0 + (x2*y1) - (y2 * x1))/math.sqrt(math.pow((y2 - y1), 2) + math.pow((x2 - x1), 2))

	return d

def isBetween(pointA, pointB, sq):
	#A grid square can only be between two points if it is inside the rectangle that bounds the two points
	# bound_tl = (min(pointA[0], pointB[0]), max(pointA[1], pointB[1]))
	# bound_br = (max(pointA[0], pointB[0]), min(pointA[1], pointB[1]))

	# bbox = grid_sq(bound_tl, bound_br, 0)
	# if isIn(sq.tl, bbox) or isIn(sq.br, bbox):
	#A noble attempt, but the bounding box gets really small if the line is horizontal or vertical

	#A grid square can only be between the two points if it is within half its size of the 
	#distance from the line between the two points
	cx = sq.tl[0] + sq.width/2.0
	cy = sq.tl[1] - sq.height/2.0
	if point_line_distance(pointA, pointB, (cx, cy)) < sq.width/2.0:

		#But it can't be outside of the box bounded by an expansion of the line
		bound_tl = (min(pointA[0], pointB[0]) - sq.width, max(pointA[1], pointB[1]) + sq.height)
		bound_br = (max(pointA[0], pointB[0]) + sq.width, min(pointA[1], pointB[1]) - sq.height)

		bbox = grid_sq(bound_tl, bound_br, 0)
		if isIn((cx,cy), bbox):
			return True

	#No edge intersected
	return False

class grid_sq(object):
	def __init__(self, tl = [0,0], br = [0,0], heading = 0):
		self.tl = tl
		self.br = br
		self.width = abs(tl[0] - br[0])
		self.height = abs(tl[1] - br[1])
		self.heading = heading
		self.isAssigned = False

	def assign(self, heading):
		self.heading = heading
		self.isAssigned = True

	def get_center(self):
		return (self.tl[0] + self.width/2.0, self.tl[1] - self.height/2.0)


def neighbors(decomp, len_x, len_y, index):
	neighborIdx = []

	#Top neighbor, check to prevent wrapping across top of list
	if index % len_x != 0:
		idx = index - 1
		if idx >= 0:
			neighborIdx.append(idx)

	#Bottom neighbor, check to prevent wrapping across bottom of list
	if (index + 1) % len_x != 0:
		idx = index + 1
		if idx < len_y*len_x:
			neighborIdx.append(idx)

	#Left neighbor
	idx = index - len_x
	if idx >= 0:
		neighborIdx.append(idx)

	#Right neighbor
	idx = index + len_x
	if idx < len_y * len_x:
		neighborIdx.append(idx)

	#Get the neighbors by index
	return [decomp[idx] for idx in neighborIdx]

def getDistance(p1, p2):
	#Euclidian distance
	return math.sqrt(math.pow(p1[0]-p2[0],2) + math.pow(p1[1]-p2[1],2))

def getNearest(sq, points):
	#Get the center
	cx = sq.tl[0] + sq.width/2.0
	cy = sq.tl[1] - sq.height/2.0

	#Find the closest point
	minDist = float('inf')
	ret = None
	for point in points:
		d = getDistance((cx, cy), point) < minDist
		if d < minDist:
			minDist = d
			ret = point

	return point

#Add vector to next nearest point on the path
def getNextNearest(newsq, points):
	nearest = getNearest(newsq, points)
	idx = points.index(nearest)
	#If it the end point, use it
	if idx == len(points) -1 :
		return points[idx]
	#It's not the end point
	return points[idx + 1]

#Add a point between each pair of points
def interpolate_pts(pts, times = 1):
	newpoints = []
	for idx in range(len(pts)-1):
		newpoint = ((pts[idx][0] + pts[idx+1][0])/2.0, (pts[idx][1] + pts[idx+1][1])/2.0)
		newpoints.append(pts[idx])
		newpoints.append(newpoint)
	newpoints.append(pts[-1])

	if times == 1:
		return newpoints
	else:
		return interpolate_pts(newpoints, times = times-1)

#Get the point that is the closest point on a line
def get_closest(start, end, point):
	x1 = start[0]
	y1 = start[1]
	x2 = end[0]
	y2 = end[1]
	x3 = point[0]
	y3 = point[1]
	d = math.pow((x1-x2), 2) + math.pow((y1-y2),2)
	
	#Make sure the ends of the segment are not the same
	if d == 0:
		return None

	u = ((x3 - x1)*(x2 - x1) + (y3 - y1)*(y2 - y1))/d

	#Constrain to segment
	u = max(0, min(1,u))

	#Get intersection point
	x = x1 + u*(x2-x1)
	y = y1 + u*(y2-y1)

	return (x,y)


#Get the point on the path made of points that is closest to p0
def get_closest_path(p0, points):
	closest = None
	minDist = float('inf')
	for pIdx in range(len(points)-1):
		canidate = get_closest(points[pIdx], points[pIdx + 1], p0)
		d = getDistance(p0, canidate)
		if d < minDist:
			minDist = d
			closest = canidate
	return closest

def average_angle(angles):
	x = y = 0
	for angle in angles:
		x += math.cos(angle)
		y += math.sin(angle)
	return math.atan2(y, x)


def get_heading(fromPt, toPt):
	return math.atan2(fromPt[1] - toPt[1], fromPt[0] - toPt[0])
	#return math.atan2(toPt[1] - fromPt[1], toPt[0] - fromPt[0])

def assign_path(x_coords, y_coords, width, height, points):
	#Assign the basic path
	decomp = []
	for y in y_coords:
		for x in x_coords:
			sq = grid_sq([x, y], [x + width, y - height], 0)# math.pi * (random.random() - 0.5))

			# if isIn((0,0), sq):
			# 	sq.assign(0)
			for pointIdx in range(len(points)-1):
				if isIn(points[pointIdx], sq):
					#Heading of each point is towards next point
					thisSq = points[pointIdx]
					nextSq = points[pointIdx+1]
					sq.assign(get_heading(nextSq, thisSq))
				elif isBetween(points[pointIdx], points[pointIdx+1], sq):
					#Heading is from square center to next point
					nextPt = points[pointIdx+1]
					center = sq.get_center()
					sq.assign(get_heading(nextPt, center))

			decomp.append(sq)

	return decomp

def assign_end(x_coords, y_coords, points, decomp):
	decomp_2 = []
	for index, sq in enumerate(decomp):
		newsq = copy.deepcopy(sq)
	
		#Don't reassign
		for neighbor in neighbors(decomp, len(x_coords), len(y_coords), index):
			if isIn(points[-1], neighbor):
				if not newsq.isAssigned:
					#Point it towards the end point
					center = newsq.get_center()
					newsq.assign(get_heading(points[-1], center))
		decomp_2.append(newsq)
	return decomp_2

def assign_outside_path(x_coords, y_coords, decomp):
	#Assign all the spaces which are next to at least one assigned space
	#This builds a new decomposition to not keep copying updated values
	decomp_2 = []
	for index, sq in enumerate(decomp):
		newsq = copy.deepcopy(sq)
	
		#Don't reassign
		if not newsq.isAssigned:
			avg_heading = 0
			count = 0

			#Get the data to set this square's heading to the average
			angles = []
			for neighbor in neighbors(decomp, len(x_coords), len(y_coords), index):
				if neighbor.isAssigned:
					angles.append(neighbor.heading)
			if len(angles) > 0:
				#Float to avoid integer math data loss
				avg_heading = average_angle(angles)
				newsq.assign(avg_heading)
		decomp_2.append(newsq)
	return decomp_2

def assign_remaining(x_coords, y_coords, points, decomp):
	#Assign all the spaces which are next to at least one assigned space
	#This builds a new decomposition to not keep copying updated values
	changed = True
	decomp_old = decomp
	decomp_new = []

	#Keeps updating points, based on the average of their assigned neigbors, 
	#until no points are assigned.
	while changed:

		#Set flag to indicate we're done
		changed = False

		for index, sq in enumerate(decomp_old):
			newsq = copy.deepcopy(sq)
		
			#Don't reassign
			if not newsq.isAssigned:
				avg_heading = 0
				count = 0

				#Get the data to set this square's heading to the average
				angles = []
				for neighbor in neighbors(decomp_old, len(x_coords), len(y_coords), index):
					if neighbor.isAssigned:
						angles.append(neighbor.heading)
				if len(angles) > 0:
					#TODO these don't drive to path particularly hard
					#Add vector to next nearest point on the path
					#nearest = getNextNearest(newsq, points)
					#Alternative, drives to nearest rather than next nearest
					#nearest = getNearest(newsq, points)

					nearest = get_closest_path(newsq.get_center(), points)

					#Get heading from the center of this square to the next point
					center = newsq.get_center()

					heading = get_heading(nearest, center)

					#Combine with previously collected values
					angles.append(heading)
					#Float to avoid integer math data loss
					avg_heading = average_angle(angles)
					newsq.assign(avg_heading)

					#We did make a change, so reset the flag
					changed = True

			decomp_new.append(newsq)

		if changed:
			#a change was made, so prepare to do a new sweep
			decomp_old = decomp_new
			decomp_new = []

	return decomp_new




#Given the top left and bottom right corners of a space, a list of points in the space, and
#the resolution of the decomposition, return a decomposition of the space into grid squares,
#each of which contains the heading that a robot in the square should move towards in 
#order to travel along the path
def get_decomposition(space_tl, space_br, points, resolution = 0.15):
	space_w = (abs(space_tl[0]) + abs(space_br[0]))
	space_h = (abs(space_tl[1]) + abs(space_br[1]))
	#Count of spaces
	width_c = space_w/resolution
	height_c = space_h/resolution
	#Size of spaces
	width = space_w/width_c
	height = space_h/height_c

	x_coords = np.linspace(space_tl[0], space_br[0], width_c, endpoint = False)
	y_coords = np.linspace(space_tl[1], space_br[1], height_c, endpoint = False)

	#Assign the basic path
	decomp = assign_path(x_coords, y_coords, width, height, points)

	#Assign points around end point to point in
	decomp = assign_end(x_coords, y_coords, points, decomp)

	#Assign headings for points around path
	#This embiggens the path so it's not just one grid square wide
	decomp = assign_outside_path(x_coords, y_coords, decomp)
	
	#Assign all reminaing points based on the average of their assigned neighbors
	decomp = assign_remaining(x_coords, y_coords, points, decomp)

	return decomp

if __name__=="__main__":
	
	#Points on the path in the space
	points = [(-3.5,-1.1),(-2.3,0.0),(-1.2,0.2),(0.0,0.2),(3.0,0.22),(3.5,1.0)]

	#tl and br corners of space
	space = [(-4,2),(4,-2)]

	#Assign the basic path
	decomp = get_decomposition(space[0], space[1], points)

	pg_dbg(space, decomp, points)

