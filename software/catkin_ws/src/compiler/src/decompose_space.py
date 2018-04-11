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

#tl and br corners of space
space = [(-4,2),(4,-2)]

def to_pygame(r, width = 640, height = 480, ppm = 70):
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

def point_to_pygame(pt, width = 640, height = 480, ppm = 70):
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
	size = width,height = 640,480
	screen = pygame.display.set_mode(size)
	screen.fill((255,255,255))

	#Draw the space
	green = (0,128,0)
	#Pygame rects are left, top, width, height
	pygame.draw.rect(screen, green, to_pygame(space) ,0)

	for sq in decomp:
		
		#Draw a vector for each decomp grid square
		#Get the center point
		center = (sq.tl[0] + sq.width/2, sq.tl[1] - sq.height/2)
		pg_center = point_to_pygame(center)

		#Get a point that's half the width away on the heading
		hx = (sq.tl[0] + sq.width/2) + (sq.width/2 * math.cos(sq.heading))
		hy = (sq.tl[1] - sq.height/2) + (sq.width/2 * math.sin(sq.heading))
		pg_vector = point_to_pygame([hx, hy])


		#Conditional coloring of vectors
		color = (0,200,0)
		if sq.isAssigned:
			color = (100, 100, 0)

		pygame.draw.line(screen, color, (pg_center[0], pg_center[1]), (pg_vector[0], pg_vector[1]), 2)
		pygame.draw.rect(screen, (0,0,190), pygame.Rect(pg_center[0], pg_center[1], 2, 2))
	
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
		if isIn((cx,cy), bbox) or isIn((cx,cy), bbox):

			#A grid square is between two points if a line between the points 
			#intersects any of the sides of the square
			tl = sq.tl
			tr = (sq.br[0], sq.tl[1])
			br = sq.br
			bl = (sq.tl[0], sq.br[1])
			
			point = intersection(tl, tr, pointA, pointB)
			if point is not None:
				if isIn(point, sq):
					return True

			point = intersection(bl, br, pointA, pointB)
			if point is not None:
				if isIn(point, sq):
					return True

			point = intersection(tl, bl, pointA, pointB)
			if point is not None:
				if isIn(point, sq):
					return True

			point = intersection(tr, br, pointA, pointB)
			if point is not None:
				if isIn(point, sq):
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


if __name__=="__main__":
	
	resolution = 0.25
	space_w = (abs(space[0][0]) + abs(space[1][0]))
	space_h = (abs(space[0][1]) + abs(space[1][1]))
	#Count of spaces
	width_c = space_w/resolution
	height_c = space_h/resolution
	#Size of spaces
	width = space_w/width_c
	height = space_h/height_c

	x_coords = np.linspace(space[0][0], space[1][0], width_c, endpoint = False)
	y_coords = np.linspace(space[0][1], space[1][1], height_c, endpoint = False)

	#Points on the path in the space
	points = [(-3.5,-1.1),(-2.3,0.0),(-1.2,0.2),(0.0,0.2),(3.0,0.2),(3.5,1.0)]

	#Assign the basic path
	decomp = []
	for y in y_coords:
		for x in x_coords:
			sq = grid_sq([x, y], [x + width, y - height], math.pi * (random.random() - 0.5))

			# if isIn((0,0), sq):
			# 	sq.assign(0)
			for pointIdx in range(len(points)-1):
				if isIn(points[pointIdx], sq):
					#Heading of each point is towards next point
					p2 = points[pointIdx]
					p1 = points[pointIdx+1]
					sq.assign(-math.atan2(p2[1]-p1[1], p1[0]-p2[0]))
				elif isBetween(points[pointIdx], points[pointIdx+1], sq):
					#Heading is from square center to next point
					p1 = points[pointIdx+1]
					p2 = (sq.tl[0] + sq.width/2, sq.tl[1] - sq.height/2)
					sq.assign(-math.atan2(p2[1]-p1[1], p1[0]-p2[0]))

			#Random headings to test rendering
			decomp.append(sq)

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
			for neighbor in neighbors(decomp, len(x_coords), len(y_coords), index):
				if neighbor.isAssigned:
					count += 1
					avg_heading += neighbor.heading
			if count > 0:
				#Float to avoid integer math data loss
				avg_heading = avg_heading/float(count)
				newsq.assign(avg_heading)
		decomp_2.append(newsq)

	pg_dbg(space, decomp_2, points)

	#Debugging intersection
	
	# sq1 = grid_sq([2,3], [3,2], 0)
	# sq2 = grid_sq([2,2], [3,1], 0)

	# import pdb; pdb.set_trace()

	# intersect = isBetween([1,2.5], [4,2.5], sq1) #should be true
	