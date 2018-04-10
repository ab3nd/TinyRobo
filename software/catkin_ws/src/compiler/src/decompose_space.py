#!/usr/bin/python

#Decomposes a space into voroni cells, and figures out the vector for each 
#cell to drive robots in that cell along a specific path

#For visualization
import pygame
import sys
import numpy as np
import random
import math

#Points on the path in the space
points = [(-3.5,-1.1),(-2.3,0.0),(-1.2,0.2),(0.0,0.2),(3.0,0.2),(3.5,1.0)]

#Bottom left and upper right corners of space
space = [(-4,-2), (4,2)]

def to_pygame(r, width = 640, height = 480, ppm = 70):
	

	#Get the width and height of the rectangle in pixels
	rect_w = (abs(r[0][0]) + abs(r[1][0])) * ppm
	rect_h = (abs(r[0][1]) + abs(r[1][1])) * ppm

	#Get the top left corner in pixels
	tl_x = r[0][0] * ppm
	tl_y = -r[1][1] * ppm # negate because increasing y moves down the screen, not up

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

def pg_dbg(space, decomp):
	pygame.init()
	size = width,height = 640,480
	screen = pygame.display.set_mode(size)
	screen.fill((255,255,255)) #Black screen

	#Draw the space
	green = (0,128,0)
	#Pygame rects are left, top, width, height
	pygame.draw.rect(screen, green, to_pygame(space) ,0)

	#Draw a point for each decomp coordinate
	for sq in decomp:
		#ppt = point_to_pygame(sq.tl)
		#pygame.draw.rect(screen, (0,200,0), pygame.Rect(ppt[0], ppt[1], 2, 2), 0)

		#Get the center point
		center = (sq.tl[0] + sq.width/2, sq.tl[1] - sq.height/2)
		pg_center = point_to_pygame(center)

		#Get a point that's half the width away on the heading
		hx = (sq.tl[0] + sq.width/2) + (sq.width/2 * math.cos(sq.heading))
		hy = (sq.tl[1] - sq.height/2) + (sq.width/2 * math.sin(sq.heading))
		pg_vector = point_to_pygame([hx, hy])

		print hx, hy
		pygame.draw.line(screen, (0,0,140), (pg_center[0], pg_center[1]), (pg_vector[0], pg_vector[1]), 2)
		#print pg_center
		#Draw a point in the middle of each grid square
		#pygame.draw.rect(screen, (0,0,140), pygame.Rect(pg_center[0], pg_center[1], 2, 2), 0) 
	#center = pygame.draw.rect(screen, (150, 0, 0), pygame.Rect(width/2 - 5, height/2 - 5, 10,10),0)
	pygame.display.flip()

	while True:
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				pygame.quit(); sys.exit();


class grid_sq(object):
	def __init__(self, tl = [0,0], br = [0,0], heading = 0):
		self.tl = tl
		self.br = br
		self.width = abs(tl[0] - br[0])
		self.height = abs(tl[1] - br[1])
		self.heading = heading

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

	x_coords = np.linspace(space[0][0], space[1][0], width_c)
	y_coords = np.linspace(space[0][1], space[1][1], height_c)

	decomp = []
	for x in x_coords[:-1]:
		for y in y_coords[1:]:
			#Random headings to test rendering
			decomp.append(grid_sq([x, y], [x + width, y + height], math.pi * (random.random() - 0.5)))

	pg_dbg(space, decomp)