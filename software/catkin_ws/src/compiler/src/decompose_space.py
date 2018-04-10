#!/usr/bin/python

#Decomposes a space into voroni cells, and figures out the vector for each 
#cell to drive robots in that cell along a specific path

#For visualization
import pygame
import sys

#Points on the path in the space
points = [(-3.5,-1.1),(-2.3,0.0),(-1.2,0.2),(0.0,0.2),(3.0,0.2),(3.5,1.0)]

#Bottom left and upper right corners of space
space = [(-4,-2), (4,2)]

def to_pygame(r, width = 640, height = 480):
	
	#Conversion of pixels to meters
	ppm = 70

	#Get the width and height of the rectangle in pixels
	rect_w = (abs(r[0][0]) + abs(r[1][0])) * ppm
	rect_h = (abs(r[0][1]) + abs(r[1][1])) * ppm

	print rect_w, rect_h

	#Get the top left corner in pixels
	tl_x = r[0][0] * ppm
	tl_y = -r[1][1] * ppm # negate because increasing y moves down the screen, not up

	print tl_x, tl_y

	#Convert to offset from center
	tl_x += width/2
	tl_y += height/2
	
	print tl_x, tl_y

	return pygame.Rect(tl_x, tl_y, rect_w, rect_h)

def pg_dbg(space):
	pygame.init()
	size = width,height = 640,480
	screen = pygame.display.set_mode(size)
	screen.fill((0,0,0)) #Black screen

	#Draw the space
	green = (0,128,0)
	#Pygame rects are left, top, width, height
	spc = pygame.draw.rect(screen, green, to_pygame(space) ,0)
	#center = pygame.draw.rect(screen, (150, 0, 0), pygame.Rect(width/2 - 5, height/2 - 5, 10,10),0)
	pygame.display.flip()

	while True:
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				pygame.quit(); sys.exit();

if __name__=="__main__":
	pg_dbg(space)
