#!/usr/bin/python

# Calibration script for trying to figure out how twist messages map to actual velocities of robots.
# Basic behavior is to gradually bump up the twist velocity while bagging all the tag detections. 
# Then the commanded rotational and linear velocities can be retreived from the twists, and the
# actual velocities can be retreived from the displacements measured in the bag file

import rospy
from geometry_msgs.msg import PointStamped, Twist
from apriltags_ros.msg import *
import pygame
from pygame.locals import *

def main():
	# Initialise screen
	pygame.init()
	screen = pygame.display.set_mode((300, 300))
	pygame.display.set_caption('Cheesy Calibration UI')

	# Fill background
	background = pygame.Surface(screen.get_size())
	background = background.convert()
	background.fill((250, 250, 250))

	# Display some text
	font = pygame.font.Font(None, 36)
	text = font.render("up/down l/r", 1, (10, 10, 10))
	textpos = text.get_rect()
	textpos.centerx = background.get_rect().centerx
	background.blit(text, textpos)

	# Blit everything to the screen
	screen.blit(background, (0, 0))
	pygame.display.flip()

	#Set up some initial conditions
	send_twist = False
	linear = 0.0
	rotational = 0.0
	increment = 0.02

	#Intialize ros stuff
	rospy.init_node('cal_ui', anonymous=True)
	pub = rospy.Publisher('ui_twist', Twist, queue_size=0)
	
	#Create a stop twist and send that
	t = Twist()
	#only two params are used for robots on a table
	t.linear.x = linear
	t.angular.z = rotational
	#The rest are not used
	t.linear.y = t.linear.z = 0
	t.angular.x = t.angular.y = 0
	pub.publish(t)

	# Event loop
	while 1:
		for event in pygame.event.get():
			if event.type == QUIT:
				return
			if event.type == pygame.KEYDOWN:
				send_twist = True
				if event.key == pygame.K_UP:
					linear += increment
				elif event.key == pygame.K_DOWN:
					linear -= increment
				elif event.key == pygame.K_LEFT:
					rotational += increment
				elif event.key == pygame.K_RIGHT:
					rotational -= increment
				else:
					#Any other key is stop
					linear = rotational = 0.0

		if send_twist:
			t = Twist()
			#only two params are used for robots on a table
			t.linear.x = linear
			t.angular.z = rotational
			#The rest are not used
			t.linear.y = t.linear.z = 0
			t.angular.x = t.angular.y = 0
			pub.publish(t)

			#Don't resend until another key is pressed
			send_twist = False

			background = pygame.Surface(screen.get_size())
			background = background.convert()
			background.fill((250, 250, 250))
			text = font.render("r: {} l: {}".format(rotational, linear), 1, (10, 10, 10))
			textpos = text.get_rect()
			textpos.centerx = background.get_rect().centerx
			background.blit(text, textpos)

		screen.blit(background, (0, 0))
		pygame.display.flip()


if __name__ == '__main__': main()
