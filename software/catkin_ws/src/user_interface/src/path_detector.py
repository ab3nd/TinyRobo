#!/usr/bin/python

import rospy
from user_interface.msg import Kivy_Event, Stroke, Gesture
from geometry_msgs.msg import Point
import math
from apriltags_ros.msg import *
import classification_heuristics
import numpy as np

# Receives the locations of the robots and the strokes made by the user, and determines if the 
# resulting configuration could be a box selection. Generally, this means that the user drew a line
# and that the bounding box of the line has robots inside of it. 
# This is actually too permissive of a criterion for box selection, but higher level nodes will
# accept or reject the box selection canidates based on state and previous actions

class PathDetector(object):
	def __init__(self):
		self.currentTags = {}
		self.gesturePub = rospy.Publisher("gestures", Gesture, queue_size=10)

	def min_bounding_oval(self, points, tolerance=0.01):
		#Create a numpy 2xN matrix from a list of 2D points
		P = np.matrix(points).T

		# Pseudocode from https:#stackoverflow.com/questions/1768197/bounding-ellipse
		# Pythonificiation by me
		# Input: A 2xN matrix P storing N 2D points 
		#        and tolerance = tolerance for error.
		# Output: The equation of the ellipse in the matrix form, 
		#         i.e. a 2x2 matrix A and a 2x1 vector C representing 
		#         the center of the ellipse.

		# Dimension of the points
		d = 2;   
		# Number of points
		N = len(points);  

		# Add a row of 1s to the 2xN matrix P - so Q is 3xN now.
		Q = np.vstack([P,np.ones((1,N))]) 

		# Initialize
		count = 1;
		err = 1;
		#u is an Nx1 vector where each element is 1/N
		u = (1.0/N) * np.ones((N,1))

		# Khachiyan Algorithm
		while err > tolerance:
			# Matrix multiplication: 
			X = Q * np.diagflat(u) * Q.T

			M = np.diagonal(Q.T * X.I * Q)

			# Find the value and location of the maximum element in the vector M
			maximum = M.max()
			j = np.argmax(M);

			# Calculate the step size for the ascent
			step_size = (maximum - d -1)/((d+1)*(maximum-1));

			# Calculate the new_u:
			# Take the vector u, and multiply all the elements in it by (1-step_size)
			new_u = (1 - step_size) * u ;

			# Increment the jth element of new_u by step_size
			new_u[j] = new_u[j] + step_size;

			# Store the error by taking finding the square root of the SSD 
			# between new_u and u
			err = math.sqrt(((new_u - u)**2).sum());
			#print err

			# Increment count and replace u
			count = count + 1;
			u = new_u;

		# Put the elements of the vector u into the diagonal of a matrix
		# U with the rest of the elements as 0
		U = np.diagflat(u);

		# Compute the A-matrix
		A = (1.0/d) * (P * U * P.T - (P * u)*(P*u).T ).I

		# And the center,
		c = P * u

		return [A, c]

	def update_robot_points(self, msg):
		#Just saves the detections
		self.currentTags = {}
		for ii in range(len(msg.detections)):
			self.currentTags[int(msg.detections[ii].id)] = msg.detections[ii]

	def check_stroke(self, msg):
		#circles and lines are checked by the lasso select detector and box selection detector
		if classification_heuristics.is_arc(msg):
			#Arcs are always treated as paths
			evt = Gesture()
			evt.eventName = "path"
			evt.stamp = rospy.Time.now()
			evt.isButton = False 
			evt.strokes = [msg]
			self.gesturePub.publish(evt)
		elif classification_heuristics.is_circle(msg):
			#Clear out selected tags and see if the new gesture selects any of them
			selected_tags = []
			isDrag = False
			#Set up to watch for a tag close to the start of the circle
			closest_robot = None
			closest_distance = float("inf")

			if len(self.currentTags) > 0:
				#Get the points of the lasso as a list
				points = [(event.point.x, event.point.y) for event in msg.events]

				#See which points fall inside the list (which represents a polygon)
				for tag in self.currentTags.values():
					tag_x = tag.tagCenterPx.x
					tag_y = tag.tagCenterPx.y

					# From http://www.ariel.com.au/a/python-point-int-poly.html
					n = len(points)
					inside =False

					p1x,p1y = points[0]
					for i in range(n+1):
						p2x,p2y = points[i % n]
						if tag_y > min(p1y,p2y):
							if tag_y <= max(p1y,p2y):
								if tag_x <= max(p1x,p2x):
									if p1y != p2y:
										xinters = (tag_y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
									if p1x == p2x or tag_x <= xinters:
										inside = not inside
						p1x,p1y = p2x,p2y

					if inside:
						selected_tags.append(tag.id)

					#Keep track of the closest tag to the start of the original circle
					d = math.sqrt(math.pow(tag_x - msg.events[0].point.x, 2) + math.pow(tag_y - msg.events[0].point.y, 2))
					if d < closest_distance:
						closest_robot = tag.id
						closest_distance = d
					
					#Approx width of a finger on my screen
					if closest_distance < 80: 
						isDrag = True
		
				evt = Gesture()	
				evt.stamp = rospy.Time.now()
				evt.isButton = False 
				evt.strokes = [msg]

				if len(selected_tags) > 0 and not isDrag:
					#This is possibly a lasso select, pack it up and publish it
					evt.eventName = "lasso_select"
					evt.robots = selected_tags
				elif isDrag:
					#It's a line over robots, but it's a drag, so it isn't a lasso select
					evt.eventName = "drag_robot"
					evt.robots = [closest_robot]
				else:
					#Is neither a single robot drag nor a lasso select, so it's a path
					evt.eventName = "path"
					
				self.gesturePub.publish(evt)


		elif classification_heuristics.is_line(msg):
			#Clear out selected tags and see if the new gesture selects any of them
			selected_tags = []
			isDrag = False
			closest_robot = None
			closest_distance = float('inf')

			if len(self.currentTags) > 0:
				#Get the bounding box of the stroke
				xs = [event.point.x for event in msg.events]
				minX = min(xs)
				maxX = max(xs)
				ys = [event.point.y for event in msg.events]
				minY = min(ys)
				maxY = max(ys)

				#Get all the tags that have at least one corner in the box
				for tag in self.currentTags.values():
					# Rescale from pixels in camera view to pixels in UI view (cropped, embiggened image)
					tag_x = tag.tagCenterPx.x #* 1.640625
					tag_y = tag.tagCenterPx.y #(tag.tagCenterPx.y - 120) * 1.640625

					if (minX < tag_x < maxX) and (minY < tag_y < maxY):
						selected_tags.append(tag.id)


					#Check that this robot isn't very near the start of the line. If it is,
					#this should be treated as a drag move of the robot, not a selection of other robots. 
					d = math.sqrt(math.pow(tag_x - msg.events[0].point.x, 2) + math.pow(tag_y - msg.events[0].point.y, 2))
					if d < closest_distance:
						closest_robot = tag.id
						closest_distance = d

				#Approx width of a finger on my screen
				if closest_distance < 80: 
					isDrag = True
		
				evt = Gesture()	
				evt.stamp = rospy.Time.now()
				evt.isButton = False 
				evt.strokes = [msg]

				if len(selected_tags) > 0 and not isDrag:
					#This is possibly a box select, pack it up and publish it
					evt.eventName = "box_select"
					evt.robots = selected_tags
				elif isDrag:
					#It's a line over robots, but it's a drag, so it isn't a box select
					evt.eventName = "drag_robot"
					evt.robots = [closest_robot]
				else:
					#Is neither a single robot drag nor a box select, so it's a path
					evt.eventName = "path"
					
				self.gesturePub.publish(evt)



rospy.init_node('path_detect')
pd = PathDetector()
strokeSub = rospy.Subscriber("/strokes", Stroke, pd.check_stroke)
tagSub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, pd.update_robot_points)

rospy.spin()
