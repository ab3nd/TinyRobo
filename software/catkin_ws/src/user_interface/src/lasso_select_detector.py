#!/usr/bin/python

import rospy
from user_interface.msg import Kivy_Event, Stroke
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


class LassoSelectDetector(object):
	def __init__(self):
		self.currentTags = {}

	def update_robot_points(self, msg):
		#Just saves the detections
		self.currentTags = {}
		for ii in range(len(msg.detections)):
			self.currentTags[int(msg.detections[ii].id)] = msg.detections[ii]

	def min_bounding_oval(self, points, tolerance=0.01):
		#Create a numpy 2xN matrix from a list of 2D points
		P = np.matrix(points).T

		# Pseudocode from https:#stackoverflow.com/questions/1768197/bounding-ellipse
		# Pythonitficiation by me
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

	def check_stroke(self, msg):
		#Only closed shapes can be lasso selects
		if classification_heuristics.is_circle(msg):		
			#Clear out selected tags and see if the new gesture selects any of them
			selected_tags = []
			if len(self.currentTags) > 0:
				#Get the minimum bounding oval of the lasso gesture
				points = [(event.point.x, event.point.y) for event in msg.events]
				try:
					A, c = self.min_bounding_oval(points)
					#A lot of this test is based on https://math.stackexchange.com/questions/76457/check-if-a-point-is-within-an-ellipse
					# but since the ellipse isn't axis-aligned, the E matrix isn't [[1 0]
					#                                                               [0 1]]
					# and the algorthm for the best fit ellipse above includes the unit vectors for the axes already.
					# I think at an abstract level, what this ends up doing is calculating a matrix representing a
					# conversion from the ellipse to a unit circle at (0,0), and then subtracting the ellipse center
					# moves the canidate point to the (0,0) of the transformed space. I could be wrong about that, though. 

					#Convert A to a whitening matrix (W = A**-1/2)
					w, v = np.linalg.eig(A)
					D = np.diagflat(w)
					W = v * np.sqrt(D) * v.I

					#Get all the tags that have their center in the ellipse
					for tag in self.currentTags.values():
						# Rescale from pixels in camera view to pixels in UI view (cropped, embiggened image)
						tag_x = tag.tagCenterPx.x * 1.640625
						tag_y = (tag.tagCenterPx.y - 120) * 1.640625

						#Subtract the center of the ellipse and use the whitening matrix
						p = np.matrix([[tag_x],[tag_y]])
						p_center = p - c
						p_white = W * p_center

						#Check if the whitened point is in the ellipse
						if np.linalg.norm(p_white) <= 1:
							selected_tags.append(tag.id)
				except np.linalg.LinAlgError:
					#Singular matrix usually causes this, the stroke was probably a tap
					return 			
				if len(selected_tags) > 0:
					#This is possibly a box select, pack it up and publish it
					rospy.loginfo("{0} selects {1}".format(msg.uid, selected_tags))
		
rospy.init_node('lasso_select_detect')
lsd = LassoSelectDetector()
strokeSub = rospy.Subscriber("/strokes", Stroke, lsd.check_stroke)
tagSub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, lsd.update_robot_points)

rospy.spin()
