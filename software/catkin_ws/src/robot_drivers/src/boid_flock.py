#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from apriltags_ros.msg import *

#Attempt to write a boid flocker for the tiny robos

#Boid flocking has three steering components
#1. Steer to avoid crowding local flockmates
#2. Steer towards the average heading of local flockmates
#3. Steer to move to the average position of local flockmates

class BoidFlocker(object):

	def __init__(self, my_id):
		self.robot_id = my_id
		self.my_tag = None
		self.neighborhood_r = 0.08
		self.my_neighbors = {}
		self.old_neighbors = {}

	def distance(self, p1, p2):
		return float(sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2)))

	def update_tags(self, msg):
		
		#Filter the tags down to my neighborhood
		#First I need to get my own current location
		for tag in msg.detections:
			if tag.id == self.robot_id:
				self.my_tag = tag
				break
		#Now filter all the others based on distance
		self.old_neighbors = self.my_neighbors
		self.my_neighbors = {}
		for tag in msg.detections:
			if tag.id != self.robot_id:
				if self.distance(tag.pose, self.my_tag.pose) < self.neighborhood_r:
					self.my_neighbors[tag.id] = tag

		#Get closest robot (#1 - collision avoidance)
		closest_neighbor = None
		closest_distance = int('Inf')
		for neighbor_id in self.my_neighbors.keys():
			d = self.distance(my_neighbors[neighbor_id].pose, self.my_tag.pose)
			if d < closest_distance:
				closest_neighbor = neighbor_id
				closest_distance = d

		#Get average heading of neighbors (#2 - navigation)
		angle_x = angle_y = 0
		for neighbor_id in self.my_neighbors.keys():
			w = self.my_neighbors[neighbor_id].pose.orientation.w
			x = self.my_neighbors[neighbor_id].pose.orientation.x
			y = self.my_neighbors[neighbor_id].pose.orientation.y
			z = self.my_neighbors[neighbor_id].pose.orientation.z

			#The roll direction is what I'd call yaw.
			#RPY are ambiguious, nothing to be done for it.
			(roll, pitch, yaw) = transf.euler_from_quaternion([w, x, y, z]) 
			
			#Accumulate x, y coordinates for each vector
			angle_x += math.cos(roll)
			angle_y += math.sin(roll)
		avg_heading = math.atan2(angle_y, angle_x)

		#Get average speed of neighbors (#2 - navigation)
		#Original boids work treated heading and velocity as vector with magnitude
		count = 0
		avg_speed = 0
		for neighbor_id in self.my_neighbors.keys():
			if neighbor_id in self.old_neighbors.keys():
				#Get the distance traveled 
				d = self.distance(self.my_neighbors[neighbor_id].pose, self.old_neighbors[neighbor_id].pose)
				t = self.my_neighbors[neighbor_id].pose.header.stamp - self.old_neighbors[neighbor_id].pose.header.stamp
				avg_speed = d/t
				count += 1
		avg_speed = avg_speed/count

		#Get centroid (#3 - cohesion)
		center_x = center_y = 0
		for neighbor_id in self.my_neighbors.keys():
			center_x += self.my_neighbors[neighbor_id].pose.position.x
			center_y += self.my_neighbors[neighbor_id].pose.position.y
		center_x = center_x/len(self.my_neighbors)
		center_y = center_y/len(self.my_neighbors)

		#Figure out motion in that direction
		#This may require limits on maximum speed
		rospy.loginfo("{}: Closest: {} ({}), heading: {}, speed {}, centroid ({}, {})").format(self.my_id, closest_neighbor, closest_distance, avg_heading, avg_speed, center_x, center_y)

if __name__ == "__main__":

	rospy.init_node('distance_oracle_server')
	
	rospy.loginfo("Distance oracle started...")

	my_id = 0
	bf = BoidFlocker(my_id)
	

	#Subscribe to april tag tag detections
	rospy.Subscriber("/tag_detections", AprilTagDetectionArray, bf.update_tags)
	rospy.spin()

