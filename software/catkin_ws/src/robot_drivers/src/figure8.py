#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist, PointStamped
from apriltags_ros.msg import *
import random
import math
from tf import transformations as transf

# The file name isn't great, this script just tries to make the robot drive in a square. 
# Turning in the other direction will be implemented once a robot actually manages a square. 
# Turning in both directions is good because it will highlight assymetry in the robot's drive ability. 

def dot(a, b):
	assert len(a) == len(b)
	d = 0
	for i, j in zip(a, b):
		d += i*j
	return d

class Point_Converter():
	def __init__(self, robot = 8):
		
		self.robot_id = robot

		self.currentTime = None
		self.currentHeading = 0.0
		self.currentX = 0.0
		self.currentY = 0.0
		self.currentZ = 0.0
		
		self.prevTime = None
		self.prevHeading = 0.0
		self.prevX = 0.0
		self.prevY = 0.0
		self.prevZ = 0.0

		all_tags = rospy.get_param('/apriltag_detector/tag_descriptions')
		self.tagsize = 0.051 #default, but we try to update it
		for tag in all_tags:
			if tag['id'] == self.robot_id:
				self.tagsize = tag['size']

		self.dist = 0
		self.vel = 0
		self.travelIsValid = False

	def update_tags(self, tags_msg):
		#Get the location of this robot from the tag
		tag = None
		try:
			tag = [x for x in tags_msg.detections if x.id == self.robot_id][0]
		except IndexError as ie:
			#This is caused by there not being a detection of the tag in this set of 
			#tag detections
			rospy.logwarn("Didn't see tag {0} in this frame".format(self.robot_id))
			return 

		#If we already have tags, calculate the distance moved and velocity moved
		if self.currentTime is None:
			self.currentTime = tag.pose.header.stamp
			self.currentX = tag.pose.pose.position.x 
			self.currentY = tag.pose.pose.position.y 
			self.currentZ = tag.pose.pose.position.z
			self.startX = self.currentX
			self.startY = self.currentY

			#Update the robot's heading.
			#The roll direction is what I'd call yaw.
			#RPY are ambiguious, nothing to be done for it.
			w = tag.pose.pose.orientation.w
			x = tag.pose.pose.orientation.x
			y = tag.pose.pose.orientation.y
			z = tag.pose.pose.orientation.z
			(roll, pitch, yaw) = transf.euler_from_quaternion([w, x, y, z]) 
			self.currentHeading = roll
			self.startHeading = self.currentHeading

		else:
			#Roll over the current and previous positions
			self.prevTime = self.currentTime
			self.prevX = self.currentX
			self.prevY = self.currentY
			self.prevZ = self.currentZ
			self.prevHeading = self.currentHeading
			
			#Update from the tag
			self.currentTime = tag.pose.header.stamp
			self.currentX = tag.pose.pose.position.x 
			self.currentY = tag.pose.pose.position.y 
			self.currentZ = tag.pose.pose.position.z

			w = tag.pose.pose.orientation.w
			x = tag.pose.pose.orientation.x
			y = tag.pose.pose.orientation.y
			z = tag.pose.pose.orientation.z
			(roll, pitch, yaw) = transf.euler_from_quaternion([w, x, y, z]) 
			
			self.currentHeading = roll
			
			#Now we have two poses, so we can measure travel between them
			self.travelIsValid = True

			#Calculate the distance traveled
			self.dist = math.sqrt(sum([pow(x - y, 2) for x,y in [(self.prevX, self.currentX),(self.prevY, self.currentY)]]))
			#Travel is less than noise floor 
			if self.dist < 0.01:
				self.dist = 0.0
			#Distance is in meters, this is in m/sec
			self.lin_vel = self.dist/((self.currentTime - self.prevTime).to_sec())
			

			#Calculate rotational vel in rads/sec
			#This might be prone to oscillation very near +/-p
			self.rot_dist = abs(self.currentHeading - self.prevHeading)
			#Check if change is less than noise floor 
			if self.rot_dist < 0.005: 
				self.rot_dist = 0.0 
			self.rot_vel = self.rot_dist/((self.currentTime - self.prevTime).to_sec())

	def get_travel(self):
		if not self.travelIsValid:
			rospy.logerr("Requested travel but it's not valid")
			return None
		else:
			#Get the distance from start to here
			d = math.sqrt(sum([pow(x - y, 2) for x,y in [(self.startX, self.currentX),(self.startY, self.currentY)]]))
			return d

	def clear_travel(self):
		#Reset so travel is measured from where we are now
		self.startX = self.currentX
		self.startY = self.currentY
		self.startHeading = self.currentHeading
		#Invalidate travel until a new report comes in
		self.travelIsValid = False

	def isMoving(self):
		if self.travelIsValid:
			return self.lin_vel > 0.0
		else:
			return False

	def isTurning(self):
		if self.travelIsValid:
			return self.rot_vel > 0.0
		else:
			return False
if __name__ == '__main__':
	rospy.init_node('point_driver', anonymous=True)

	robot_id = rospy.get_param("/driver/robot_id")
	pc = Point_Converter(robot=robot_id)

	loc_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, pc.update_tags)

	twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

	lin_vel = 0.0
	rot_vel = 0.0
	lin_vel_inc = 0.02 #m/sec
	rot_vel_inc = 0.01 #rad/sec
	side_len = 0.25 #m, side of the "8"

	#publish messages every 20th of a second
	pub_rate = rospy.Rate(20)
	
	#Check distance 50 times a second
	move_rate = rospy.Rate(50)

	while not rospy.is_shutdown():
		#Reset distance traveled
		pc.clear_travel()

		#Try to start the robot moving
		while not pc.isMoving():
			#increment the linear velocity
			lin_vel += lin_vel_inc

			#Create a twist message and send it 
			sTwist = Twist()
			#only two params are used for robots on a table
			sTwist.linear.x = lin_vel
			sTwist.angular.z = 0
			#The rest are not used
			sTwist.linear.y = sTwist.linear.z = 0
			sTwist.angular.x = sTwist.angular.y = 0
		
			#Send it
			twist_pub.publish(sTwist)

			#wait for a little bit for it to take effect
			pub_rate.sleep()

		#Now the robot is moving, wait until it has gone a fixed distance
		while pc.get_travel() < side_len:
			#Delay a little to let it move
			move_rate.sleep()

		while pc.isMoving():
			lin_vel = rot_vel = 0.0
			#Send a message that stops all motors
			sTwist = Twist()
			#Set all params to zero
			sTwist.linear.x = sTwist.linear.y = sTwist.linear.z = 0
			sTwist.angular.z = sTwist.angular.x = sTwist.angular.y = 0
					
			#Send it and wait for a little bit for it to take effect
			twist_pub.publish(sTwist)
			pub_rate.sleep()

		# Let's pause and see if noise in rotational velocity drops when we're still
		rospy.sleep(4.0)

		# #Calculate the heading we want to be on if we turn 90 degrees L
		# desired_heading = pc.currentHeading + 90.0 * (math.pi/180.0)
		# #Wrap around 0 radians from one direction
		# #Turning the other way requres different limit check
		# if desired_heading > (2 * math.pi):
		# 	desired_heading = desired_heading - (2*math.pi)

		# while not pc.isTurning():
		# 	#increment the turn speed
		# 	rot_vel += rot_vel_inc

		# 	#Create a twist message and send it 
		# 	sTwist = Twist()
		# 	#only two params are used for robots on a table
		# 	sTwist.linear.x = 0 #We're not driving forwards
		# 	sTwist.angular.z = rot_vel
		# 	#The rest are not used
		# 	sTwist.linear.y = sTwist.linear.z = 0
		# 	sTwist.angular.x = sTwist.angular.y = 0
		
		# 	#Send it
		# 	twist_pub.publish(sTwist)

		# 	#wait for a little bit for it to take effect
		# 	pub_rate.sleep()			

		# #Get the smallest angle, it should be aprox 90 degrees/1.5 rad or so		
		# smallest_angle = math.atan2(math.sin(pc.currentHeading-desired_heading), math.cos(pc.currentHeading-desired_heading))

		# while smallest_angle > 0.05:
		# 	#Watch the smallest angle while we turn
		# 	smallest_angle = math.atan2(math.sin(pc.currentHeading-desired_heading), math.cos(pc.currentHeading-desired_heading))
		# 	move_rate.sleep()

		# #Come to a stop after turning
		# while pc.isTurning():
		# 	#Send a message that stops all motors
		# 	sTwist = Twist()
		# 	#Set all params to zero
		# 	sTwist.linear.x = sTwist.linear.y = sTwist.linear.z = 0
		# 	sTwist.angular.z = sTwist.angular.x = sTwist.angular.y = 0
					
		# 	#Send it and wait for a little bit for it to take effect
		# 	twist_pub.publish(sTwist)
		# 	pub_rate.sleep()

