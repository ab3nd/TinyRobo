#!/usr/bin/python

import random
import json

from time import sleep

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from network_service.msg import *
import math
from tf import transformations as transf
from argos_bridge.msg import Proximity
from argos_bridge.msg import ProximityList


class ProgramLoader(object):
	def __init__(self):
		# Example program, will get replaced
		# Default is to not go anywhere
		self.program = [("True", "self.stop()", 1.0)]

	#This is how programs get deployed to this runner instance
	#Currently just a string containing a set of GCPR tuples
	#This isn't anything like secure, since eval is getting used. 
	#There also may be thread safety concerns
	def replaceProgram(self, msg):
		rospy.logwarn("Got new program")
		self.program = json.loads(msg.data)
		
	def getProgram(self):
		#rospy.loginfo_throttle(10, "getProgram called")
		#rospy.loginfo_throttle(10, json.dumps(self.program))
		return self.program


class GCPR_driver(object):
	def __init__(self):
		self.programLoader = ProgramLoader()
		self.laser_readings = []
		self.max_range = 0.6 #TODO TOTALLY ARBITARY THRESHOLD FIXME

		self.twistPub = rospy.Publisher('cmd_vel', Twist, queue_size=0)
		self.laser_msg = None

		#For ARGoS sensors
		self.proxReadings = []
		self.heading = 0.0
		self.traveled_x = 0.0
		self.traveled_y = 0.0
		self.lastPosition = None

		#for Bug algo
		self.closest_visited_point = None
		self.target_point = None
		self.hit_point = None

		#For GCPR program counter, default to 0
		self.prog_ctr = 0

		self.desired_heading = 0
		self.current_heading = 0
		#For collision avoidance, calculated from sensors
		self.avoid_heading = 0

		#For debugging
		self.ns = rospy.get_namespace()

	def update_laser(self, laserMsg):
		self.laser_readings = laserMsg.ranges
		self.laser_msg = laserMsg

	def update_pose(self, poseMsg):
		if self.lastPosition is not None:
			self.traveled_x += abs(self.lastPosition.position.x - poseMsg.position.x)
			self.traveled_y += abs(self.lastPosition.position.y - poseMsg.position.y)
			w = poseMsg.orientation.w
			x = poseMsg.orientation.x
			y = poseMsg.orientation.y
			z = poseMsg.orientation.z

			#Update the robot's heading.
			(roll, pitch, yaw) = transf.euler_from_quaternion([w, x, y, z]) 
			self.current_heading = roll

		#For future distance calculation
		self.lastPosition = poseMsg

		#Debugging
		#print poseMsg

	def update_prox(self, proxMsg):
		#They arrive sorted by angle
		#The array is wrapped around the robot CCW, with 0 being just left of the front 
		#and 23 being just right of the front. 
		self.proxReadings = proxMsg.proximities

	def is_in(self, tl, br):
		if self.lastPosition is not None:
			x = self.lastPosition.position.x
			y = self.lastPosition.position.y
			if (br[0] >= x and tl[0] <= x ) and (br[1] <= y and tl[1] >= y):
				return True
		return False
			
	def recv_msg(self, msg):
		rospy.logwarn("Got a message {0}".format(msg.data))
		pass

	def replace_program(self, msg):
		self.programLoader.replaceProgram(msg)

	def run_gcpr(self):
		#Nothing to do yet
		todo_list = []

		#Get all the rules and see if any of them have satisfied guards
		try:
			
			for rule in self.programLoader.getProgram():
				if eval(rule[0]):
					#Check the rate
					if random.random() < rule[2]:
						#Add to the list of things to do
						todo_list.append(rule[1])
			
			random.shuffle(todo_list)
		except IndexError as e:
			print e
			print "---caused by---"
			print rule

		#This needs some smarter method to unify the drive commands than just FIFO
		#Or does it? FIFO fast enough, plus a short-cycle time would be a sort of
		#stochastic PWM effect
		for item in todo_list:
			#TODO, add "self." to calls in the program...
			#https://stackoverflow.com/questions/1911281/how-do-i-get-list-of-methods-in-a-python-class
			#might be of help for figuring out what substrings are callable
			eval(item)

		#rospy.loginfo_throttle(3, "{} heading {}, distanceX {}, distanceY {}".format(self.ns, self.current_heading, self.traveled_x, self.traveled_y))

	#Functions for handling heading
	def set_desired_heading(self, value):
		#rospy.logwarn("Set heading {}".format(value))
		self.desired_heading = value

	#Within threshold of heading
	def on_heading(self, heading = None):
		if heading is None:
			heading = self.desired_heading
		smallest_angle = math.atan2(math.sin(self.current_heading-heading), math.cos(self.current_heading-heading))
	 	if abs(smallest_angle) < 0.05:
	 		return True	
	 	return False

 	def turn_heading(self, speed, heading = None):
 		if heading is None:
			heading = self.desired_heading
		
 		if not self.on_heading(heading):
	 		#Decide turn direction
	 		smallest_angle = math.atan2(math.sin(self.current_heading-heading), math.cos(self.current_heading-heading))
	 		
	 		if smallest_angle > 0:
	 			self.move_turn(abs(speed))
	 		else:
	 			self.move_turn(-abs(speed))

 	def turn_heading_arc(self, rot_speed, fwd_speed, heading = None):
 		if heading is None:
			heading = self.desired_heading
		
 		if not self.on_heading(heading):
	 		#Decide turn direction
	 		smallest_angle = math.atan2(math.sin(self.current_heading-heading), math.cos(self.current_heading-heading))
	 		
	 		if smallest_angle > 0:
	 			self.move_arc(abs(rot_speed), fwd_speed)
	 		else:
	 			self.move_arc(-abs(rot_speed), fwd_speed)

		
 	def reset_travel(self):
 		self.traveled_y = self.traveled_x = 0

	#Functions for handling a program counter for sequential state changes
	def set_pc(self,value):
		rospy.loginfo_throttle(3, "{} set pc to {}".format(self.ns, value))
		self.prog_ctr = value
		#TODO this is a HACK
		self.reset_travel()

	def pc_is(self, value):
		return self.prog_ctr == value

	# Basic action of moving along an arc
	def move_arc(self, rot_speed, trans_speed):
		rTwist = Twist()
		#only two params are used for robots on a table
		rTwist.linear.x = trans_speed
		rTwist.angular.z = rot_speed
		#The rest are not used
		rTwist.linear.y = rTwist.linear.z = 0
		rTwist.angular.x = rTwist.angular.y = 0

		#publish it and wait
		self.twistPub.publish(rTwist)
		
	# Moving straight is moving in an arc with no rotational speed
	def move_fwd(self, speed):
		#rospy.loginfo_throttle(3, "{} Moving with speed {}".format(self.ns, speed))
		self.move_arc(0, speed)

	# Turning is moving in an arc with no translational speed
	def move_turn(self, speed):
		#rospy.loginfo_throttle(3, "{} Turning with speed {}".format(self.ns, speed))
		self.move_arc(speed, 0)

	# Stopping is moving with no velocity.
	def stop(self, success=None):
		if success is not None:
			if success:
				rospy.loginfo_throttle(5, "{} arrived".format(self.ns))
			else:
				rospy.loginfo_throttle(5, "{} cannot arrive".format(self.ns))
		#rospy.loginfo_throttle(10, "{} Stopping".format(self.ns))
		self.move_arc(0,0)

	# This is for doing printouts from inside the GCPR code and having it get out to ROS
	def dbg_print(self, message):
		rospy.logwarn(message)
		
	# Define the guards that can be sensed. Guards are boolean, so they should return either a boolean
	# or something that python is going to treat as a boolean. I've decided to name the guards as 
	# questions so that it reads better

	def is_near_anything(self):
		if sum([x.value for x in self.proxReadings]) > 0:
			return True
		return False

	def is_near_left_f_quarter(self):
		return self.check_readings(-(math.pi/2), 0.0 )

	def is_near_right_f_quarter(self):
		return self.check_readings(0.0, (math.pi/2))
	
	def is_near_left_r_quarter(self):
		return self.check_readings(-(math.pi/2), -math.pi)

	def is_near_right_r_quarter(self):
		return self.check_readings((math.pi/2), math.pi)
	
	def is_near_front(self):
		return self.check_readings(-(math.pi/2), (math.pi/2))

	def check_readings(self, start, end):
		for reading in self.proxReadings:
			if reading.angle >= start and reading.angle <= end:
				if reading.value > 0:
					return True
		return False


	#Bounds checks on x and y position for areas outside of defined paths
	def x_gt(self, otherX):
		if self.lastPosition is not None:
			x = self.lastPosition.position.x
			if x > otherX:
				return True
		return False

	def y_gt(self, otherY):
		if self.lastPosition is not None:
			y = self.lastPosition.position.y
			if y > otherY:
				return True
		return False

	def x_lt(self, otherX):
		if self.lastPosition is not None:
			x = self.lastPosition.position.x
			if x < otherX:
				return True
		return False

	def y_lt(self, otherY):
		if self.lastPosition is not None:
			y = self.lastPosition.position.y
			if y < otherY:
				return True
		return False
		
	def x_between(self, minX, maxX):
		if self.lastPosition is not None:
			x = self.lastPosition.position.x
			if minX <= x <= maxX:
				#rospy.logwarn("x {} is between {} and {}".format(x, minX, maxX))
				return True
		return False
	
	def y_between(self, minY, maxY):
		if self.lastPosition is not None:
			y = self.lastPosition.position.y
			if minY <= y <= maxY:
				#rospy.logwarn("y {} is between {} and {}".format(y, minY, maxY))
				return True
		return False
		
	#=============== Sensors for bug algorithims ====================
	#Return the closest point to the goal in the tangent graph
	def closest_tangent_point(self):
		min_d = float('inf')
		closest = None
		#For each non-zero point in the sensor reading
		for index, reading in enumerate(self.proxReadings):
			isTangent = False
			if reading.value != 0:
				#This reading detects something, check if either of the readings around it are 0
				#If it is, this is a tangent
				if index == 0:
					if self.proxReadings[1] == 0 or self.proxReadings[23] == 0:
						isTangent = True
				elif index == 23:
					if self.proxReadings[0] == 0 or self.proxReadings[22] == 0:
						isTangent = True
				else:
					if self.proxReadings[index-1] == 0 or self.proxReadings[index+1] == 0:
						isTangent = True
			if isTangent:
				#Argos prox sensors have a 10cm range, convert this angle and range to a point
				#10 cm is 1 dm is 0.1 m
				x = 0.10 * math.cos(reading.angle)
				y = 0.10 * math.sin(reading.angle)
				d = self.distance((x,y), (self.target_point[0], self.target_point[1]))
				
				#If it is closer to the target point than previously seen, save it
				if d < min_d:
					min_d = d
					closest = (x,y)
		return closest

	#Return the closest point to the goal in free space
	def closest_free_point(self, goal):
		min_d = float('inf')
		closest = None
		x2 = self.lastPosition.position.x
		y2 = self.lastPosition.position.y
		for index, reading in enumerate(self.proxReadings):
			#For each zero point in the sensor readings
			if reading.value == 0:
				isEdge = False
				x0 = y0 = 0
				#This reading detects something, check if either of the readings around it are 0
				if index == 0:
					if self.proxReadings[1].value > 0:
						x0 = (self.proxReadings[1].value *  0.10) * math.cos(self.proxReadings[1].angle) + x2
						y0 = (self.proxReadings[1].value *  0.10) * math.sin(self.proxReadings[1].angle) + y2
						isEdge = True
					if self.proxReadings[23].value > 0:
						x0 = (self.proxReadings[23].value *  0.10) * math.cos(self.proxReadings[23].angle) + x2
						y0 = (self.proxReadings[23].value *  0.10) * math.sin(self.proxReadings[23].angle) + y2
						isEdge = True
				elif index == 23:
					if self.proxReadings[0].value > 0:
						x0 = (self.proxReadings[0].value *  0.10) * math.cos(self.proxReadings[0].angle) + x2
						y0 = (self.proxReadings[0].value *  0.10) * math.sin(self.proxReadings[0].angle) + y2
						isEdge = True
					if self.proxReadings[22].value > 0:
						x0 = (self.proxReadings[22].value *  0.10) * math.cos(self.proxReadings[22].angle) + x2
						y0 = (self.proxReadings[22].value *  0.10) * math.sin(self.proxReadings[22].angle) + y2
						isEdge = True						
				else:
					if self.proxReadings[index-1].value > 0:
						x0 = (self.proxReadings[index-1].value *  0.10) * math.cos(self.proxReadings[index-1].angle) + x2
						y0 = (self.proxReadings[index-1].value *  0.10) * math.sin(self.proxReadings[index-1].angle) + y2
						isEdge = True						
					if self.proxReadings[index+1].value > 0:
						x0 = (self.proxReadings[index+1].value *  0.10) * math.cos(self.proxReadings[index+1].angle) + x2
						y0 = (self.proxReadings[index+1].value *  0.10) * math.sin(self.proxReadings[index+1].angle) + y2
						isEdge = True

				#Argos prox sensors have a 10cm range, convert this angle and range to a point
				#10 cm is 1 dm is 0.1 m
				x1 = 0.10 * math.cos(reading.angle) + x2
				y1 = 0.10 * math.sin(reading.angle) + y2
				d = self.distance((x1,y1), (goal[0], goal[1]))

				#Detected object check
				if isEdge:

					dObj = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1)/self.distance((x1,y1),(x2,y2))

					#If the distance from the detected object to the line for the free point is less than
					#the width of the robot, the point isn't clear enough to head for
					#rospy.logwarn("index: {} dObj = {}".format(index, dObj))
					width = 0.01
					if dObj < width:
						continue
						
				#If it is closer to the target point than previously seen, save it
				if d < min_d:
					min_d = d
					closest = (x1,y1)
		#rospy.logwarn("Closest free point ({},{})".format(closest[0], closest[1]))
		return closest

	#Distance between two points
	def distance(self, p1, p2):
		return math.sqrt(math.pow(p1[0] - p2[0] , 2) + math.pow(p1[1] - p2[1], 2))

	#Calculate the heading to a point from the current location
	def get_heading(self, p2):
		if self.lastPosition is None:
			return 0.0 #This is not great, but it beats crashing
		return 2 * math.atan2(p2[0] - self.lastPosition.position.x, p2[1] - self.lastPosition.position.y)

	#True if within a specified distance of a point
	def at(self, p1, threshold = 0.05):
		if self.lastPosition is None:
			return False #It's not a place, we can't possibly be there
		if self.distance((self.lastPosition.position.x, self.lastPosition.position.y), p1) < threshold:
			return True
		return False


rospy.init_node("gcpr_driver", anonymous=True)

#Get the ID of the robot that this instance of the driver is driving
#robot_id = rospy.get_param("/gcpr/robot_id")
#Argos sets these as namespaces, so I may just be able to get the proximity from the 
#namespace, which saves me dealing with the robot ID in the parameters

#Set up an instance of the GCPR driver
gDriver = GCPR_driver()

#Subscribe to laser for this robot
#laser_sub = rospy.Subscriber("/laser_driver_{0}".format(robot_id), LaserScan, gDriver.update_laser)
#Argos proximity sensor
proxSub = rospy.Subscriber("proximity", ProximityList, gDriver.update_prox)

#Subscribe to the position for this robot
#We're not actually using the absolute position, just the distance traveled
#and the heading, as if we had odom and a compass/IMU
poseSub = rospy.Subscriber("position", Pose, gDriver.update_pose)

#Listen for new programs to load
prog_sub = rospy.Subscriber("robot_prog", String, gDriver.replace_program)

#Cleanup code to make sure the robot stops moving when ros is shut down
#Not guaranteed, because the message might not make it out
rospy.on_shutdown(gDriver.stop)	

#TODO figure out what a good rate for the driver to run at is,
# and update at that rate
r = rospy.Rate(100) # in Hz
while not rospy.is_shutdown():
    #Update the GCPR driver
    gDriver.run_gcpr()
    r.sleep()
