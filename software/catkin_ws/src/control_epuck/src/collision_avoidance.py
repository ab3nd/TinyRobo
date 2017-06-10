#!/usr/bin/python
import rospy
import math
import signal
import sys
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
#Simple collision avoidance with the laser scan emulation on the epucks


def drange(start, stop, step):
	r = start
	while r < stop:
		yield r
		r += step

def fmap(x, in_min, in_max, out_min, out_max):
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

class Controller(object):
	def __init__(self):
		rospy.init_node('controlNode', anonymous=True)

		#Get the robot to attach to
		botname = rospy.get_param(rospy.get_name()+"/robot_name")
		print "Controlling {0}".format(botname)
	
		#subscribe to the laser messages
		self.sub = rospy.Subscriber("/{0}/scan".format(botname), LaserScan, self.stupidLaserCallback)
		
		#Publish motor speeds
		self.pub = rospy.Publisher('/{0}/mobile_base/cmd_vel'.format(botname), Twist, queue_size=1)

		self.twist = Twist()

	#Callback for the motor speeds
	def laserCallback(self, laserData):
		#Creates a multiplier array with an entry for each sensor, with higher multipliers in the front
		#and lower ones on the sides
		angles = drange(laserData.angle_min, laserData.angle_max, laserData.angle_increment)
		angles = [abs(abs(x) - laserData.angle_max) for x in angles]
		
		#Set the weights for each range sensor based on the angles
		weightedRanges = []
		for index, distance in enumerate(laserData.ranges):
			weightedRanges.append(abs((laserData.range_max - distance) * angles[index]))
		
		#Get the front, left, and right range totals
		halfLen = len(weightedRanges)/2
		lVals = sum(weightedRanges[:halfLen])
		rVals = sum(weightedRanges[halfLen:])
		centerVals = sum(weightedRanges[halfLen -1: halfLen + 1])
		
		#print weightedRanges
		#print weightedRanges[:halfLen],weightedRanges[halfLen:] 
		#print weightedRanges[halfLen -1: halfLen + 1]
		#print "\n"
		
		#Calculate the twist to send to the robot. Only the x component of the linear velocity 
		#and the z component of the angular velocity are used. 
		max_speed = 4
		#Rotational speed is max speed times the ratio of left to right
		#with the sign based on difference of left to right 
		rot = 0.0

		if rVals > 0.005 and rVals > lVals:
			multiplier = lVals/rVals
			print "Right {0}".format(multiplier)
			rot = max_speed * -multiplier * 5  
		elif lVals < 0.005 and lVals > rVals:
			multiplier = rVals/lVals
			print "Left {0}".format(multiplier)
			rot = max_speed * multiplier * 5
		else:
			#they are the same, don't rotate
			pass

		#Linear speed is maximum speed minus the center values times a correction factor
		#to make the robot reverse if it is too close to something
		correction = 80
		lin = max_speed - (centerVals * correction)

		print rVals, lVals
		print lin, rot
		self.twist.linear.x = lin
		self.twist.angular.z = rot
		self.pub.publish(self.twist)

	def stupidLaserCallback(self, laserData):
		#print "Calback called"
		threshold = 0.07
		
		#Split the scan into large left and right ranges, small center range
		left = laserData.ranges[:8]
		center = laserData.ranges[8:11]
		right = laserData.ranges[11:]

		#Check the left and right sensors
		detectLeft = any([x < threshold for x in left])
		detectRight = any([x < threshold for x in right])
		detectCenter =  any([x < threshold for x in center])
		
		if detectCenter and detectLeft and detectRight:
			#Surrounded, go full reverse
			self.twist.linear.x = -4
			self.twist.angular.z = 0
		elif detectCenter:
			#Back up, with angle depending on left and right
			if detectRight:
				#Right motor more than left to back and turn away
				self.twist.linear.x = -2
				self.twist.angular.z = -2
			elif detectLeft:
				#Left motor more than right to back and turn away
				self.twist.linear.x = -2
				self.twist.angular.z = 2
			else:
				#Rotate in place
				self.twist.linear.x = 0
				#TODO could pick a random direction here
				self.twist.angular.z = 2
		elif detectLeft:
			#Turn to the right
			self.twist.linear.x = 2
			self.twist.angular.z = 2
		elif detectRight:
			self.twist.linear.x = 2
			self.twist.angular.z = -2			
		else:
			#No detection, go straight
			self.twist.linear.x = 5
			self.twist.angular.z = 0
		self.pub.publish(self.twist)

	def shutdown(self, signal, frame):
		#Stop listening for commands and stop the motors
		self.sub.unregister()
		self.twist.linear.x = 0.0
		self.twist.angular.z = 0.0
		self.pub.publish(self.twist)
		print "Shutting down"
		sys.exit(0)


#botname = sys.argv[1]
if __name__ == "__main__":
	control = Controller()
	signal.signal(signal.SIGINT, control.shutdown)
	rospy.spin()