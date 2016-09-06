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
	def __init__(self, botname):
		rospy.init_node('controlNode', anonymous=True)

		#subscribe to the laser messages
		self.sub = rospy.Subscriber("/{0}/scan".format(botname), LaserScan, self.laserCallback)
		
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

	def shutdown(self, signal, frame):
		#Stop listening for commands and stop the motors
		self.sub.unregister()
		self.twist.linear.x = 0.0
		self.twist.angular.z = 0.0
		self.pub.publish(self.twist)
		sys.exit(0)


botname = "epuck_robot_9"

control = Controller(botname)
signal.signal(signal.SIGINT, control.shutdown)
rospy.spin()