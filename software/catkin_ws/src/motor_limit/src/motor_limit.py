#!/usr/bin/python

# Handles communication with a TinyRobo to send it motor commands and to limit 
# the max voltage used to avoid overcurrent conditions. 

import rospy
import socket
import select

from tiny_robo_msgs.msg import Motor_Vel_Cmd


class RobotComms:
	def __init__(self):
		self.connection = None

	def connect(self, ip):
		self.connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.connection.connect((ip, 4321))
		return self.isConnected()
		
	def isConnected(self):
		self.connection.send('Q') 
		#Pretty hackey, might not get the full thing in one go. 
		received = self.connection.recv(10)
		if received.startswith("TinyRobo"):
			return True
		else:
			return False

	def sendMotor(self, cmd):
		#Send the motor speeds and directions
		self.connection.send(bytearray([ord('M'), cmd[0], cmd[1], cmd[2], cmd[3]]))
		
		#Get the reply from the motor driver and convert it into an array of integers
		buff = ""
		#TODO if we're not writing anything back, this is going to be an issue
		while len(buff) < 6:
			#Wait two seconds to be able to read
			retval = select.select([self.connection], [], [], 2.0)
			if len(retval[0]) == 0:
				rospy.logwarn("Timed out waiting for response from robot")
			else:
				buff += self.connection.recv(6)
		return bytearray(buff)

  # According to the DRV8830 data sheet, there are two registers:
  #   0x00 - Control
  #   bit function
  #   7-2 Output voltage (VSET), which roughly correlates with speed in DC motors
  #   1   IN2 One of the H-bridge control lines
  #   0   IN1 The other H-bridge control line

  #   The valid range for VSET is 0x06h (0.48V) to 0x3Fh (5.06V) in 0.08V increments.
  #   My batteries are 3.7V, so 0x30h (3.86V) ahd higher is 100% power.
  #   0x00h-0x05h are "reserved".

  #   H-bridge truth table
  #   IN1 IN2 OUT1  OUT2  Function
  #   0   0   Z     Z     Coast (motor leads high impedence)
  #   0   1   L     H     Reverse
  #   1   0   H     L     Forward (relative to "reverse", anyway)
  #   1   1   H     H     Brake

  #   0x01 - Fault
  #   bit function
  #   7   CLEAR, clears fault status when written to 1
  #   6-5 Unused
  #   4   Current limit (ILIMIT) if set, indicates fault was extended overcurrent
  #   3   Overtemperature (OTS) if set, chip is too hot
  #   2   undervoltage (UVLO) if set, supply voltage is too low
  #   1   Overcurrent (OCP) if set, overcurrent event
  #   0   FAULT, if set, a fault condition exists

class motorLimiter:
	def __init__(self):
		#Tiny robos have two motors
		self.m1_maxV = 0x30
		self.m2_maxV = 0x30
		self.robot = RobotComms()

		self.ILIMIT = 0x010
		self.OTS = 0x08
		self.UVLO = 0x04
		self.OCP = 0x02
		self.FAULT = 0x01

	def connect(self, ipAddr):
		if self.robot.connect(ipAddr):
			rospy.loginfo("Connected to robot at {0}".format(ipAddr))
			return True
		else:
			rospy.logerr("Could not connect to robot at {0}".format(ipAddr))
			return False

	def mapSpeed(self, cmd, motor):
		#Get the direction
		#TODO this may, unfortunately, depend on the robot's wiring. Make configurable
		direction = 0x00
		if cmd > 0:
			direction = 0x01
		elif cmd < 0:
			direction = 0x02

		#Get the maximum cutoff for the motor
		# and use it to set the speed
		maxV = 0x00
		if motor == 0:
			maxV = self.m1_maxV
		else:
			maxV = self.m2_maxV
		cmd = abs(cmd)
		speed = int((cmd * maxV) / 128.0)

		return speed, direction
		

	def setSpeeds(self, m1_cmd, m2_cmd):
		m1_dir = 0x00
		m1_speed = 0x00
		m2_dir = 0x00
		m2_speed = 0x00

		#Map each motor speed to a speed/direction value for the motor
		m1_speed, m1_dir = self.mapSpeed(m1_cmd, 0)
		m2_speed, m2_dir = self.mapSpeed(m2_cmd, 1)

		#rospy.logwarn("({0},{1}), ({2},{3})".format(m1_speed, m1_dir, m2_speed, m2_dir))
		#Send the speed/direction values
		#Result is [motor1 speed, motor1 dir, motor1 status, motor2 speed, motor2 dir, motor2 status]
		result = self.robot.sendMotor([m1_speed, m1_dir, m2_speed, m2_dir])
		#rospy.logwarn(result) Not useful, they're not printable chars

		#If there's a problem, adapt the motor voltage downward
		#The conversions to int are because otherwise result is treated as a string
		if (result[2] & self.ILIMIT) or (result[2] & self.OCP):
			pass
			#self.m1_maxV = max(0, self.m1_maxV - 1)
			#rospy.logwarn("Overcurrent on motor 1, reducing power to {0}".format(self.m1_maxV))
		elif result[2] & self.UVLO:
			#rospy.logwarn("Supply undervoltage on motor 1")
			pass

		if (result[5] & self.ILIMIT) or (result[5] & self.OCP):
			pass
			#self.m2_maxV = max(0, self.m2_maxV - 1)
			#rospy.logwarn("Overcurrent on motor 2, reducing power to {0}".format(self.m2_maxV))
		elif result[5] & self.UVLO:
			#rospy.logwarn("Supply undervoltage on motor 2")
			pass

	def handleMessage(self, msg):
		self.setSpeeds(msg.motor1, msg.motor2)

if __name__ == "__main__":
	rospy.init_node('motor_speed_limiter')

	#Get the IP address of the robot to connect to
	ipAddr = rospy.get_param("~robot_addr")

	#TODO I may want to set up static leases in the wifi router to attempt 
	#ipAddr = '192.168.1.119' #hexbugbase
	#ipAddr = '192.168.1.176' #medium-size tank
	#ipAddr = '192.168.1.119' #TODO this should be a conf parameter with ROSPARAM
	#ipAddr = '192.268.1.101' #2-wheeler

	#Get a motor speed limiter object and connect it to the robot at the given IP address
	ml = motorLimiter()
	ml.connect(ipAddr)

	#Subscribe to a motor command message channel
	sub = rospy.Subscriber("/default_driver/drive_cmd", Motor_Vel_Cmd, ml.handleMessage)
	#Publish a motor command message channel
	pub = rospy.Publisher('limited_cmds', Motor_Vel_Cmd, queue_size=10)

	rospy.spin()