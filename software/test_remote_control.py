#!/usr/bin/python
# -*- coding: utf-8 -*-

# Stupid remote control for robots from a pygame interface. 
# The interface provides connection control, l/r arrows are one motor, 
# up/down are the other motor, spacebar to stop. 

import Tkinter as Tk
import socket
import errno
import time

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
		self.connection.send(bytearray([ord('M'), cmd[0], cmd[1], cmd[2], cmd[3]]))

class remoteControl:
	def __init__(self, window):
		#For connecting to the robot
		self.robot =  RobotComms()
		self.command = [0x00, 0x00, 0x00, 0x00]

		#Connection IP address entry field
		ipFrame = Tk.Frame(window)
		ipFrame.pack()
		self.ipAddr = Tk.Entry(ipFrame)
		self.ipAddr.pack(side=Tk.LEFT)
		ipButton = Tk.Button(ipFrame, text="Connect", command=self.connectIP)
		ipButton.pack(side=Tk.RIGHT)

		controlFrame = Tk.Frame(window)
		controlFrame.pack()
		#Motor one frame
		m1Frame = Tk.Frame(controlFrame)
		m1Frame.pack(side = Tk.LEFT)
		m1Label = Tk.Label(m1Frame, text="Motor 1")
		m1Label.pack(side=Tk.TOP)
		m1Fwd = Tk.Button(m1Frame, text="⇧", command=self.motor1Fwd)
		m1Fwd.pack(side = Tk.LEFT)
		m1Back = Tk.Button(m1Frame, text="⇩", command=self.motor1Back)
		m1Back.pack(side = Tk.RIGHT)

		#Motor two frame
		m2Frame = Tk.Frame(controlFrame)
		m2Frame.pack(side = Tk.RIGHT)
		m2Label = Tk.Label(m2Frame, text="Motor 2")
		m2Label.pack(side=Tk.TOP)
		m2Fwd = Tk.Button(m2Frame, text="⇧", command=self.motor2Fwd)
		m2Fwd.pack(side = Tk.LEFT)
		m2Back = Tk.Button(m2Frame, text="⇩", command=self.motor2Back)
		m2Back.pack(side = Tk.RIGHT)

		stopButton = Tk.Button(controlFrame, text="Stop", command=self.stopAll)
		stopButton.pack()

	def stopAll(self):
		self.command = [0x00, 0x00, 0x00, 0x00]
		self.robot.sendMotor(self.command)

	def motor1Fwd(self):
		self.command[0] = 0x20
		self.command[1] = 0x02
		self.robot.sendMotor(self.command)

	def motor2Fwd(self):
		self.command[2] = 0x20
		self.command[3] = 0x02
		self.robot.sendMotor(self.command)

	def motor1Back(self):
		self.command[0] = 0x20
		self.command[1] = 0x01
		self.robot.sendMotor(self.command)

	def motor2Back(self):
		self.command[2] = 0x20
		self.command[3] = 0x01
		self.robot.sendMotor(self.command)

	def connectIP(self):
		if self.robot.connect(self.ipAddr.get()):
			print "Got connection"
		else:
			print "Connection Failed"

if __name__ == "__main__":
	root = Tk.Tk()
	app = remoteControl(root)
	root.mainloop()
