#!/usr/bin/python

from lark import Lark, UnexpectedInput
from user_interface.msg import Gesture
from robot_drivers.srv import MapPoint
import subprocess
import roslaunch
import argparse
import fnmatch
import rospy
import sys
import os


class ProgGen(object):
	def __init__(self, logpath = "test_log.text"):
		self.gestureSub = rospy.Subscriber("gestures", Gesture, self.addGesture)
		#All gestures seen so far, treated as a stack
		self.gestures = []

		self.throw_count = 0

		#Open a log file for append, we tear this generator down each time 
		#so there isn't saved state
		self.logfile=open(logpath, "w")

		#Implemented as per my thesis paper, only without CamelCase because lark is case sensitive
		gesture_grammar='''
			start: (patrol | makeformation | moveobject | removerobot | disperse | gohere) "end"
			patrol: selection "patrol" path
			makeformation: selection "make_formation" path
			moveobject: selection "move_object" selection path
			removerobot: "remove_robot" selection
			disperse: selection (robot_path | path) ~ 4..5  
			gohere: selection path | robot_path
			path: drag_path | waypoint+ 
			selection: gestureselect | groupselect
			gestureselect: tap_robot+ | lasso | box
			groupselect: "select_group" tap_robot

			robot_path: "drag_robot" robot_list point_list

			waypoint: "tap_waypoint" point_list

			drag_path: "path" point_list

			box: "box_select" robot_list

			lasso: "lasso_select" robot_list

			tap_robot: "tap_select" robot_list

			robot_list: "[" robot_id+ "]"
			robot_id: INTEGER 

			point_list: "[" point+ "]"
			point: "(" x "," y ")"
			x: DECIMAL
			y: DECIMAL

			INTEGER: ("0".."9")+
			DECIMAL.2: "-"? INTEGER "." INTEGER

			%import common.WS
			%ignore WS
		'''

		self.parser = Lark(gesture_grammar)

	def addGesture(self, gestureMsg):
		self.gestures.append(gestureMsg)
		#TODO checking the gesture list can be made conditional on the arriving gesture. 
		#This allows timeouts (sort-of, more than Nsec since the previous gesture), a "go" gesture,
		#or firing off the previous command when selection begins a new command. 
		self.checkGestureList()
		


	def parseGestures(self, gesture_list):
		#Get a list of the gestures
		prog_str = " ".join([g.eventName for g in gesture_list])
		#Log it for debugging/testing
		self.log("Program: {}\n".format(prog_str))
		#Generate a code string from the gestures in the buffer
		prog = []
		for g in gesture_list:
			prog.append(g.eventName)
			if g.eventName in ["tap_select", "box_select", "lasso_select"]:
				prog.append("[")
				prog.extend([str(x) for x in g.robots])
				prog.append("]")
			if g.eventName in ["drag_robot"]:
				prog.append("[")
				prog.extend([str(x) for x in g.robots])
				prog.append("]")
				prog.append("[")
				prog.extend(self.get_path(g))
				prog.append("]")
			if g.eventName in ["path"]:
				prog.append("[")
				prog.extend(self.get_path(g))
				prog.append("]")
			if g.eventName in ["tap_waypoint"]:
				prog.append("[")
				prog.extend(self.get_start(g)) #Only has one point
				prog.append("]")
		
		prog_str = " ".join(prog)
		parse_tree = self.parser.parse(prog_str)

		#For now just prettyprint it
		self.log(prog_str)
		self.log("\n")
		self.log(parse_tree.pretty())
		self.log("\n")
		self.log("---")
		self.log("\n")
		# for t in parse_tree.children:
		#Don't bother, we just want to see if it parses at all
		#self.handle_instruction(parse_tree)

	def checkGestureList(self):
		#rospy.logwarn("1. {} gestures in queue".format(len(self.gestures)))
		#If the last thing in is an end gesture, we're good to try to parse the gestures
		if self.gestures[-1].eventName == "end":
			self.log("--> gesture ended\n")
			#Call the parser on it
			#rospy.logwarn("1.2 {} gestures in queue".format(len(self.gestures)))
			try:
				#Parse it to build the tree
				self.parseGestures(self.gestures)
			except UnexpectedInput as e:
				self.log(str(e))
			#Flush the gesture buffer
			self.gestures = []
			#rospy.logwarn("1.3 {} gestures in queue".format(len(self.gestures)))

		#If the last thing in is a select, and there are any selects already present in the buffer,
		#then it's time to end the previous gesture and parse that, and leave the select in as
		#the beginning of a new gesture
		elif self.isSelect(self.gestures[-1]) and any([self.isSelect(x) for x in self.gestures[:-1]]):
			import pdb; pdb.set_trace()
			#A second select is possible for a move gesture, so we can accept it in this case
			if not any(x.eventName == 'move_object' for x in self.gestures):
				#rospy.logwarn("2.1 {} gestures in queue".format(len(self.gestures)))
				self.log("--> select started new gesture\n")
				#insert an end to it and ship it to the parser
				toParse = self.gestures[:-1]
				evt = Gesture()
				evt.eventName = "end"
				evt.stamp = rospy.Time.now()
				evt.isButton = False 
				evt.robots = []
				evt.strokes = []
				toParse.append(evt)

				#Call the parser on it
				try:
					self.parseGestures(toParse)
				except UnexpectedInput as e:
					self.log(str(e))
				#Clear the stuff that was just sent to the parser, but leave the 
				#selection gesture that kicked off this parse attempt
				#rospy.logwarn("2.2 {} gestures in queue".format(len(self.gestures)))
				self.gestures = [self.gestures[-1]]
				#rospy.logwarn("2.3 {} gestures in queue".format(len(self.gestures)))

	def isPath(self, gestureMsg):
		if gestureMsg.eventName == "path":
			return True
		return False

	def isSelect(self, gestureMsg):
		if gestureMsg.eventName in ["box_select", "lasso_select"]: # tap not listed, we can chain tap selects
			return True
		return False


	def get_path(self,g):
		path = []
		pm = rospy.ServiceProxy('map_point', MapPoint)
		for e in g.strokes:
			for p in e.events:
				resp = pm(p.point)
				path.append("(" + str(resp.inMeters.x) + "," + str(resp.inMeters.y)+ ")")
		return path
	
	def get_start(self,g):
		path = []
		path.append("(" + str(g.strokes[0].events[0].point.x) + "," + str(g.strokes[0].events[0].point.y)+ ")")
		return path

	def log(self, message):
		self.logfile.write(message)
		self.logfile.write("\n")
		self.logfile.flush()

	def flush(self):
		self.logfile.flush()
		
#From https://stackoverflow.com/questions/1724693/find-a-file-in-python
def find(pattern, path):
    result = []
    for root, dirs, files in os.walk(path):
        for name in files:
            if fnmatch.fnmatch(name, pattern):
                result.append(os.path.join(root, name))
    return result


#Set up the mock generator
rospy.init_node('mock_gcpr_gen')

#The script can publish gestures too, so it can send end gestures
gesturePub = rospy.Publisher("gestures", Gesture, queue_size=10)

mapperP = subprocess.Popen(["rosrun", "robot_drivers", "sim_point_mapper_svc.py"])

#Get the gesture files
# basepath = "./good_run/"
# gesture_files = find("*gestures.bag", basepath)

#bags = sorted(gesture_files)
#for bag in bags: # sorted(gesture_files):

bag = sys.argv[1]

outfile = os.path.basename(bag).replace("gestures.bag", "output.txt")

#This automatically subscribes to /gestures
progam_generator = ProgGen(outfile)

rospy.sleep(1)

progam_generator.log("\n========================================\n" + bag)	

#play the gesture bag file
rosbagP = subprocess.Popen(["rosbag", "play", bag])
rospy.loginfo("Playing bag, please wait")

rosbagP.wait()

#Send a double-tap
evt = Gesture()
evt.eventName = "end"
evt.stamp = rospy.Time.now()
evt.isButton = False 
evt.robots = []
evt.strokes = []
gesturePub.publish(evt)

rospy.sleep(3)

progam_generator.log("\ndone.")
	
def myhook():
  mapperP.terminate()
  # rosbagP.terminate()

rospy.on_shutdown(myhook)