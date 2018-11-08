#!/usr/bin/python

# Turns gestures received from the UI and gesture nodes into programs to be dispatched to robots

# For the sake of simplicity, the first version of this program will only accept the "single move" command, 
# which is a drag of a single robot, from the location it is currently in, to a new location, where it stops. 

from user_interface.msg import Gesture
from robot_drivers.srv import MapPoint
from std_msgs.msg import String
import json
import rospy
import decompose_space
import math
import re
from lark import Lark, UnexpectedInput


class ProgSender(object):
	def __init__(self):
		self.robotPubs = {}
		self.topicRE = re.compile("\/bot([0-9]*)\/position")
		self.updatePubs()

	def updatePubs(self):
		#We can't get a list via rospy.get_published_topics(), because this node
		#is what's supposed to be publishing the topics, so get the IDs by looking at the 
		#other /botN/* topics
		for topic in rospy.get_published_topics():
			if self.topicRE.match(topic[0]):
				robot_id = self.topicRE.match(topic[0]).group(1)
				if robot_id not in self.robotPubs.keys():
					self.robotPubs[robot_id] = rospy.Publisher('/bot{}/robot_prog'.format(robot_id), String, queue_size=10)

	def pubProg(self, robot, program):
		#If we don't have a publisher for this robot yet, create one
		if robot not in self.robotPubs.keys():
			self.robotPubs[robot] = rospy.Publisher('/bot{}/robot_prog'.format(robot), String, queue_size=10)
			#Wait for publisher to be ready, and yes, this can be a problem. 
			rospy.sleep(0.5)

		self.robotPubs[robot].publish(json.dumps(program))


class ProgGen(object):
	def __init__(self):
		self.gestureSub = rospy.Subscriber("gestures", Gesture, self.addGesture)
		#All gestures seen so far, treated as a stack
		self.gestures = []

		#Spatial resolution of path points
		self.resolution = 0.3 #in meters

		#Object to handle sending programs to robots
		self.sender = ProgSender()

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

	def distance(self, p1, p2):
		return math.sqrt(math.pow(p1[0]-p2[0],2) + math.pow(p1[1]-p2[1],2))


	#Moving to a point with GCPR/Modified bug algo
	def go_point(self):
		program = []
		program.append(("self.at(self.goal)", "self.stop()", 1.0))
		program.append(("not(self.at(self.goal)) and not(self.is_near_anything())", "self.set_desired_heading(self.get_heading(self.goal))", 1.0))
		program.append(("not(self.at(self.goal)) and not(self.is_near_anything()) and self.on_heading()", "self.move_fwd(0.5)", 1.0))
		program.append(("not(self.at(self.goal)) and not(self.is_near_anything()) and not(self.on_heading())", "self.turn_heading(0.7)", 1.0))
		program.append(("not(self.at(self.goal)) and self.is_near_anything() and (self.get_l_front() == self.get_r_front() == 0)", "self.move_fwd(0.5)", 1.0))
		#----- OK down to here, so far -----
		program.append(("not(self.at(self.goal)) and self.is_near_anything() and (self.get_l_front() > 0 and self.get_r_front() == 0)", "self.move_arc(-0.8, 0.04)", 1.0))
		program.append(("not(self.at(self.goal)) and self.is_near_anything() and (self.get_l_front() == 0 and self.get_r_front() > 0)", "self.move_arc(0.8, 0.04)", 1.0))
		program.append(("not(self.at(self.goal)) and self.is_near_anything() and (self.get_l_front() > 0 and self.get_r_front() > 0) and self.get_l_front() <= self.get_r_front()", "self.move_arc(-0.8, -0.05)", 1.0))
		program.append(("not(self.at(self.goal)) and self.is_near_anything() and (self.get_l_front() > 0 and self.get_r_front() > 0) and self.get_l_front() > self.get_r_front()", "self.move_arc(-0.8, -0.05)", 1.0))
		return program

	def follow_path(self, path):
		program = self.go_point()
		for idx, point in enumerate(path):
			if idx == 0:
				#First point, just set it as the goal
				program.append(("self.prog_ctr == None", "self.set_pc({})".format(idx), 1.0))
				program.append(("self.prog_ctr == {}".format(idx), "self.set_goal({})".format(point), 1.0))
			elif idx == len(path)-1:
				#Last point
				#If the point is not reachable or we're there, stop
				program.append(("self.prog_ctr == {} and self.not_reachable({})".format(idx, point), "self.stop()", 1.0))
				program.append(("self.prog_ctr == {} and self.at({})".format(idx, point), "self.stop()", 1.0))
			else:
				#If we made it there, move on to the next point
				program.append(("self.prog_ctr == {} and self.at({})".format(idx, point), "self.set_pc({})".format(idx+1), 1.0))
				program.append(("self.at({})".format(point), "self.set_goal({})".format(path[idx+1]), 1.0))
				#If it's not reachable, increment the pc and move on to the next point
				program.append(("self.prog_ctr == {} and self.not_reachable({})".format(idx, point), "self.set_pc({})".format(idx+1), 1.0))
				program.append(("self.not_reachable({})".format(point), "self.set_goal({})".format(path[idx+1]), 1.0))

		for line in program:
			rospy.logwarn(line)
		return program

	def patrol(self, path):
		#Patrol is following a path without stopping
		program = self.go_point()
		for idx, point in enumerate(path):
			if idx == 0:
				#First point, just set it as the goal
				program.append(("self.prog_ctr == None", "self.set_pc({})".format(idx), 1.0))
				program.append(("self.prog_ctr == {}".format(idx), "self.set_goal({})".format(point), 1.0))
			else:
				#Roll the next index over at the end of the path
				next_idx = idx+1
				if next_idx >= len(path):
					next_idx = 0
				
				#If we made it there, move on to the next point
				program.append(("self.prog_ctr == {} and self.at({})".format(idx, point), "self.set_pc({})".format(next_idx), 1.0))
				program.append(("self.at({})".format(point), "self.set_goal({})".format(path[next_idx]), 1.0))
				#If it's not reachable, increment the pc and move on to the next point
				program.append(("self.prog_ctr == {} and self.not_reachable({})".format(idx, point), "self.set_pc({})".format(next_idx), 1.0))
				program.append(("self.not_reachable({})".format(point), "self.set_goal({})".format(path[next_idx]), 1.0))
		return program


	def disperse(self):
		program = []
		#Initially, pick a random heading to travel on
		program.append(("self.prog_ctr == 0", "self.set_desired_heading((random.random() * math.pi * 2)-math.pi)", 1.0))
		program.append(("self.prog_ctr == 0", "self.set_pc(1)", 1.0))

		#Move towards heading if not near anything
		program.append(("len(self.neighbors(d=4)) > 2 and not(self.is_near_anything()) and self.on_heading()", "self.move_fwd(0.4)", 1.0))
		program.append(("len(self.neighbors(d=4)) > 2 and not(self.is_near_anything()) and not(self.on_heading())", "self.turn_heading(0.8)", 1.0))

		# Too few neighbors, turn around		
		program.append(("len(self.neighbors(d=4)) < 2", "self.set_desired_heading(self.add_headings(self.current_heading, math.pi))", 1.0))
		#Just right neighbors, stop
		program.append(("len(self.neighbors(d=4)) == 2", "self.stop()", 1.0))

		#Obstacle avoid, but don't want to keep moving if the neighbor count is ok
		program.append(("not(len(self.neighbors(d=4)) == 2) and self.is_near_anything() and (self.get_l_front() == self.get_r_front() == 0)", "self.move_fwd(0.5)", 1.0))
		program.append(("not(len(self.neighbors(d=4)) == 2) and self.is_near_anything() and (self.get_l_front() > 0 and self.get_r_front() == 0)", "self.move_arc(-0.8, 0.04)", 1.0))
		program.append(("not(len(self.neighbors(d=4)) == 2) and self.is_near_anything() and (self.get_l_front() == 0 and self.get_r_front() > 0)", "self.move_arc(0.8, 0.04)", 1.0))
		program.append(("not(len(self.neighbors(d=4)) == 2) and self.is_near_anything() and (self.get_l_front() > 0 and self.get_r_front() > 0) and self.get_l_front() <= self.get_r_front()", "self.move_arc(-0.8, -0.05)", 1.0))
		program.append(("not(len(self.neighbors(d=4)) == 2) and self.is_near_anything() and (self.get_l_front() > 0 and self.get_r_front() > 0) and self.get_l_front() > self.get_r_front()", "self.move_arc(-0.8, -0.05)", 1.0))
		return program

	def move_object(self, object_loc, path):
		program = self.go_point()
		#Move to the object using modified bug
		program.append(("self.prog_ctr == None", "self.set_pc(1)", 1.0))
		program.append(("self.prog_ctr == 1", "self.set_goal({}})".format(object_loc), 1.0))
		#At the object and the point is in it, or at the goal point
		program.append(("self.prog_ctr == 1 and self.not_reachable({})".format(object_loc), "self.set_pc(2)", 1.0))
		program.append(("self.prog_ctr == 1 and self.at({})".format(object_loc), "self.set_pc(2)", 1.0))

		#Follow the path, but need to have this A) have the index start with the PC being 2 already, and B) add the code to push on things
		for idx, point in enumerate(path):
			if idx == 0:
				#First point, just set it as the goal
				program.append(("self.prog_ctr == None", "self.set_pc({})".format(idx), 1.0))
				program.append(("self.prog_ctr == {}".format(idx), "self.set_goal({})".format(point), 1.0))
			elif idx == len(path)-1:
				#Last point
				#If the point is not reachable or we're there, stop
				program.append(("self.prog_ctr == {} and self.not_reachable({})".format(idx, point), "self.stop()", 1.0))
				program.append(("self.prog_ctr == {} and self.at({})".format(idx, point), "self.stop()", 1.0))
			else:
				#If we made it there, move on to the next point
				program.append(("self.prog_ctr == {} and self.at({})".format(idx, point), "self.set_pc({})".format(idx+1), 1.0))
				program.append(("self.at({})".format(point), "self.set_goal({})".format(path[idx+1]), 1.0))
				#If it's not reachable, increment the pc and move on to the next point
				program.append(("self.prog_ctr == {} and self.not_reachable({})".format(idx, point), "self.set_pc({})".format(idx+1), 1.0))
				program.append(("self.not_reachable({})".format(point), "self.set_goal({})".format(path[idx+1]), 1.0))
		return program


	def formation(self, formation, robots):
		while len(robots) > len(formation):
			new_formation = []
			#There are more robots than points in the formation, so interpolate points on the formation
			for k in range(1,len(formation)):
				p1 = formation[k]
				p2 = formation[k-1]
				new_formation.append(p2)
				#average of points
				x = (p1[0] + p2[0])/2.0
				y = (p1[1] + p2[1])/2.0
				new_formation.append((x,y))
			formation = new_formation

		#Pick points on the formation and just go to those
		program = self.go_point()
		#TODO could assign each robot a different subset of the points in the formation, rather than all of them
		#Or try to satisfy some sort of distribution requirement
		for idx, point in enumerate(formation):
			if idx == 0:
				#First point, just set it as the goal
				program.append(("self.prog_ctr == None", "self.set_pc({})".format(idx), 1.0))
				program.append(("self.prog_ctr == {}".format(idx), "self.set_goal({})".format(point), 1.0))
			elif idx == len(path)-1:
				#Last point
				#If the point is not reachable or we're there, stop
				program.append(("self.prog_ctr == {} and self.not_reachable({})".format(idx, point), "self.stop()", 1.0))
				program.append(("self.prog_ctr == {} and self.at({})".format(idx, point), "self.stop()", 1.0))
			else:
				#If we made to a point on the formation, stop, we're "in formation"
				program.append(("self.prog_ctr == {} and self.at({})".format(idx, point), "self.stop()", 1.0))
				#If it's not reachable, increment the pc and move on to the next point
				program.append(("self.prog_ctr == {} and self.not_reachable({})".format(idx, point), "self.set_pc({})".format(idx+1), 1.0))
				program.append(("self.not_reachable({})".format(point), "self.set_goal({})".format(path[idx+1]), 1.0))
		return program
		

	def get_child(self, t, name):	
		for child in t.children:
			if child.data == name:
				return child
		return None

	def get_data(self, t, name):	
		for child in t.children:
			if child.data == name:
				return child
		return None

	def path_as_list(self, path):
		point_list = []
		try:
			if path.data == "point_list":
				for child in path.children:
					x = float(self.get_child(child, 'x').children[0].value)
					y = float(self.get_child(child, 'y').children[0].value)
					point_list.append((x,y))
				return point_list
			else:
				for c in path.children:
					point_list.extend(self.path_as_list(c))
				return point_list
		except AttributeError as e:
			#This is caused by trying to get path.data from a leaf node
			return point_list

	def robots_as_list(self, robots):
		robot_list = []
		try:
			if robots.data == "robot_list":
				for robot in robots.children:
					robot_id = int(robot.children[0].value)
					robot_list.append(robot_id)
				return robot_list
			else:
				for c in robots.children:
					robot_list.extend(self.robots_as_list(c))
				return robot_list
		except AttributeError as e:
			#This is caused by trying to get path.data from a leaf node
			return robot_list

	def handle_instruction(self, t):
		if t.data == 'start' or t.data == 'cmd':
			#Syntactic sugar, really care about children
			for child in t.children:
				self.handle_instruction(child)

		if t.data == 'gohere':
			#Could be a drag path, could be a selection and then path
			path = self.path_as_list(t)
			robots = self.robots_as_list(t)
			rospy.logwarn(path)
			program = self.follow_path(path)
			self.publishProgram(robots, program)

		if t.data == 'patrol':
			#Get the path and the selected robots
			path = self.path_as_list(t)
			robots = self.robots_as_list(t)
			#Give each robot a path
			program = self.patrol(path)
			self.publishProgram(robots, program)

		if t.data == 'makeformation':
		 	#Get the path and the selected robots
			formation = self.path_as_list(t)
			robots = self.robots_as_list(t)
			program = self.formation(formation, robots)
			self.publishProgram(robots, program)

		if t.data == 'moveobject':
		 	#Two selections to get, first is robots, second is object
		 	robot_selection = None
		 	obj_selection = None
		 	for child in t.children:
		 		if robot_selection is None:
		 			if child.data == 'selection':
		 				robot_selection = self.robots_as_list(child)
		 		else:
		 			if child.data == 'selection':
		 				#TODO at the moment, robots are the only selectable things
		 				obj_selection = self.robots_as_list(child)
		 	
		 	path = self.path_as_list(t)
		 	program = self.move_object(obj_selection, path)
		 	self.publishProgram(robots, program)

		if t.data == 'removerobot':
		 	#Get the selection
		 	robot = self.robots_as_list(t)
		 	rospy.logwarn("Delete {}".format(robot))
		 	#TODO this doesn't actually delete the robot

		if t.data == 'disperse':
		 	#Get the selection
		 	robots = self.robots_as_list(t)
		 	program = self.disperse()
		 	self.publishProgram(robots, program)
		

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

	def parseGestures(self, gesture_list):
		#Get a list of the gestures
		#prog_str = " ".join([g.eventName for g in gesture_list])

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
		# print prog_str
		# print parse_tree.pretty()
		# print "---"

		# for t in parse_tree.children:
		self.handle_instruction(parse_tree)

	def checkGestureList(self):
		#If the last thing in is an end gesture, we're good to try to parse the gestures
		if self.gestures[-1].eventName == "end":
			print "--> gesture ended"

			#Call the parser on it
			try:
				#Parse it to build the tree
				self.parseGestures(self.gestures)
			except UnexpectedInput as e:
					print e
			finally:
				#Flush the gesture buffer
				self.gestures = []

		#If the last thing in is a select, and there are any selects already present in the buffer,
		#then it's time to end the previous gesture and parse that, and leave the select in as
		#the beginning of a new gesture
		elif self.isSelect(self.gestures[-1]) and any([self.isSelect(x) for x in self.gestures[:-1]]):
			#A second select is possible for a move gesture, so we can accept it in this case
			if not any(x.eventName == 'move_object' for x in self.gestures):
				print "--> select started new gesture"
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
						print e
				finally:			
					#Clear the stuff that was just sent to the parser, but leave the 
					#selection gesture that kicked off this parse attempt
					self.gestures = [self.gestures[-1]]

	def isPath(self, gestureMsg):
		if gestureMsg.eventName == "path":
			return True
		return False

	def isSelect(self, gestureMsg):
		if gestureMsg.eventName in ["box_select", "lasso_select"]: # tap not listed, we can chain tap selects
			return True
		return False

	def publishProgram(self, robots, program):
		import pprint 
		#pprint.pprint(robots)
		#pprint.pprint(program)
		for robot in robots:
			self.sender.pubProg(robot, program)

#Start everything up and then just spin
rospy.init_node('gcpr_gen')
progam_generator = ProgGen()
rospy.spin()