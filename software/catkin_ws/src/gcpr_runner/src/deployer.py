#!/usr/bin/python

# Handle deployment of a program to the robots. 

# Given a robot and a program, the robot should begin executing the program. 
# I think the way to do this may be to have the robot's progam executer/GCPR evaluator
# listening for dynamic reconfiguration or something like that, and automatically 
# evaluate the new program once it gets reconfigured. 
# It's probably also possible to have this simply send the GCPR program as a ROS message that
# contains the new code, rather than using dynamic reconfigure. 

import rospy
from std_msgs.msg import String
import json 

#I'm not sure how quotes should be escaped here
test_prog = [("True", 1.0, "dbg_print(\"Test Program is running\")")]

rospy.init_node("gcpr_deployer")

pub = rospy.Publisher('/robot_prog/7', String, queue_size=10)

r = rospy.Rate(1)

while not rospy.is_shutdown():
	pub.publish(String(json.dumps(test_prog)))
	r.sleep()



