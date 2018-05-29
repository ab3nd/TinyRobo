#!/usr/bin/python


import rospy
from geometry_msgs.msg import Twist
from apriltags_ros.msg import *
import random
import math
from tf import transformations as trans

#Attempt at a controller that picks random points and moves a single robot to them

#Subscribe to position feed

#Subscribe to camera to show points for debug

#Pick a location

#Highlight the location

#Move the robot