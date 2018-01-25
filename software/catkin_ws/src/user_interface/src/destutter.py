#!/usr/bin/python

#ROS wrapper for gesture stroke consolidation, mostly getting gestures that are broken up 
#by stuttering contact with the touchscreen into single sets of events

import rospy
from geometry_msgs.msg import PointStamped

import touch_cleaner

#Subscribe to the point messages from the UI 

def touch_event_callback(event):
	print "called"

topic = "/touches"
rospy.init_node('touch_destutter')
touchSub = rospy.Subscriber(topic, PointStamped, touch_event_callback)
rospy.spin()