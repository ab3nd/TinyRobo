#!/usr/bin/python

#Given a robot ID, regularly output messages as if this was a range and bearing sensor
#on that robot that gets the ranges and bearings to other robots
import rospy
from distance_oracle.srv import *
from bearing_oracle.srv import *
from range_and_bearing.msg import RangeAndBearing
from apriltags_ros.msg import *

class TagTracker(object):
	def __init__(self):
		self.currentTags = {}
		self.tagSub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.update_tags)

	def update_tags(self, msg):
		self.currentTags = {}
		for ii in range(len(msg.detections)):
			self.currentTags[int(msg.detections[ii].id)] = msg.detections[ii].pose

	def tag_list(self):
		return self.currentTags.keys()

def RaBSensor():
	rospy.init_node('range_and_bearing', anonymous=True)
	#ID of this robot
	id = rospy.get_param("~robot_id", 0)	
	#Rotation of the april tag relative to the robot front, in radians
	tag_rotation = rospy.get_param("~tag_rotation", 0)

	pub = rospy.Publisher('range_and_bearing'.format(id), RangeAndBearing, queue_size=0)

	#TODO make this a parameter
	rate = rospy.Rate(3)

	rospy.wait_for_service("/distance_oracle")
	rospy.wait_for_service("/bearing_oracle")
	
	#Set up to get a list of tags
	tt = TagTracker()
		
	while not rospy.is_shutdown():
		#Try the service call
		try:
			robots = tt.tag_list()
			msgOut = RangeAndBearing()
			#Set up proxies once
			distMsg = rospy.ServiceProxy("/distance_oracle", DistanceOracle)
			bearingMsg = rospy.ServiceProxy("/bearing_oracle", BearingOracle)
					
			for ii in robots:
				if ii != id: #I already know how far I am from myself...
					#Repeated proxy calls
					dResponse = distMsg(fromID = id, toID = ii)
					bResponse = bearingMsg(fromID = id, toID = ii)
					#Stick it in the message if it's not myself
					msgOut.ids.append(ii)
					msgOut.ranges.append(dResponse.distance)
					msgOut.bearings.append(bResponse.bearing)
			#Publish the message
			pub.publish(msgOut)

		except rospy.ServiceException, e:
			rospy.logwarn("Service call failed: {0}".format(e))

		rate.sleep()

if __name__ == '__main__':
    try:
        RaBSensor()
    except rospy.ROSInterruptException:
        pass
