#!/usr/bin/python

#Given a robot ID, regularly output messages as if this was a range and bearing sensor
#on that robot that gets the ranges and bearings to other robots
import rospy
from distance_oracle.srv import *
from bearing_oracle.srv import *
from range_and_bearing.msg import RangeAndBearing

def RaBSensor():
	rospy.init_node('range_and_bearing', anonymous=True)
	#ID of this robot
	id = rospy.get_param("~robot_id", 0)	
	#Rotation of the april tag relative to the robot front, in radians
	tag_rotation = rospy.get_param("~tag_rotation", 0)

	pub = rospy.Publisher('range_and_bearing_{0}'.format(id), RangeAndBearing, queue_size=0)

	#TODO make this a parameter
	rate = rospy.Rate(3)

	rospy.wait_for_service("distance_oracle")
	rospy.wait_for_service("bearing_oracle")
		
	while not rospy.is_shutdown():
		#Try the service call
		try:
			#TODO THIS IS A GRODY HACK
			robots = [0,1,3]
			for ii in robots:
				distMsg = rospy.ServiceProxy("distance_oracle", DistanceOracle)
				dResponse = distMsg(fromID = id, toID = ii)

				bearingMsg = rospy.ServiceProxy("bearing_oracle", BearingOracle)
				bResponse = bearingMsg(fromID = id, toID = ii)

				rospy.logwarn("{0} -> {1}, {2}, {3}".format(id, ii, dResponse.distance, bResponse.bearing))

		except rospy.ServiceException, e:
			print "Service call failed: {0}".format(e)

		rate.sleep()

if __name__ == '__main__':
    try:
        RaBSensor()
    except rospy.ROSInterruptException:
        pass
