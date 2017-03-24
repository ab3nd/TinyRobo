#!/usr/bin/python

#Send a random twist message to a tiny robo every few seconds. 

import rospy
from geometry_msgs.msg import Twist
import random

def Random_Driver():
	pub = rospy.Publisher('random_twists', Twist, queue_size=0)
	rospy.init_node('random_driver', anonymous=True)
	rate = rospy.Rate(0.3) #Every three seconds
	while not rospy.is_shutdown():
		#Random linear and rotational velocites
		#Twists are usually in m/sec, but 1m/sec is plenty fast for any of my robots
		#These are in the range +/-1, more or less 
		linear = (random.random()*2)-1
		rotational = (random.random()*2)-1

		rTwist = Twist()
		#only two params are used for robots on a table
		rTwist.linear.x = linear
		rTwist.angular.z = rotational
		#The rest are not used
		rTwist.linear.y = rTwist.linear.z = 0
		rTwist.angular.x = rTwist.angular.y = 0

		#publish it and wait
		pub.publish(rTwist)
		rate.sleep()

if __name__ == '__main__':
    try:
        Random_Driver()
    except rospy.ROSInterruptException:
        pass
