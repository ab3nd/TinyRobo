#ifndef RANDOM_DRIVER_H_
#define RANDOM_DRIVER_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <vector>
//#include <random> Can't do this because ROS coding standards call for a version of C++ that's nearly old enough to drive
#include <cstdlib>

class RandomDriver
{
	private:
		//Publishes twist messages
		ros::Publisher twistPub;

	public:
		RandomDriver(ros::NodeHandle node);
		void drive();
};
#endif
