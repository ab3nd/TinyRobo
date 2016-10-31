#ifndef NEIGHBOR_SENSE
#define NEIGHBOR_SENSE

#include "ros/ros.h"
#include <apriltags_ros/AprilTagDetectionArray.h>

class NeighborSense{
public:
	NeighborSense(ros::NodeHandle nh);
	void tagCallback(const apriltags_ros::AprilTagDetectionArray::ConstPtr& tagArray);
private:
	ros::NodeHandle nodeHandle;
	ros::Subscriber sub;
	std::map<int, ros::Publisher> robotSensors;
};
#endif
