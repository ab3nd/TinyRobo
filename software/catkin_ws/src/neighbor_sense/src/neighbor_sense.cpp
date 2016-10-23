#include <neighbor_sense/neighbor_sense.h>
#include <std_msgs/String.h>

NeighborSense::NeighborSense(ros::NodeHandle nh){
	  sub = nh.subscribe("/tag_detections", 10, &NeighborSense::tagCallback, this);

	  //Keep the nodehandle, we'll need it to set up the publishers
	  nodeHandle = nh;
}

void NeighborSense::tagCallback(const apriltags_ros::AprilTagDetectionArray::ConstPtr& tagArray)
{
	ROS_WARN("Tag callback called");
	//For each visible tag
	for(std::vector<apriltags_ros::AprilTagDetection>::const_iterator tag = tagArray->detections.begin(); tag != tagArray->detections.end(); ++tag)
	{
		//If a publisher for that tag does not exist
		if(robotSensors.count((*tag).id) == 0)
		{
			std::stringstream publisherName;
			publisherName << "robot_" << (*tag).id << "_neighbors";
			//TODO this isn't a string
			ros::Publisher p = nodeHandle.advertise<std_msgs::String>(publisherName.str(), 100);
			//store it
			robotSensors[(*tag).id] = p;
		}

		//Publish the distances from this tag to all others
		std_msgs::String msg;
		msg.data = "LOL test";
		robotSensors[(*tag).id].publish(msg);
	}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "NeighborSense");
	ros::NodeHandle node("~");

	NeighborSense ns = NeighborSense(node);

	ros::spin();
}
