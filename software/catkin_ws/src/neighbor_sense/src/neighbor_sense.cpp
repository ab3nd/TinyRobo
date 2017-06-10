#include <neighbor_sense/neighbor_sense.h>
#include <std_msgs/String.h>

NeighborSense::NeighborSense(ros::NodeHandle nh){
	  sub = nh.subscribe("/tag_detections", 10, &NeighborSense::tagCallback, this);

	  //Keep the nodehandle, we'll need it to set up the publishers
	  nodeHandle = nh;
}

void NeighborSense::tagCallback(const apriltags_ros::AprilTagDetectionArray::ConstPtr& tagArray)
{
	//ROS_WARN("Tag callback called");
	//For each visible tag
	for(std::vector<apriltags_ros::AprilTagDetection>::const_iterator tag = tagArray->detections.begin(); tag != tagArray->detections.end(); tag++)
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
		for(std::vector<apriltags_ros::AprilTagDetection>::const_iterator otherTag = tagArray->detections.begin(); otherTag != tagArray->detections.end(); otherTag++)
		{
			//Euclidian distance in 2D. Could extend to 3D, but tag occlusion might become an issue.
			float thisX = (*tag).pose.pose.position.x;
			float thisY = (*tag).pose.pose.position.y;
			float otherX = (*otherTag).pose.pose.position.x;
			float otherY = (*otherTag).pose.pose.position.y;
			float distance = sqrt(pow((thisX - otherX),2) + pow((thisY - otherY),2));

			//Get the rotation in the XY (yaw), YZ (roll), and ZX (pitch) planes
			//For robots on the table, roll and pitch should be quite small
			float thisZ = (*tag).pose.pose.position.z;
			float otherZ = (*otherTag).pose.pose.position.z;
			float yaw = atan2((thisY-otherY),(thisX-otherX));
			float roll = atan2((thisY-otherY), (thisZ-otherZ));
			float pitch = atan2((thisX-otherX), (thisZ-otherZ));

			//Create a quaternion from rotation vectors

			//Text message for debug reasons, convert to something else when it's working
			std_msgs::String msg;
			std::stringstream debug;
			debug << (*tag).id << " -> " << (*otherTag).id << " : " << distance << std::endl << roll << ", " << pitch << ", " << yaw;
			msg.data = debug.str();
			robotSensors[(*tag).id].publish(msg);
		}
	}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "NeighborSense");
	ros::NodeHandle node("~");

	NeighborSense ns = NeighborSense(node);

	ros::spin();
}
