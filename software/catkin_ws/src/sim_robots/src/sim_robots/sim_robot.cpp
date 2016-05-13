#include <sim_robots/sim_robot.h>

SimRobot::SimRobot(ros::NodeHandle node)
{
	//Listen for updates from the world
	clockSub = node.subscribe("/sim_world/sim_world_clock", 1, &SimRobot::timeCallback, this);
}

void SimRobot::timeCallback(const std_msgs::Header::ConstPtr& msg)
{
	ROS_INFO("%s got %f ",ros::this_node::getName().c_str(), msg->stamp.toSec());
}

void SimRobot::motorCallback()
{
	//Do nothing for the moment
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "sim_robot");
	ros::NodeHandle node("~");

	SimRobot sr = SimRobot(node);

	ros::spin();
}
