#include <sim_robots/sim_robot.h>

void SimRobot::timeCallback(const std_msgs::HeaderConstPtr& msg)
{
	ROS_INFO(msg->stamp);
}

void SimRobot::motorCallback()
{
	//Do nothing for the moment
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "sim_robot");
	ros::NodeHandle node("~");

	SimRobot sr();

	//Listen for updates from the world
	clockSub = node.subscribe("sim_world_clock", 1, &SimRobot::timeCallback, &sr);

	//TODO Listen for motor velocity commands
}
