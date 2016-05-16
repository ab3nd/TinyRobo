#include <sim_robots/sim_robot.h>

SimRobot::SimRobot(ros::NodeHandle node)
{
	//Listen for updates from the world
	clockSub = node.subscribe("/sim_world/sim_world_clock", 1, &SimRobot::timeCallback, this);
	//Listen for motion commands to this robot
	cmdSub = node.subscribe(ros::this_node::getName() + "/drive_cmd", 1, &SimRobot::motorCallback, this);
}

void SimRobot::timeCallback(const std_msgs::Header::ConstPtr& msg)
{
	ROS_INFO("%s got %f ",ros::this_node::getName().c_str(), msg->stamp.toSec());
}

void SimRobot::motorCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	ROS_INFO("%s got %f, %f", ros::this_node::getName().c_str(), msg->linear.x, msg->angular.z);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "sim_robot");
	ros::NodeHandle node("~");

	SimRobot sr = SimRobot(node);

	ros::spin();
}
