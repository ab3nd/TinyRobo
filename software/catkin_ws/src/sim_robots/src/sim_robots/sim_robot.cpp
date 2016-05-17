#include <sim_robots/sim_robot.h>

SimRobot::SimRobot(ros::NodeHandle node, std::string driver)
{
	//Listen for updates from the world
	clockSub = node.subscribe("/sim_world/sim_world_clock", 1, &SimRobot::timeCallback, this);
	//Listen for motion commands to this robot
	cmdSub = node.subscribe(driver + "/drive_cmd", 1, &SimRobot::motorCallback, this);
}

void SimRobot::timeCallback(const std_msgs::Header::ConstPtr& msg)
{
	//ROS_INFO("%s got %f ",ros::this_node::getName().c_str(), msg->stamp.toSec());
	//TODO Update this robot's position from time elapsed and motions
}

void SimRobot::motorCallback(const tiny_robo_msgs::Motor_Vel_Cmd::ConstPtr& msg)
{
	ROS_INFO("%s got %d, %d", ros::this_node::getName().c_str(), msg->motor1, msg->motor2);
	//TODO update the motor speeds
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "sim_robot");
	ros::NodeHandle node("~");

	//Attempt to get the driver parameter to determine which driver this robot listens to
	std::string drv_src;
	// Topic to listen to
	node.param < std::string > (ros::this_node::getName() + "/translator", drv_src, "/default_driver/drive_cmd");

	SimRobot sr = SimRobot(node, drv_src);

	ros::spin();
}
