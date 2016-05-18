#include <sim_robots/sim_robot.h>

SimRobot::SimRobot(ros::NodeHandle node)
{
	//Attempt to get the driver parameter to determine which driver this robot listens to
	std::string driver;
	// Topic to listen to
	node.param < std::string > (ros::this_node::getName() + "/translator", driver, "/default_driver/drive_cmd");
	//Listen for motion commands to this robot
	cmdSub = node.subscribe(driver + "/drive_cmd", 1, &SimRobot::motorCallback, this);

	//Attempt to get the initial position of the robot within the world
	location = std::vector<float>(3, 0); //three ints with the value 0
	node.getParam("location", location);
	//Print out the new position
	for (std::vector<float>::iterator it = location.begin(); it != location.end(); ++it)
	{
		ROS_INFO("%s location %f", ros::this_node::getName().c_str(), *(it));
	}

	//Listen for updates from the world
	clockSub = node.subscribe("/sim_world/sim_world_clock", 1, &SimRobot::timeCallback, this);

}

void SimRobot::timeCallback(const std_msgs::Header::ConstPtr& msg)
{
	//ROS_INFO("%s got %f ",ros::this_node::getName().c_str(), msg->stamp.toSec());
	//TODO Update this robot's position from time elapsed and motions
}

void SimRobot::motorCallback(const tiny_robo_msgs::Motor_Vel_Cmd::ConstPtr& msg)
{
	ROS_INFO("%s got %d, %d", ros::this_node::getName().c_str(), msg->motor1, msg->motor2);
	ROS_INFO("Location (%f, %f, %f)", location[0], location[1], location[2]);
	//TODO update the motor speeds
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "sim_robot");
	ros::NodeHandle node("~");

	SimRobot sr = SimRobot(node);

	ros::spin();
}

