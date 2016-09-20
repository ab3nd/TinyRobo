#include <sim_robots/sim_robot.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "sim_spider_robot");
	ros::NodeHandle node("~");

	SpiderRobot sr = SpiderRobot(node);

	ros::spin();
}
