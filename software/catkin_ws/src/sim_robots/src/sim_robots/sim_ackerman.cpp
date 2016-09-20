#include <sim_robots/sim_robot.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "sim_ackerman_robot");
	ros::NodeHandle node("~");

	AckermanRobot ar = AckermanRobot(node);

	ros::spin();
}
