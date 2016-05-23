#include <sim_robots/sim_robot.h>

#include <sim_robots/sim_robot.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "sim_differential_robot");
	ros::NodeHandle node("~");

	DifferentialRobot dr = DifferentialRobot(node);

	ros::spin();
}
