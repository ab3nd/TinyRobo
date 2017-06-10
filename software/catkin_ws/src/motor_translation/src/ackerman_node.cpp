#include <motor_translation/motor_translation.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ackerman_motor_translator");
	ros::NodeHandle node("~");

	//Configure the robot control code that this translator listens to
	std::string cmd_src;
	// Topic to listen to
	node.param < std::string > (ros::this_node::getName() + "/driver_name", cmd_src, "/default_driver/drive_cmd");

	AckermanTranslator mt = AckermanTranslator(node, cmd_src);

	//This node publishes every time it gets a message, so there's no need to delay
	ros::spin();
}
