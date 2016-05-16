#include <motor_translation/motor_translation.h>

MotorTranslator::MotorTranslator(ros::NodeHandle node, std::string commander)
{
	//Configure the publisher and subscriber for this node
	cmdSub = node.subscribe(commander, 1, &MotorTranslator::cmdCallback, this);
	motorPub = node.advertise<tiny_robo_msgs::Motor_Vel_Cmd>("drive_cmd", 10);
}

MotorTranslator::MotorTranslator(const geometry_msgs::Twist::ConstPtr& msg)
{
	tiny_robo_msgs::Motor_Vel_Cmd mvc = tiny_robo_msgs::Motor_Vel_Cmd();
	/* This is a REALLY STUPID pass-through. If you know what kind of robot this is driving, there
	 * is no doubt a better way to set these two commands. Simple Ackerman steering has a binary (trinary?)
	 * left/right/center, but servoed steering is smarter. Robots with one motor probably ignore even more of
	 * the twist message.
	 */
	mvc.motor1 = (int)(msg->linear.x * 128)
	mvc.motor2 = (int)(msg->rot.z * 128)
	motorPub.publish(mvc);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "basic_motor_translator");
	ros::NodeHandle node("~");

	//Configure the robot control code that this translator listens to
	std::string cmd_src;
	// Topic to listen to
	node.param < std::string > (ros::this_node::getName() + "/driver_name", cmd_src, "/default_driver/drive_cmd");

	MotorTranslator mt = MotorTranslator(node, cmd_src);

	//This node publishes every time it gets a message, so there's no need to delay
	ros::spin();
}
