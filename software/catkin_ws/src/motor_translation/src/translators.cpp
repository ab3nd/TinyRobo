#include <motor_translation/motor_translation.h>

MotorTranslator::MotorTranslator(ros::NodeHandle node, std::string commander)
{
	//Configure the publisher and subscriber for this node
	cmdSub = node.subscribe(commander, 1, &MotorTranslator::cmdCallback, this);
	motorPub = node.advertise<tiny_robo_msgs::Motor_Vel_Cmd>("drive_cmd", 10);
}

//For now this is just a call to the superclass cons
HolonomicTranslator::HolonomicTranslator(ros::NodeHandle node, std::string driver):MotorTranslator(node, driver){}

void HolonomicTranslator::cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	tiny_robo_msgs::Motor_Vel_Cmd mvc = tiny_robo_msgs::Motor_Vel_Cmd();
	/* This is desgined for the spiderbots, which have one motor control rotation and the other motor
	 * control motion of the legs. Other holonomic platforms might work differently. */
	mvc.motor1 = (int)(msg->linear.x * 128);
	mvc.motor2 = (int)(msg->angular.z * 128);
	motorPub.publish(mvc);
}

AckermanTranslator::AckermanTranslator(ros::NodeHandle node, std::string driver):MotorTranslator(node, driver)
{
	deadBand = 0.2;
}

void AckermanTranslator::cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	tiny_robo_msgs::Motor_Vel_Cmd mvc = tiny_robo_msgs::Motor_Vel_Cmd();
	/* One motor controls the steering, the other controls the velocity.
	 * Since most toys don't have servo steering, the "speed" of the steering motor
	 * is 100% one way or the other.
	 */
	if(msg->angular.z < deadBand)
	{
		//Not enough steering signal to change direction
		mvc.motor2 = 0;
	}
	else
	{
		//Turn is left or right
		if (msg->angular.z > 0)
		{
			mvc.motor2 = 127;
		}
		else
		{
			mvc.motor2 = -127;
		}
	}
	//Speed of drive motor
	mvc.motor1 = (int)(msg->linear.x * 127);
	motorPub.publish(mvc);
}

//For now this is just a call to the superclass cons
DifferentialTranslator::DifferentialTranslator(ros::NodeHandle node, std::string driver):MotorTranslator(node, driver){}

void DifferentialTranslator::cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	tiny_robo_msgs::Motor_Vel_Cmd mvc = tiny_robo_msgs::Motor_Vel_Cmd();
	/* This is a sort of mixing controller, which combines the linear and rotational velocites to produce
	 * drive signals for each motor. This is kind of a draft, so it may have bugs. The algorithm is based on
	 * the one in the ros diff_drive_controller */
	//float wheelSep = 0.01; //wheel seperation in meters
	//float wheelRad = 0.02; //wheel radius in meters
	//mvc.motor1 = (int)(((msg->linear.x - msg->angular.z * wheelSep / 2.0) / wheelRad) * 128);
	//mvc.motor2 = (int)(((msg->linear.x + msg->angular.z * wheelSep / 2.0) / wheelRad) * 128);
	//Probably don't need wheel radius and seperation because the output is PWM frequencies, not wheel-edge-velocities
	mvc.motor1 = (int)(((msg->linear.x + msg->angular.z)/2) * 127);
	mvc.motor2 = (int)(((msg->linear.x - msg->angular.z)/2) * 127);

	motorPub.publish(mvc);
}


