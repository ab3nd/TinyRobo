#ifndef MOTOR_TRANSLATION_H_
#define MOTOR_TRANSLATION_H_

#include <ros/ros.h>
#include <tiny_robo_msgs/Motor_Vel_Cmd.h>
#include <geometry_msgs/Twist.h>
#include <vector>

class MotorTranslator
{
	protected:
		//Publishes motor speeds based on twist messages
		ros::Publisher motorPub;
		//Listens for twist messages to convert to motor drive messages
		ros::Subscriber cmdSub;
	public:
		//Receives twists and converts them to motor drive signals
		virtual void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg);
		MotorTranslator(ros::NodeHandle node, std::string driver);
};
#endif

class HolonomicTranslator: public MotorTranslator
{
	//If I don't override anything, it's a MotorTranslator, right?
	public:
		HolonomicTranslator(ros::NodeHandle node, std::string driver);
		void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg);
};
