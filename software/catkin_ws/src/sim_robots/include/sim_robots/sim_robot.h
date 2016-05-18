#ifndef SIM_ROBOT_HPP_
#define SIM_ROBOT_HPP_

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <tiny_robo_msgs/Motor_Vel_Cmd.h>
#include <vector>

class SimRobot
{
	private:
		//Publishes location and position to the world
		ros::Publisher posPub;
		//Location in world coordinates
		std::vector<float> location;
		//Orientation in rotation relative to world origin, in radians
		std::vector<float> orientation;
		//Scale factors for motor speeds, determines how motor speed changes robot position
		float scaleMotor1;
		float scaleMotor2;
		ros::Subscriber clockSub;
		ros::Subscriber cmdSub;
		int speedMotor1, speedMotor2;
		ros::Time lastTime;
		bool hasLastTime;

	public:
		//Receives time updates from the world, updates robot state
		void timeCallback(const std_msgs::Header::ConstPtr& msg);
		//Receives motor speeds, updates robot state
		void motorCallback(const tiny_robo_msgs::Motor_Vel_Cmd::ConstPtr& msg);

		SimRobot(ros::NodeHandle node);
};
#endif
