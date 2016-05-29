#ifndef SIM_ROBOT_HPP_
#define SIM_ROBOT_HPP_

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tiny_robo_msgs/Motor_Vel_Cmd.h>
#include <vector>

class SimRobot
{
	protected:
		//Publishes location and position to the world
		ros::Publisher posPub;
		//Location in world coordinates
		std::vector<float> location;
		//Orientation in rotation relative to world origin, in radians
		std::vector<float> orientation;
		ros::Subscriber clockSub;
		ros::Subscriber cmdSub;
		int speedMotor1, speedMotor2;
		ros::Time lastTime;
		bool hasLastTime;
		//Scale factors for motor speeds, determines how motor speed changes robot position
		float scaleMotor1;
		float scaleMotor2;

	public:
		//Receives time updates from the world, updates robot state
		virtual void timeCallback(const std_msgs::Header::ConstPtr& msg) = 0;
		//Receives motor speeds, updates robot state
		void motorCallback(const tiny_robo_msgs::Motor_Vel_Cmd::ConstPtr& msg);

		SimRobot(ros::NodeHandle node);
};

class AckermanRobot: public SimRobot
{
	private:
		float wheelBase;
		float wheelCircumference;
		float deadBand;
		float steeringAngle;
	public:
		void timeCallback(const std_msgs::Header::ConstPtr& msg);
		void motorCallback(const tiny_robo_msgs::Motor_Vel_Cmd::ConstPtr& msg);
		AckermanRobot(ros::NodeHandle node);
};

class DifferentialRobot: public SimRobot
{
	private:
		//Scale factors for motor speeds, determines how motor speed changes robot position
		float robotWidth;
		float wheelCircumference;
		//Doesn't have a wheelbase, differential drive robots don't have front and back wheels

	public:
		void timeCallback(const std_msgs::Header::ConstPtr& msg);
		void motorCallback(const tiny_robo_msgs::Motor_Vel_Cmd::ConstPtr& msg);
		DifferentialRobot(ros::NodeHandle node);
};

class SpiderRobot: public SimRobot
{

	public:
		void timeCallback(const std_msgs::Header::ConstPtr& msg);
		void motorCallback(const tiny_robo_msgs::Motor_Vel_Cmd::ConstPtr& msg);
		SpiderRobot(ros::NodeHandle node);
};


#endif
