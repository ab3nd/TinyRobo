#ifndef SIM_ROBOT_HPP_
#define SIM_ROBOT_HPP_

#include <ros/ros.h>
#include <array>

class SimRobot
{
	private:
		//Publishes location and position to the world
		ros::Publisher posPub;
		//Location in world coordinates
		std::array<float, 3> location;
		//Orientation in rotation relative to world origin, in radians
		std::array<float, 3> orientation;
		//Scale factors for motor speeds, determines how motor speed changes robot position
		float scaleMotor1;
		float scaleMotor2;
		ros::Subscriber clockSub;

	public:
		//Receives time updates from the world, updates robot state
		void timeCallback();
		//Receives motor speeds, updates robot state
		void motorCallback();
};
#endif
