#ifndef SIM_WORLD_HPP_
#define SIM_WORLD_HPP_

#include <ros/ros.h>
#include <std_msgs/Header.h>

class SimWorld
{
	private:

	public:
		//Publish the states of all the robots in the world
		//TODO will eventually want a top-down image of the world like the camera image for the real arena
		ros::Publisher worldState;
		//Send out timing updates to all simulated robots subscribed to the world
		ros::Publisher worldClock;
		//Send out a clock signall to all the subscribed sim robots
		void step();
		//Receive an update from a sim robot and update it
		void update();
		SimWorld(ros::NodeHandle node);

};
#endif
