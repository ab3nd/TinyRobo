#ifndef SIM_WORLD_HPP_
#define SIM_WORLD_HPP_

#include <ros/ros.h>

class SimWorld
{
	public:
		//Publish the states of all the robots in the world
		//TODO will eventually want a top-down image of the world like the camera image for the real arena
		ros::Publisher worldState;
	private:
};
#endif
