#include <sim_robots/sim_world.h>

SimWorld::SimWorld(ros::NodeHandle node)
{
	//Set up the publisher
	worldClock = node.advertise<std_msgs::Header>("sim_world_clock", 10);

	//Set up the size of the world. This is used for making images as well as coordinate stuff, so is in pixels
	worldSizeX = 640;
	worldSizeY = 480;

}

void SimWorld::renderWorld()
{
	//Create an image and publish it
}

void SimWorld::step()
{
	/* Publish a clock message, which is just a ROS Header.
	 * The header automatically has a timestamp, which is what the sim robots
	 * need to update themselves.
	 */
	std_msgs::Header msg;
	msg.stamp = ros::Time::now();
	worldClock.publish(msg);
}

void SimWorld::update()
{
	//For now, do nothing
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "sim_world");
	ros::NodeHandle node("~");

	SimWorld world = SimWorld(node);

	//TODO Subscribe to all robots updates

	ros::Rate r(100); //Update frequency for the world, in Hz
	while(ros::ok())
	{
		world.step();
		r.sleep();
	}

}
