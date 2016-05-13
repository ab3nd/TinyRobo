#include <sim_robots/sim_world.h>

SimWorld::SimWorld()
{
	//Set up the publisher

}
void SimWorld::step()
{
	/* Publish a clock message, which is just a ROS Header.
	 * The header automatically has a timestamp, which is what the sim robots
	 * need to update themselves.
	 */
	std_msgs::Header msg;
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

	SimWorld world = SimWorld();

	//TODO Subscribe to all robots updates

	ros::Rate r(100); //Update frequency for the world, in Hz
	while(ros::ok())
	{
		world.update();
		r.sleep();
	}

}
