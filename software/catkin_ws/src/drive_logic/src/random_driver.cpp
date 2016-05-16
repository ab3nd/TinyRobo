
#include <drive_logic/random_driver.h>

#define PI 3.1415926

RandomDriver::RandomDriver(ros::NodeHandle node)
{
	twistPub = node.advertise<geometry_msgs::Twist>("drive_cmd", 10);
	std::srand(ros::Time::now().toSec());
}

void RandomDriver::drive()
{
	//Generate a twist message
	geometry_msgs::Twist t = geometry_msgs::Twist();
	//I assume these are M/sec
	t.linear.x = (2*((std::rand()/((float)RAND_MAX)) - 1) * 0.1);  //forward motion
	t.linear.y = (2*((std::rand()/((float)RAND_MAX)) - 1) * 0.1);
	t.linear.z = (2*((std::rand()/((float)RAND_MAX)) - 1) * 0.1);
	//And these should be radians/sec
	t.angular.x = (2*((std::rand()/((float)RAND_MAX)) - 1) * (PI/2.0));
	t.angular.y = (2*((std::rand()/((float)RAND_MAX)) - 1) * (PI/2.0));
	t.angular.z = (2*((std::rand()/((float)RAND_MAX)) - 1) * (PI/2.0)); //heading turning
	//Publish it
	twistPub.publish(t);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "random_driver");
	ros::NodeHandle node("~");

	RandomDriver rd = RandomDriver(node);

	ros::Rate r(0.5); //Only change robot motion every two seconds
	while(ros::ok())
	{
		rd.drive();
		r.sleep();
	}

}
