
#include <drive_logic/random_driver.h>

#define PI 3.1415926

RandomDriver::RandomDriver(ros::NodeHandle node)
{
	twistPub = node.advertise<geometry_msgs::Twist>("drive_cmd", 10);
	std::srand(ros::Time::now().toSec());
}

float RandomDriver::getVel()
{
	//Range is m/sec
	return getRand(0.1);
}

float RandomDriver::getTurn()
{
	//Range is r/sec
	return getRand(PI/2.0);
}

//Return a value in +/- range
float RandomDriver::getRand(float range)
{
	return ((2*(std::rand()/((float)RAND_MAX)))-1) * range;
}

void RandomDriver::drive()
{
	//Generate a twist message
	geometry_msgs::Twist t = geometry_msgs::Twist();
	//I assume these are M/sec
	t.linear.x = getVel();  //forward motion
	t.linear.y = getVel();
	t.linear.z = getVel();
	//And these should be radians/sec
	t.angular.x = getTurn();
	t.angular.y = getTurn();
	t.angular.z = getTurn(); //heading turning
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
