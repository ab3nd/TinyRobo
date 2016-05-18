#include <sim_robots/sim_robot.h>

SimRobot::SimRobot(ros::NodeHandle node)
{
	//Attempt to get the driver parameter to determine which driver this robot listens to
	std::string driver;
	// Topic to listen to
	node.param < std::string > (ros::this_node::getName() + "/translator", driver, "/default_driver/drive_cmd");
	//Listen for motion commands to this robot
	cmdSub = node.subscribe(driver + "/drive_cmd", 1, &SimRobot::motorCallback, this);

	//Attempt to get the initial position of the robot within the world
	location = std::vector<float>(3, 0); //three ints with the value 0
	node.getParam("location", location);

	//Initial motor speeds are zero
	speedMotor1 = speedMotor2 = 0;
	//Listen for updates from the world
	clockSub = node.subscribe("/sim_world/sim_world_clock", 1, &SimRobot::timeCallback, this);

	//We haven't seen a time callback from the world yet
	hasLastTime = false;

	//Even big motor speed results in small motion
	scaleMotor1 = scaleMotor2 = 0.00001;
}

void SimRobot::timeCallback(const std_msgs::Header::ConstPtr& msg)
{
	if(hasLastTime)
	{
		//We have seen a last time message
		lastTime = msg->stamp;
		hasLastTime = true;
	}
	else
	{
		/* Motor rotation distance is motor velocity divided by the amount of elapsed time.
		 * Note that this is assuming a unit-diameter wheel, we can add wheel diameter later.
		 * TODO This is a very simple uniwheel robot, where velocity of the motor directly becomes
		 * x-position change of the robot, and so it just demonstrates that things are being
		 * updated properly.
		 */
		//ROS_INFO("Pos change %f", (float)(speedMotor1 * scaleMotor1)/(msg->stamp - lastTime).toSec());
		location[0] += (float)((speedMotor1)/(msg->stamp - lastTime).toSec()) * scaleMotor1;

		//Set last time to the new message
		lastTime = msg->stamp;
	}
	//ROS_INFO("%s got %f ",ros::this_node::getName().c_str(), msg->stamp.toSec());

}

void SimRobot::motorCallback(const tiny_robo_msgs::Motor_Vel_Cmd::ConstPtr& msg)
{
	ROS_INFO("%s got %d, %d", ros::this_node::getName().c_str(), msg->motor1, msg->motor2);
	speedMotor1 = msg->motor1;
	speedMotor2 = msg->motor2;

	//Print out the new position
	for (std::vector<float>::iterator it = location.begin(); it != location.end(); ++it)
	{
		ROS_INFO("%s location %f", ros::this_node::getName().c_str(), *(it));
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "sim_robot");
	ros::NodeHandle node("~");

	SimRobot sr = SimRobot(node);

	ros::spin();
}

