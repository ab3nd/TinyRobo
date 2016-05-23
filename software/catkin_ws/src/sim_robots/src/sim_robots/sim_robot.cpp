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

	//Listen for clock tick updates from the world
	clockSub = node.subscribe("/sim_world/sim_world_clock", 1, &SimRobot::timeCallback, this);
	//We haven't seen a time callback from the world yet
	hasLastTime = false;

	//Publisher for the robot's position and orientation
	posPub = node.advertise<geometry_msgs::Pose>(ros::this_node::getName() + "/pose", 10);


	//Physical parameters of the robot
	//Even big motor speed results in small motion
	scaleMotor1 = scaleMotor2 = 0.00001;
	/* Assume the robot is 2cm across and has 2.5cm circumference wheels
	 * "Wheel circumference" in a non-wheeled robot is just a scale factor that
	 * describes the distance traveled per complete rotation of the drive motor.
	 */
	robotWidth = 0.02;
	wheelCircumference = 0.025;
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
		/* These calculations are based on the motion of the robot for the roverhawk,
		 * https://github.com/UMLRoverHawks/roboteq_driver/blob/master/mdc2250/src/quad_drive_node.cpp
		 * which was 4-wheel differential drive.
		 *
		 * TODO subclass sim_robot to have different conversions from motor speeds to
		 * change in position of the robot. So far, the cases are:
		 *  - Differential drive (this method)
		 *  - Holonomic spider
		 *  - Crappy ackerman (toy RC cars)
		 */

		//Change in time since the last tick from the world simulator
		double deltaT = (msg->stamp - lastTime).toSec();

		/* Convert motor speeds to for each wheel from motor speed and wheel size.
		 * Note that scaleMotor1 == scaleMotor2, unless the motors are different, which
		 * would be pretty bad design for a differential drive robot
		 */
		double lDist = (speedMotor1/wheelCircumference) * scaleMotor1;
		double rDist = (speedMotor2/wheelCircumference) * scaleMotor2;

		double v = (lDist + rDist)/2.0; //Average motion contributed by a wheel
		double w = (rDist - lDist)/robotWidth; //rotation

		//Change in distance and rotation
		double dv = v/deltaT;
		double dw = w/deltaT;

		//TODO For a flying robot, this should include Z
		double x, y;
		if(v != 0)
		{
			//Distance traveled in x and y
			x = cos(w) * v;
			y = -sin(w) * v;
			//Update position
			location[0] += (cos(location[2]) * x - sin(location[2]) * y);
			location[1] += (sin(location[2]) * x + cos(location[2]) * y);
		}
		if(w != 0)
		{
			//Update rotation
			location[2] += w;
		}

		//Create the new message and send it
		geometry_msgs::Point robotLocation;
		robotLocation.x = location[0];
		robotLocation.y = location[1];
		robotLocation.z = location[2];
		geometry_msgs::Quaternion robotQuat;
		robotQuat.x = robotQuat.y = 0.0;
		robotQuat.z = robotQuat.w = w/2.0;
		geometry_msgs::Pose outMsg;
		outMsg.position = robotLocation;
		outMsg.orientation = robotQuat;
		posPub.publish(outMsg);

		//Set last time to the new message timestamp
		lastTime = msg->stamp;
	}
}

void SimRobot::motorCallback(const tiny_robo_msgs::Motor_Vel_Cmd::ConstPtr& msg)
{
	//ROS_INFO("%s got %d, %d", ros::this_node::getName().c_str(), msg->motor1, msg->motor2);
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

