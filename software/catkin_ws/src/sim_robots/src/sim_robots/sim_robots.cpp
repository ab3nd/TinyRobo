#include <sim_robots/sim_robot.h>

//Constructor for a simulated robot
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

}

void SimRobot::motorCallback(const tiny_robo_msgs::Motor_Vel_Cmd::ConstPtr& msg)
{
	speedMotor1 = msg->motor1;
	speedMotor2 = msg->motor2;
}

/************************ Differential/Tank Robot ************************/

DifferentialRobot::DifferentialRobot(ros::NodeHandle node): SimRobot(node)
{
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

void DifferentialRobot::timeCallback(const std_msgs::Header::ConstPtr& msg)
{
	if(!hasLastTime)
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

void DifferentialRobot::motorCallback(const tiny_robo_msgs::Motor_Vel_Cmd::ConstPtr& msg)
{
	//Do nothing (calls superclass)
	 SimRobot::motorCallback(msg);
}

/************************ Holonomic(?) Spider Robot ************************/
SpiderRobot::SpiderRobot(ros::NodeHandle node): SimRobot(node)
{
	//Even big motor speed results in small motion
	//These can be different
	scaleMotor1 = 0.00005; //rotational scale
	scaleMotor2 = 0.00001; //linear motion scale
}

void SpiderRobot::timeCallback(const std_msgs::Header::ConstPtr& msg)
{
	/* These calculations are from the paper "Spider-Bots: A Low Cost Cooperative Robotics
	 * Platform, by Damien Laird, Jack Price, and Ioannis A. Raptis, but I may not have
	 * the implementation precisely right.
	 */
	if(!hasLastTime)
	{
		//We have seen a last time message
		lastTime = msg->stamp;
		hasLastTime = true;
	}
	else
	{
		//Change in time since the last tick from the world simulator
		double deltaT = (msg->stamp - lastTime).toSec();

		/* Convert motor speeds to distance traveled.
		 * The motor scales can be different, one rotates the robot and
		 * the other moves the robot in the direction it is pointed
		 */
		double rotVel = (speedMotor1 * scaleMotor1);
		double linVel = (speedMotor2 * scaleMotor2);

		//Change in distance and rotation
		double dx = (linVel * cos(rotVel))/deltaT;
		double dy = (linVel * sin(rotVel))/deltaT;
		double dw = rotVel/deltaT;

		if(linVel != 0)
		{
			//Update position
			location[0] += dx;
			location[1] += dy;
		}
		if(rotVel != 0)
		{
			//Update rotation
			location[2] += dw;
		}

		//Create the new message and send it
		geometry_msgs::Point robotLocation;
		robotLocation.x = location[0];
		robotLocation.y = location[1];
		robotLocation.z = location[2];
		geometry_msgs::Quaternion robotQuat;
		robotQuat.x = robotQuat.y = 0.0;
		robotQuat.z = robotQuat.w = dw/2.0;
		geometry_msgs::Pose outMsg;
		outMsg.position = robotLocation;
		outMsg.orientation = robotQuat;
		posPub.publish(outMsg);

		//Set last time to the new message timestamp
		lastTime = msg->stamp;
	}

}

void SpiderRobot::motorCallback(const tiny_robo_msgs::Motor_Vel_Cmd::ConstPtr& msg)
{
	//Do nothing (calls superclass)
	SimRobot::motorCallback(msg);
}

/************************ Ackerman/RC car ************************/
AckermanRobot::AckermanRobot(ros::NodeHandle node): SimRobot(node)
{
	//Even big motor speed results in small motion
	//These can be different
	scaleMotor1 = 0.00005; //rotational scale
	scaleMotor2 = 0.00001; //linear motion scale

	wheelBase = 0.002; //in meters
	wheelCircumference = 0.002;
	deadBand = 40; //In motor speed units
	steeringAngle = 0.8; //In radians
}

void AckermanRobot::timeCallback(const std_msgs::Header::ConstPtr& msg)
{
	if(!hasLastTime)
	{
		//We have seen a last time message
		lastTime = msg->stamp;
		hasLastTime = true;
	}
	else
	{
		/* Convert the steering motor signal into one of full-right, full-left, or straight ahead.
		 * The intent here is to have this robot imitate cheap toy cars, which don't have servo steering.
		 */
		float currentAngle = 0.0;
		if(abs(speedMotor1) > deadBand)
		{
			//This will be +/-1 times the (fixed) steering angle
			currentAngle = (speedMotor1/abs(speedMotor1)) * steeringAngle;
		}

		/* Convert motor speeds to distance traveled.
		 * The motor scales can be different, one rotates the robot and
		 * the other moves the robot in the direction it is pointed.
		 * Linear speed is just based on the drive motor.
		 */
		double linVel = (speedMotor2 * scaleMotor2);
		/* Rotational velocity is a little more complex, and was largely cribbed from
		 * http://www.asawicki.info/Mirror/Car%20Physics%20for%20Games/Car%20Physics%20for%20Games.html
		 */
		double rotVel = 0.0;
		if(currentAngle != 0.0){
			double turnRadius = wheelBase/sin(currentAngle);
			rotVel = linVel/turnRadius;
		}

		//Change in time since the last tick from the world simulator
		double deltaT = (msg->stamp - lastTime).toSec();

		//Calculate the change in distance and rotation and apply them
		double dx = (linVel * cos(rotVel))/deltaT;
		double dy = (linVel * sin(rotVel))/deltaT;
		double dw = rotVel/deltaT;

		if(linVel != 0)
		{
			//Update position
			location[0] += dx;
			location[1] += dy;
		}
		if(rotVel != 0)
		{
			//Update rotation
			location[2] += dw;
		}

		//Create the new message and send it
		geometry_msgs::Point robotLocation;
		robotLocation.x = location[0];
		robotLocation.y = location[1];
		robotLocation.z = location[2];
		geometry_msgs::Quaternion robotQuat;
		robotQuat.x = robotQuat.y = 0.0;
		robotQuat.z = robotQuat.w = dw/2.0;
		geometry_msgs::Pose outMsg;
		outMsg.position = robotLocation;
		outMsg.orientation = robotQuat;
		posPub.publish(outMsg);

		//Set last time to the new message timestamp
		lastTime = msg->stamp;
	}
}

void AckermanRobot::motorCallback(const tiny_robo_msgs::Motor_Vel_Cmd::ConstPtr& msg)
{
	//Do nothing (calls superclass)
	SimRobot::motorCallback(msg);
}


