// ROS Stuff #include "ros/ros.h"
#include "argos_bridge/Proximity.h"
#include "argos_bridge/ProximityList.h"

/* Include the controller definition */
#include "tiny_robo_ros.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

#include <iostream>
#include <sstream>

#include <ros/callback_queue.h>

using namespace std;
using namespace argos_bridge;

// Initialize ROS node.  There will be only one ROS node no matter how many robots are created in
// ARGoS.  However, we will have one instance of the CTinyRobo class for each ARGoS robot.
ros::NodeHandle* initROS() {
  int argc = 0;
  char *argv = (char *) "";
  ros::init(argc, &argv, "argos_bridge");
  return new ros::NodeHandle();
}

ros::NodeHandle* CTinyRobo::nodeHandle = initROS();

/****************************************/
/****************************************/

CTinyRobo::CTinyRobo() :
  m_pcWheels(NULL),
  m_pcProximity(NULL),
  m_pcPosition(NULL),
  stopWithoutSubscriberCount(10),
  stepsSinceCallback(0),
  leftSpeed(0),
  rightSpeed(0)
{
    //Deliberately empty cons body
}

void CTinyRobo::Init(TConfigurationNode& t_node) {
  // Create the topics to publish
  stringstream positionTopic, proximityTopic;
  positionTopic << "/" << GetId() << "/position";
  proximityTopic << "/" << GetId() << "/proximity";
  proximityPub = nodeHandle->advertise<ProximityList>(proximityTopic.str(), 1);
  posePub = nodeHandle->advertise<geometry_msgs::Pose>(positionTopic.str(), 1);

  // Create the subscribers
  stringstream cmdVelTopic;//, gripperTopic;
  cmdVelTopic << "/" << GetId() << "/cmd_vel";
  cmdVelSub = nodeHandle->subscribe(cmdVelTopic.str(), 1, &CTinyRobo::cmdVelCallback, this);

  // Get sensor/actuator handles
  m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
  m_pcProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
  m_pcPosition = GetSensor<CCI_PositioningSensor>("positioning");

  /*
   * Parse the configuration file
   *
   * The user defines this part. Here, the algorithm accepts three
   * parameters and it's nice to put them in the config file so we don't
   * have to recompile if we want to try other settings.
   */
  GetNodeAttributeOrDefault(t_node, "stopWithoutSubscriberCount", stopWithoutSubscriberCount, stopWithoutSubscriberCount);
}


void CTinyRobo::ControlStep() {

  /* Get reading from position sensor */
  const CCI_PositioningSensor::SReading& posReading = m_pcPosition->GetReading();
  geometry_msgs::Pose Pose;
  Pose.position.x = posReading.Position.GetX();
  Pose.position.y = posReading.Position.GetY();
  Pose.position.z = posReading.Position.GetZ();
  Pose.orientation.x = posReading.Orientation.GetX();
  Pose.orientation.y = posReading.Orientation.GetY();
  Pose.orientation.z = posReading.Orientation.GetZ();
  Pose.orientation.w = posReading.Orientation.GetW();

  posePub.publish(Pose);

  /* Get readings from proximity sensor */
  const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
  ProximityList proxList;
  proxList.n = tProxReads.size();
  for (size_t i = 0; i < proxList.n; ++i) {
    Proximity prox;
    prox.value = tProxReads[i].Value;
    prox.angle = tProxReads[i].Angle.GetValue();
    proxList.proximities.push_back(prox);
    //cout << GetId() << ": value: " << prox.value << ": angle: " << prox.angle << endl;
  }

  proximityPub.publish(proxList);

  // Wait for any callbacks to be called.
  ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));

  // If we haven't heard from the subscriber in a while, set the speed to zero.
  if (stepsSinceCallback > stopWithoutSubscriberCount) {
    leftSpeed = 0;
    rightSpeed = 0;
  } else {
    stepsSinceCallback++;
  }

  m_pcWheels->SetLinearVelocity(leftSpeed, rightSpeed);
}

void CTinyRobo::cmdVelCallback(const geometry_msgs::Twist& twist) {
  cout << "cmdVelCallback: " << GetId() << endl;

  Real v = twist.linear.x;  // Forward speed
  Real w = twist.angular.z; // Rotational speed

  // Use the kinematics of a differential-drive robot to derive the left
  // and right wheel speeds.
  leftSpeed = (v - HALF_BASELINE * w) / WHEEL_RADIUS;
  rightSpeed = (v + HALF_BASELINE * w) / WHEEL_RADIUS;

  stepsSinceCallback = 0;
}

/*
* This statement notifies ARGoS of the existence of the controller.
* It binds the class passed as first argument to the string passed as
* second argument.
* The string is then usable in the configuration file to refer to this
* controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CTinyRobo, "tiny_robo_ros")
