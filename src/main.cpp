#include "ros/ros.h"
#include "gazebo_msgs/LinkStates.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Quaternion.h"
#include "tf2/LinearMath/Quaternion.h"

#include <iostream>
#include <cmath>

const float P{1.0};
const float I{0.1};
const float D{0};

// Motor Parameters
const float stallTorque = 0.733;  // Nm
const float noLoadSpeed = 1235.7;  // rad/s

ros::Publisher commandPub;
ros::Publisher posPub, velPub, flywheelVelPub, armPrismPub;
std_msgs::Float64 commandMsg;
std_msgs::Float64 posMsg;
std_msgs::Float64 velMsg;
std_msgs::Float64 flywheelVelMsg;
std_msgs::Float64 armPrismMsg;


void chatterCallback(const gazebo_msgs::LinkStates& msg)
{
  geometry_msgs::Quaternion q = msg.pose[2].orientation;
  geometry_msgs::Vector3 a = msg.twist[2].angular;
  geometry_msgs::Vector3 flyA = msg.twist[3].angular;
  
  // convert to radians
  tf2::Quaternion myQ(q.x, q.y, q.z, q.w);
  float position = myQ.getAngle();
  float velocity = a.x;  // measured wrt world's x axis
  float flywheelVel = flyA.x;  // measured wrt world's x axis
  
  
  if (q.x < 0)  // leaning in the clockwise direction
  {
    position *= -1;
  }
  
  // compute and publish control command
  float command = P * position + I * velocity;
  float sign = (command > 0) - (command < 0);
  float maxTorque = (1 - std::abs(flywheelVel) / noLoadSpeed) * stallTorque;
  
  command = sign * std::min(maxTorque, std::abs(command));
  
  commandMsg.data = command;
  commandPub.publish(commandMsg);
  
  // Publish pendulum state for debug
  posMsg.data = position;
  velMsg.data = velocity;
  flywheelVelMsg.data = flywheelVel / 1000.0; // so it fits nicely on graph
  
  posPub.publish(posMsg);
  velPub.publish(velMsg);
  flywheelVelPub.publish(flywheelVelMsg);
  
  // we should just be able to one-and-done this but I don't know when the controller loads so best to keep emitting
  armPrismMsg.data = 0.0;
  armPrismPub.publish(armPrismMsg);
  
  ROS_INFO("ArmPos, ArmVel, FlyVel, Command: [%f, %f, %f, %f]", position, velocity, flywheelVel, command);
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "my_node");

  ros::NodeHandle n;
  
  armPrismPub = n.advertise<std_msgs::Float64>("arm_controller/command", 1000);
  commandPub = n.advertise<std_msgs::Float64>("flywheel_controller/command", 1000);
  posPub = n.advertise<std_msgs::Float64>("pendulum_pos", 1000);
  velPub = n.advertise<std_msgs::Float64>("pendulum_vel", 1000);
  flywheelVelPub = n.advertise<std_msgs::Float64>("pendulum_flywheel_vel", 1000);

  ros::Subscriber sub = n.subscribe("gazebo/link_states", 1000, chatterCallback);
  
  ros::spin();
  
  return 0;
}
