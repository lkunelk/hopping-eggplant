#include <cmath>
#include <cstring>
#include <iostream>

#include "consts.hpp"
#include "gazebo_msgs/LinkStates.h"
#include "geometry_msgs/Quaternion.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "tf2/LinearMath/Quaternion.h"
#include "util.hpp"

const float FLY_P = 0;  // 1.6e-4f;
const float P = 0.88f;
const float I = 0.00f;
const float D = 0.20f;
const float G = 9.8f;

// Motor Parameters
const float stallTorque = 0.2;     // Nm
const float noLoadSpeed = 1000.0;  // rad/s

// control related
ros::Publisher flywheelCommandPub;
std_msgs::Float64 commandMsg;
float robotAngPos = 0;
float robotAngVel = 0;
float flywheelVel = 0;

void linkStateCallback(const gazebo_msgs::LinkStates &msg) {
  geometry_msgs::Quaternion q = msg.pose[idxOf(msg.name, BASE_LINK_NAME)].orientation;
  geometry_msgs::Vector3 a = msg.twist[idxOf(msg.name, BASE_LINK_NAME)].angular;
  geometry_msgs::Vector3 flyA = msg.twist[idxOf(msg.name, FLY_LINK_NAME)].angular;

  // get robot state
  tf2::Quaternion myQ(q.x, q.y, q.z, q.w);
  robotAngPos = myQ.getAngle();
  robotAngVel = a.x;     // measured wrt world's x axis
  flywheelVel = flyA.x;  // measured wrt world's x axis

  if (q.x < 0)  // leaning in the clockwise direction
    robotAngPos *= -1;
}

void controlUpdate() {
  // compute and publish control command
  ROS_INFO("Robot angle: %f %f", robotAngPos, robotAngVel);
  float command = (-P * robotAngPos) + (-D * robotAngVel) + (+FLY_P * flywheelVel);
  float sign = (command > 0) - (command < 0);
  float maxTorque = fmax(0.0f, (1 - std::abs(flywheelVel) / noLoadSpeed) * stallTorque);

  command = sign * std::min(maxTorque, std::abs(command));

  commandMsg.data = command;
  flywheelCommandPub.publish(commandMsg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "orientation_control_node");
  ros::NodeHandle n;
  flywheelCommandPub = n.advertise<std_msgs::Float64>("flywheel_controller/command", 1000);
  ros::Subscriber sub = n.subscribe("gazebo/link_states", 1, linkStateCallback);

  ros::Rate rate(1000);  // Control update rate (Hz)
  while (ros::ok()) {
    controlUpdate();
    ros::spinOnce();  // process all the callbacks
    rate.sleep();
  }

  return 0;
}