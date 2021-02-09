#include "ros/ros.h"
#include "gazebo_msgs/LinkStates.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Quaternion.h"
#include "tf2/LinearMath/Quaternion.h"

#include <iostream>
#include <cmath>
#include <cstring>

const float P = 1.00f;
const float I = 0.05f;
const float D = 0.00f;
const float G = -9.8f;

// Motor Parameters
const float stallTorque = 0.733;  // Nm
const float noLoadSpeed = 1235.7;  // rad/s

// control related
ros::Publisher flywheelCommandPub;
std_msgs::Float64 commandMsg;

// debug related
ros::Publisher posPub;
ros::Publisher velPub;
ros::Publisher flywheelVelPub;
ros::Publisher armSpringPub;
geometry_msgs::Quaternion posMsg;
std_msgs::Float64 velMsg;
std_msgs::Float64 flywheelVelMsg;
std_msgs::Float64 armSpringMsg;

bool lastCollided = false;
  
void linkStateCallback(const gazebo_msgs::LinkStates& msg) {
  geometry_msgs::Quaternion q = msg.pose[3].orientation;  // arm pos
  geometry_msgs::Vector3 a = msg.twist[3].angular;  // arm vel
  geometry_msgs::Vector3 flyA = msg.twist[4].angular;  // flywheel vel
  
  // get robot state
  tf2::Quaternion myQ(q.x, q.y, q.z, q.w);
  float armAnglPos = myQ.getAngle();
  float armAnglVel = a.x;  // measured wrt world's x axis
  float flywheelVel = flyA.x;  // measured wrt world's x axis
  
  if (q.x < 0)  // leaning in the clockwise direction
    armAnglPos *= -1;
  
  if(!lastCollided) {
    geometry_msgs::Vector3 v = msg.twist[1].linear;
    geometry_msgs::Point p = msg.pose[1].position;
    float vh = v.y;
    float vzf = -(fabs(v.z) + sqrt(fabs(G * p.z) * 2)); // COE
    
    float delta = atan2(vzf, vh) + M_PI_2; // rotate relative to vertical
    armAnglPos -= delta;
    ROS_INFO("SET %.3f, %.3f, %.3f", vh, vzf, delta);
  }
  
  // compute and publish control command
  float command = P * armAnglPos + I * armAnglVel;
  float sign = (command > 0) - (command < 0);
  float maxTorque = (1 - std::abs(flywheelVel) / noLoadSpeed) * stallTorque;
  
  command = sign * std::min(maxTorque, std::abs(command));
  
  commandMsg.data = command;
  flywheelCommandPub.publish(commandMsg);
  
  // send 0 and let PID controller emulate spring
  armSpringMsg.data = 0.0f;
  armSpringPub.publish(armSpringMsg);
  
  // Publish pendulum state for debug
  posMsg.x = msg.pose[3].position.z;
  posMsg.y = msg.pose[1].position.z;
  posMsg.z = msg.twist[3].linear.z;
  posMsg.w = msg.twist[1].linear.z;
  velMsg.data = armAnglVel;
  flywheelVelMsg.data = flywheelVel / 1000.0; // so it fits nicely on graph
  
  posPub.publish(posMsg);
  velPub.publish(velMsg);
  flywheelVelPub.publish(flywheelVelMsg);
}

void baseContactCallback(const std_msgs::Bool msg) {
  lastCollided = msg.data;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "my_node");

  ros::NodeHandle n;
  
  armSpringPub       = n.advertise<std_msgs::Float64>("arm_spring/command", 1000);
  flywheelCommandPub = n.advertise<std_msgs::Float64>("flywheel_controller/command", 1000);
  velPub             = n.advertise<std_msgs::Float64>("pendulum_vel", 1000);
  flywheelVelPub     = n.advertise<std_msgs::Float64>("pendulum_flywheel_vel", 1000);
  posPub             = n.advertise<geometry_msgs::Quaternion>("pendulum_pos", 1000);

  ros::Subscriber sub = n.subscribe("gazebo/link_states", 1000, linkStateCallback);
  ros::Subscriber subBaseContact = n.subscribe("base_contact", 1000, baseContactCallback);
  
  ros::spin();
  
  return 0;
}
