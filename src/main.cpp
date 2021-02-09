#include "ros/ros.h"
#include "gazebo_msgs/LinkStates.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Quaternion.h"
#include "tf2/LinearMath/Quaternion.h"

#include <iostream>
#include <cmath>
#include <cstring>

// using namespace gazebo;

const float P{1.0};
const float I{0.05};
const float D{0};
const float G = -9.8f;

const float K_ARM_SPRING = 385.0f; // N/m
// T0 @300g ~ 0.175s

// 2% settling time
const float T_SETTLE = 0.2f;
// derivation:
// k = G * R_arm * M_motor - P ( = -0.775)
// Itot ~= 0.00125 kg-m^2
// zeta = D / (2*sqrt(k*Itot)) ( = 0.804)
// w0 = sqrt(k / Itot) ( = 24.92)
// -ln(0.02 * sqrt(1 - zeta ** 2)) / zeta / w0 = 0.195s

// Motor Parameters
const float stallTorque = 0.733;  // Nm
const float noLoadSpeed = 1235.7;  // rad/s

ros::Publisher commandPub;
ros::Publisher posPub, velPub, flywheelVelPub, armSpringPub;
std_msgs::Float64 commandMsg;
// std_msgs::Float64 posMsg;
geometry_msgs::Quaternion posMsg;
std_msgs::Float64 velMsg;
std_msgs::Float64 flywheelVelMsg;
std_msgs::Float64 armSpringMsg;

sensor_msgs::JointState last_joint_states;
bool last_joint_states_valid = false;
void joint_cb(const sensor_msgs::JointState& msg) {
  last_joint_states = msg;
  last_joint_states_valid = true;
}

bool last_collided = false;
void base_contact_cb(const std_msgs::Bool msg) {
  last_collided = msg.data;
}

float sgn(float x) {
  return x > 0 ? 1.0f : -1.0f;
}

float last_land_angle = 0.0f;
const float P_TAKEOFF = 1.0f; // takeoff angle P controller
const std::string
  BASE_LINK_NAME("robot::base_link"),
  ARM_LINK_NAME("robot::arm_link"),
  FLY_LINK_NAME("robot::flywheel_link"),
  TRACK_LINK_NAME("tracker::tracking_dot"),
  
  PRISM_JOINT_NAME("arm_prism"),
  SPRING_JOINT_NAME("arm_spring"),
  FLY_JOINT_NAME("arm_to_flywheel");

int idxOf(const std::vector<std::string> a, const std::string& b) {
  for(int i = 0; i < a.size(); i++) {
    if(a[i].compare(b) == 0)
      return i;
  }
  return -1;
}
void chatterCallback(const gazebo_msgs::LinkStates& msg)
{
  geometry_msgs::Quaternion q = msg.pose[idxOf(msg.name, ARM_LINK_NAME)].orientation;
  geometry_msgs::Vector3 a = msg.twist[idxOf(msg.name, ARM_LINK_NAME)].angular;
  geometry_msgs::Vector3 flyA = msg.twist[idxOf(msg.name, FLY_LINK_NAME)].angular;
  
  // convert to radians
  tf2::Quaternion myQ(q.x, q.y, q.z, q.w);
  float position = myQ.getAngle();
  // auto fpos = msg.pose[idxOf(msg.name, FLY_LINK_NAME)].position, bpos = msg.pose[idxOf(msg.name, BASE_LINK_NAME)].position;
  // tf2::Vector3 dpos = (tf2::Vector3(fpos.x, fpos.y, fpos.z) - tf2::Vector3(bpos.x, bpos.y, bpos.z)).normalized();
  // float position = acos(dpos.getZ());
  float velocity = a.x;  // measured wrt world's x axis
  float flywheelVel = flyA.x;  // measured wrt world's x axis
  
  
  if (q.x < 0)  // leaning in the clockwise direction
  {
    position *= -1;
  }
  
  if(!last_collided) {
    geometry_msgs::Vector3 v = msg.twist[idxOf(msg.name, BASE_LINK_NAME)].linear;
    geometry_msgs::Point p = msg.pose[idxOf(msg.name, BASE_LINK_NAME)].position;
    float vh = v.y; // sqrt(v.x * v.x = v.y * v.y); // although really only the rotation in the direction we control matters
    float tf = p.z + v.z / G;
    float vzf = -(fabs(v.z) + sqrt(fabs(G * p.z) * 2)); // COE
    
    last_land_angle = atan2(vzf, vh) + M_PI_2; // rotate relative to vertical
    // ROS_INFO("SET %.3f, %.3f, %.3f", vh, vzf, last_land_angle);
  }
  
  // if(last_collided)
  //   ROS_INFO("%.3f", last_joint_states.velocity[1]);
  
  // if(last_collided && last_joint_states_valid && last_joint_states.velocity[idxOf(last_joint_states.name, SPRING_JOINT_NAME)] > 0) {
  //   // second joint expanding: point in target direction
  //   float delta = msg.pose[idxOf(msg.name, TRACK_LINK_NAME)].position.y - msg.pose[idxOf(msg.name, FLY_LINK_NAME)].position.y;
  //   position += sgn(delta) * fmin(fabs(delta) / P_TAKEOFF, M_PI_4);
  // }
  // else {
  //   position -= last_land_angle;
  // }
  if(last_joint_states_valid)
    ROS_INFO("SET %.3f, %.3f, %.3f", position, last_land_angle, last_joint_states.velocity[idxOf(last_joint_states.name, FLY_JOINT_NAME)]); 
  
  // compute and publish control command
  float command = P * position + I * velocity;
  float sign = (command > 0) - (command < 0);
  float maxTorque = fmax(0.0f, (1 - std::abs(flywheelVel) / noLoadSpeed) * stallTorque);
  
  command = sign * std::min(maxTorque, std::abs(command));
  
  commandMsg.data = command;
  commandPub.publish(commandMsg);
  
  armSpringMsg.data = 0.0f;
  armSpringPub.publish(armSpringMsg);
  
  // Publish pendulum state for debug
  // posMsg.data = msg.pose[3].position.z; // position;
  posMsg.x = msg.pose[3].position.z;
  posMsg.y = msg.pose[1].position.z;
  posMsg.z = msg.twist[3].linear.z;
  posMsg.w = msg.twist[1].linear.z;
  velMsg.data = velocity;
  flywheelVelMsg.data = flywheelVel / 1000.0; // so it fits nicely on graph
  
  posPub.publish(posMsg);
  velPub.publish(velMsg);
  flywheelVelPub.publish(flywheelVelMsg);
  
  // ROS_INFO("ArmPos, ArmVel, FlyVel, Command: [%f, %f, %f, %f]", position, velocity, flywheelVel, command);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_node");

  ros::NodeHandle n;
  
  armSpringPub = n.advertise<std_msgs::Float64>("arm_spring/command", 1000);
  commandPub = n.advertise<std_msgs::Float64>("flywheel_controller/command", 1000);
  posPub = n.advertise<geometry_msgs::Quaternion>("pendulum_pos", 1000);
  velPub = n.advertise<std_msgs::Float64>("pendulum_vel", 1000);
  flywheelVelPub = n.advertise<std_msgs::Float64>("pendulum_flywheel_vel", 1000);

  ros::Subscriber joint_sub = n.subscribe("/joint_states", 1000, joint_cb);
  ros::Subscriber sub = n.subscribe("gazebo/link_states", 1000, chatterCallback);
  ros::Subscriber sub_base_contact = n.subscribe("base_contact", 1000, base_contact_cb);
  
  ros::spin();
  
  // Make sure to shut everything down.
  return 0;
}
