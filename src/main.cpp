#include "ros/ros.h"
#include "gazebo_msgs/LinkStates.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Quaternion.h"
#include "tf2/LinearMath/Quaternion.h"
#include "gazebo_msgs/ContactState.h"

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <iostream>
#include <cmath>
#include <cstring>

// using namespace gazebo;

const float P{1.0};
const float I{0.05};
const float D{0};
const float G = -9.8f;

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

#define NUM_CONTACT_BUF 32
bool contact_buffer[NUM_CONTACT_BUF] = { 0 };
uint8_t contact_buffer_idx = 0;

const std::string BASE_LINK_NAME("robot::base_link::base_link_fixed_joint_lump__base_collision_collision");
const std::string WORLD_LINK_NAME("ground_plane::link::collision");

bool collided_() {
  uint8_t j = 0;
  for(uint8_t i = 0; i < NUM_CONTACT_BUF; i++, j += contact_buffer[(contact_buffer_idx + i) & (NUM_CONTACT_BUF - 1)]);
  return j > 1;
}

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
  
  if(!collided_()) {
    geometry_msgs::Vector3 v = msg.twist[1].linear;
    geometry_msgs::Point p = msg.pose[1].position;
    float vh = v.y; // sqrt(v.x * v.x = v.y * v.y); // although really only the rotation in the direction we control matters
    float tf = p.z + v.z / G;
    float vzf = -(fabs(v.z) + sqrt(fabs(G * p.z) * 2)); // COE
    
    float delta = atan2(vzf, vh) + M_PI_2; // rotate relative to vertical
    position -= delta;
    ROS_INFO("SET %.3f, %.3f, %.3f", vh, vzf, delta);
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
  
  // ROS_INFO("ArmPos, ArmVel, FlyVel, Command: [%f, %f, %f, %f]", position, velocity, flywheelVel, command);
}

uint32_t contact_idx = 0;
gazebo::transport::SubscriberPtr sub; // what. so if this isn't scoped the subscription just gets garbage-collected? Eugh. How?!
void contactCallback(ConstContactsPtr &msg) {
  bool* contact_ = &contact_buffer[(++contact_idx) & (NUM_CONTACT_BUF - 1)];
  for(uint32_t i = 0; i < msg->contact_size(); i++) {
    auto contact = msg->contact(i);
    if((BASE_LINK_NAME.compare(contact.collision1()) == 0 && WORLD_LINK_NAME.compare(contact.collision2()) == 0) ||
      (WORLD_LINK_NAME.compare(contact.collision1()) == 0 && BASE_LINK_NAME.compare(contact.collision2()) == 0)) {
      // collision with base link: usual physics
      ROS_INFO("CONTACT %d %ds", contact_idx++, msg->time().nsec());
      *contact_ = true;
      return;
    }
  }
  *contact_ = false;
} 

void gazebo_init(int _argc, char **_argv) {
  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Listen to Gazebo world_stats topic
  sub = node->Subscribe("~/physics/contacts", contactCallback);
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
  
  gazebo_init(argc, argv);
  
  ros::spin();
  
  // Make sure to shut everything down.
  gazebo::client::shutdown();
  return 0;
}
