#include "ros/ros.h"
#include "gazebo_msgs/ContactState.h"
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include "std_msgs/Bool.h"

#define NUM_CONTACT_BUF 32
const std::string BASE_LINK_NAME("robot::base_link::base_link_fixed_joint_lump__base_collision_collision");
const std::string WORLD_LINK_NAME("ground_plane::link::collision");
bool contact_buffer[NUM_CONTACT_BUF] = { 0 };
uint32_t contact_idx = 0;
bool collided_() {
  uint8_t j = 0;
  for(uint8_t i = 0; i < NUM_CONTACT_BUF; i++, j += contact_buffer[(contact_idx + i) & (NUM_CONTACT_BUF - 1)]);
  return j > 1;
}
gazebo::transport::SubscriberPtr sub; // what. so if this isn't scoped the subscription just gets garbage-collected? Eugh. How?!
ros::Publisher pub;
std_msgs::Bool contact_msg;

void tick(ConstContactsPtr &msg) {
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

void contactCallback(ConstContactsPtr &msg) {
  tick(msg);
  contact_msg.data = collided_();
  pub.publish(contact_msg);
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

int main(int argc, char** argv) {
  ros::init(argc, argv, "base_contact");
  ros::NodeHandle n;
  
  pub = n.advertise<std_msgs::Bool>("base_contact", 1000);
  
  gazebo_init(argc, argv);
  
  ros::spin();
  
  // Make sure to shut everything down.
  gazebo::client::shutdown();
  return 0;
}