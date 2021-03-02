#include "ros/ros.h"
#include "gazebo_msgs/ContactState.h"
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include "std_msgs/Bool.h"
#include <boost/circular_buffer.hpp>

const std::string BASE_LINK_NAME("robot::base_link::base_link_collision");
const std::string WORLD_LINK_NAME("ground_plane::link::collision");

boost::circular_buffer<float> contact_buffer(32);

bool collided_() {
  uint8_t j = 0;
  for (bool contacted : contact_buffer) {
    j += contacted;
  }
  return j > 1;
}

gazebo::transport::SubscriberPtr sub; // what. so if this isn't scoped the subscription just gets garbage-collected? Eugh. How?!
ros::Publisher pub;
std_msgs::Bool contact_msg;

void tick(ConstContactsPtr &msg) {
  for (uint32_t i = 0; i < msg->contact_size(); i++) {
    auto contact = msg->contact(i);
    if ((BASE_LINK_NAME.compare(contact.collision1()) == 0 && WORLD_LINK_NAME.compare(contact.collision2()) == 0) ||
        (WORLD_LINK_NAME.compare(contact.collision1()) == 0 && BASE_LINK_NAME.compare(contact.collision2()) == 0)) {
      // collision with base link: usual physics
      contact_buffer.push_back(true);
      return;
    }
  }
  contact_buffer.push_back(false);
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

int main(int argc, char **argv) {
  ros::init(argc, argv, "base_contact");
  ros::NodeHandle n;

  pub = n.advertise<std_msgs::Bool>("base_contact", 1000);

  gazebo_init(argc, argv);

  ros::spin();

  // Make sure to shut everything down.
  gazebo::client::shutdown();
  return 0;
}