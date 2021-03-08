#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "gazebo_msgs/LinkStates.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "util.hpp"
#include "consts.hpp"

#include <boost/circular_buffer.hpp>
#include <algorithm>

const float MAX_ANGV = 9.04f; // rad/s, 0.1s/60deg
const float T_STALL = 2.8f; // N-m

bool last_collided = true; // false;
boost::circular_buffer<float> zvel_buf(32);

ros::Publisher commandPub;
std_msgs::Float64 commandMsg;

void link_cb(const gazebo_msgs::LinkStates &msg) {
  zvel_buf.push_back(msg.twist[idxOf(msg.name, FLY_LINK_NAME)].linear.z);
}

void cb(const sensor_msgs::JointState &msg) {
  int joint_idx = idxOf(msg.name, PRISM_JOINT_NAME);
  float v = msg.velocity[joint_idx];
  float Fm = 0.0f;

  bool enF = last_collided;
  float zvel_filt = 0.0f;
  if (!zvel_buf.full()) {
    enF = false;
  } else {
    // moving average filter, consider better FIR filters
    for (float zvel : zvel_buf) {
      zvel_filt += zvel / zvel_buf.size();
    }
    enF &= zvel_filt > 0;
  }

  // Is touching ground and velocity is upwards
  if (enF) {
    commandMsg.data = T_STALL * (1.0f - fmax(0.0f, fmin(1.0f, v / MAX_ANGV)));
  } else {
    commandMsg.data = 0.0;
  }

  printf("%.3f\t%.3f\t%.3f\t%.3f\t%d\t%.3f\n", zvel_filt, msg.position[0], msg.position[1], v, last_collided, commandMsg.data);
  commandPub.publish(commandMsg);
}

void base_contact_cb(const std_msgs::Bool msg) {
  last_collided = msg.data;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "prism_ctrl");
  ros::NodeHandle n;

  commandPub = n.advertise<std_msgs::Float64>("arm_controller/command", 1000);

  ros::Subscriber sub = n.subscribe("/joint_states", 1000, cb);
  ros::Subscriber sub_base_contact = n.subscribe("base_contact", 1000, base_contact_cb);
  ros::Subscriber sub_links = n.subscribe("/gazebo/link_states", 1000, link_cb);

  ros::spin();

  return 0;
}