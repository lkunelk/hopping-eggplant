#include "consts.hpp"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "util.hpp"

float SPRING_K = 433.1f;  // 0.1925; // N/m

ros::Publisher springPub;
std_msgs::Float64 springMsg;

void cbUpdateSpringK(const std_msgs::Float64 &msg) { SPRING_K = msg.data; }

void cb(const sensor_msgs::JointState &msg) {
  springMsg.data = -SPRING_K * msg.position[idxOf(msg.name, SPRING_JOINT_NAME)];
  springPub.publish(springMsg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "spring_emulator");
  ros::NodeHandle n;
  springPub = n.advertise<std_msgs::Float64>("arm_spring/command", 1000);
  ros::Subscriber sub = n.subscribe("joint_states", 1000, cb);
  ros::Subscriber subSpringK = n.subscribe("update_spring_k", 1000, cbUpdateSpringK);
  ros::spin();
  return 0;
}
