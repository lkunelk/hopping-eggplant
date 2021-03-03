#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"

const float SPRING_K = 385.0f; // 0.1925; // N/m

ros::Publisher springPub;
std_msgs::Float64 springMsg;

void cb(const sensor_msgs::JointState &msg) {
  springMsg.data = -SPRING_K * msg.position[1];
  springPub.publish(springMsg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "prism_ctrl");
  ros::NodeHandle n;
  springPub = n.advertise<std_msgs::Float64>("arm_spring/command", 1000);
  ros::Subscriber sub = n.subscribe("/joint_states", 1000, cb);
  ros::spin();
  return 0;
}