#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "gazebo_msgs/LinkStates.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

const float Kspring = 0.3850E3f; // 0.1925; // N/m
const float F0 = 5; // motor stall force (N)
ros::Publisher commandPub;
std_msgs::Float64 commandMsg;
bool last_collided = false;

float step(float x) {
	return x > 0;
}
float sgn(float x) {
	return x > 0 ? 1 : -1;
}
float relu(float x) {
	return x > 0 ? x : 0;
}

void cb(const sensor_msgs::JointState& msg) {
	float F = Kspring * relu(-msg.position[0]) + F0 * step(msg.velocity[0]) * last_collided;
	commandMsg.data = F;
	ROS_INFO("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%d", msg.position[0], msg.velocity[0], -Kspring * msg.position[0], F0 * step(msg.velocity[0]) * last_collided, F, msg.header.stamp.nsec);
	commandPub.publish(commandMsg);
}
void base_contact_cb(const std_msgs::Bool msg) {
	last_collided = msg.data;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "prism_ctrl");
  ros::NodeHandle n;
  
  commandPub = n.advertise<std_msgs::Float64>("arm_controller/command", 1000);
  ros::Subscriber sub = n.subscribe("/joint_states", 1000, cb);
  ros::Subscriber sub_base_contact = n.subscribe("base_contact", 1000, base_contact_cb);
  
  ros::spin();
  
  // Make sure to shut everything down.
	return 0;
}