#include <algorithm>
#include <boost/circular_buffer.hpp>

#include "consts.hpp"
#include "gazebo_msgs/LinkStates.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "util.hpp"

const float MAX_ANGV = 9.04f * 0.04;  // rad/s, 0.1s/60deg
const float T_STALL = 2.8f / 0.04;    // N-m

bool lastTouchSensorVal = false;
bool fireServo = false;
ros::Time beginFireServo;

ros::Publisher commandPub;
std_msgs::Float64 commandMsg;

void base_contact_cb(const std_msgs::Bool msg) { lastTouchSensorVal = msg.data; }

// emulate simplified servo motor dynamics
void servo_emulation_cb(const sensor_msgs::JointState &msg) {
  int joint_idx = idxOf(msg.name, PRISM_JOINT_NAME);
  float v = msg.velocity[joint_idx];

  if (fireServo) {
    commandMsg.data = T_STALL * (1.0f - fmax(0.0f, fmin(1.0f, v / MAX_ANGV)));
  } else {
    commandMsg.data = -40.0;  // retract servo
  }

  commandPub.publish(commandMsg);
}

void controlUpdate() {
  // if we touched ground fire servo for set amount of time
  if (fireServo) {
    ros::Duration dur = ros::Time::now() - beginFireServo;
    bool timeExpired = dur.toSec() > 0.1;
    if (timeExpired) {
      fireServo = false;
    }
  } else if (false) {  // lastTouchSensorVal == true) {
    ROS_INFO("Fire Servo!");
    fireServo = true;
    beginFireServo = ros::Time::now();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "prism_ctrl");
  ros::NodeHandle n;

  commandPub = n.advertise<std_msgs::Float64>("arm_controller/command", 1000);

  ros::Subscriber sub = n.subscribe("joint_states", 1000, servo_emulation_cb);
  ros::Subscriber sub_base_contact = n.subscribe("base_contact", 1000, base_contact_cb);

  ros::Rate rate(1000);  // Control update rate (Hz)
  while (ros::ok()) {
    controlUpdate();
    ros::spinOnce();  // process all the callbacks
    rate.sleep();
  }

  return 0;
}