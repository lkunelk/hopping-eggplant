//
// Created by nam on 2021-08-17.
//

#include "yaw_controll.h"

#include <stdio.h>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "gazebo_msgs/LinkStates.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"

#include "consts.hpp"
#include "util.hpp"

const float FLY_V_P = 8.776E-1;  // 1.6e-4f;
const float FLY_P = 6.65E3f;
const float FLY_I = 0.00f;
const float FLY_D = 5.84E3f;
const float Itot = 0.0053f;
const float G = 9.8f;

// Motor Parameters
const float stallTorque = 0.2;     // Nm
const float noLoadSpeed = 1000.0;  // rad/s
float flywheelVel = 0.0f, yaw_fly_vel = 0.0f;

// control related
ros::Publisher flywheelCommandPub;

const float YAW_P = 0.5f, YAW_D = 0.2f;
ros::Publisher yawPub;
ros::Subscriber linkStatesSub, jointStatesSub;

float sgn(float x) {
  return (x > 0) - (x < 0);
}

void jointStateCallback(const sensor_msgs::JointState &msg) {
  flywheelVel = msg.velocity[idxOf(msg.name, FLY_JOINT_NAME)];
  yaw_fly_vel = msg.velocity[idxOf(msg.name, YAW_FLY_JOINT_NAME)];
}

float signed_angle(tf2::Vector3 a, tf2::Vector3 b, tf2::Vector3 ref) {
  return acos(fmax(-1.0f, fmin(1.0f, a.dot(b)))) * sgn(a.cross(b).dot(ref));
}

ros::Time last_joint_states_time(0.0f);
void linkStateCallback(const gazebo_msgs::LinkStates &msg) {
  geometry_msgs::Quaternion qm = msg.pose[idxOf(msg.name, BASE_LINK_NAME)].orientation;
  geometry_msgs::Vector3 am = msg.twist[idxOf(msg.name, BASE_LINK_NAME)].angular;
  
  tf2::Vector3 a_world(am.x, am.y, am.z);
  tf2::Quaternion q(qm.x, qm.y, qm.z, qm.w);
  tf2::Vector3 zhat(0, 0, 1), xhat(1, 0, 0);
  
  // i think there is an easier way to do this by projecting the quaternion into the subspace of axes in the XY plane, but i have no idea how. That angular distance of that projection is the angle we want to go through.
  // for now just hack it with inelegant linear algebra
  tf2::Vector3 a = tf2::quatRotate(q.inverse(), a_world);
  tf2::Vector3 zhat_world = tf2::quatRotate(q, zhat),
               xhat_world = tf2::quatRotate(q, xhat);
  tf2::Vector3 proj_x = zhat_world.cross(zhat).normalized(); // get axis to rotate Z into world Z
  // tf2::Quaternion un_z_frame = (q * tf2::Quaternion(axis, asin(axis.length())));
  
  tf2::Quaternion x_arc_quat = tf2::shortestArcQuat(xhat_world, proj_x);
  
  // either of the below approaches work. second one is a bit more direct
  float angle = 0.0f;
  if(false) {
    // angle = x_arc_quat.getAngle() * sgn(x_arc_quat.getAxis().dot(zhat_world)); // get offset of XY axes to Z-aligned
    // switch((int)floor(angle / (M_PI_2))) {
    //   case -2:
    //     angle += M_PI;
    //     break;
    //   case 1:
    //     angle -= M_PI;
    //     break;
    // }
  }
  angle = signed_angle(xhat_world, proj_x, zhat_world);
  
  float robotAngVel = a_world.dot(xhat_world);
  float robotAngPos = cos(angle) * signed_angle(zhat_world, zhat, proj_x);
  
  {
    std_msgs::Float64 commandMsg;
    // positive command results in:
    // * negative angle
    // * negative a.z
    commandMsg.data = (-YAW_P * angle * (robotAngPos * 0.9f + 0.1f)) + YAW_D * a.z();
    yawPub.publish(commandMsg);
    ROS_INFO("%.3f\t%.3f\t%.3f\t%.3f", commandMsg.data, angle, a.z(), yaw_fly_vel);
  }
  
  {
    std_msgs::Float64 commandMsg;
    float command = (FLY_P * robotAngPos - FLY_D * robotAngVel - FLY_V_P * flywheelVel) * Itot;
    float sign = (command > 0) - (command < 0);
    float maxTorque = fmax(0.0f, (1 - std::abs(flywheelVel) / noLoadSpeed) * stallTorque);

    commandMsg.data = sign * std::min(maxTorque, std::abs(command));
    flywheelCommandPub.publish(commandMsg);
  }
  
  last_joint_states_time = ros::Time::now();
  
  // ROS_INFO("%.3f", commandMsg.data);
  
  
}

int main(int argc, char **argv) {
  
  ros::init(argc, argv, "way_node");
  ros::NodeHandle n;
  
  linkStatesSub = n.subscribe("gazebo/link_states", 1, linkStateCallback);
  jointStatesSub = n.subscribe("joint_states", 1000, jointStateCallback);
  yawPub = n.advertise<std_msgs::Float64>("yaw_flywheel_controller/command", 1000);
  flywheelCommandPub = n.advertise<std_msgs::Float64>("flywheel_controller/command", 1000);
  
  ros::Rate rate(1000);  // Control update rate (Hz)
  while (ros::ok()) {
    ros::spinOnce();  // process all the callbacks
    rate.sleep();
  }
  return 0;
  
  std_msgs::Float64 commandMsg;
  while (ros::ok()) {
    char in = ' ';
    std::cin >> in;
    switch (in) {
      case 'q':
        commandMsg.data = 1.0;
        break;
      case 'w':
        commandMsg.data = 0.0;
        break;
      case 'e':
        commandMsg.data = -1.0;
        break;
    }

    yawPub.publish(commandMsg);
  }

  return 0;
}
