#include "ros/ros.h"
#include "gazebo_msgs/LinkStates.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Quaternion.h"
#include "tf2/LinearMath/Quaternion.h"
#include "util.hpp"
#include "consts.hpp"

#include <iostream>
#include <cmath>
#include <cstring>

const float P = 1.00f;
const float I = 0.008f;
const float D = 0.00f;
const float G = 9.8f;

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

// control related
ros::Publisher flywheelCommandPub;
std_msgs::Float64 commandMsg;

// debug related
ros::Publisher posPub;
ros::Publisher velPub;
ros::Publisher flywheelVelPub;
geometry_msgs::Quaternion posMsg;
std_msgs::Float64 velMsg;
std_msgs::Float64 flywheelVelMsg;

sensor_msgs::JointState last_joint_states;
bool last_joint_states_valid = false;

void jointCallback(const sensor_msgs::JointState &msg) {
  last_joint_states = msg;
  last_joint_states_valid = true;
}

bool last_collided = false;

void baseContactCallback(const std_msgs::Bool msg) {
  last_collided = msg.data;
}

float sgn(float x) {
  return x > 0 ? 1.0f : -1.0f;
}

float last_land_angle = 0.0f;
const float P_TAKEOFF = 1.0f; // takeoff angle P controller

// #define ZVEL_NUM 32
// float zvel_buf[ZVEL_NUM] = { 0 };
// uint8_t zvel_buf_idx = 0;
// bool zvel_buf_full

void linkStateCallback(const gazebo_msgs::LinkStates &msg) {
  geometry_msgs::Quaternion q = msg.pose[idxOf(msg.name, ARM_LINK_NAME)].orientation;
  geometry_msgs::Vector3 a = msg.twist[idxOf(msg.name, ARM_LINK_NAME)].angular;
  geometry_msgs::Vector3 flyA = msg.twist[idxOf(msg.name, FLY_LINK_NAME)].angular;

  // get robot state
  tf2::Quaternion myQ(q.x, q.y, q.z, q.w);
  float armAnglPos = myQ.getAngle();
  float armAnglOffset = 0.0f;
  // auto fpos = msg.pose[idxOf(msg.name, FLY_LINK_NAME)].position, bpos = msg.pose[idxOf(msg.name, BASE_LINK_NAME)].position;
  // tf2::Vector3 dpos = (tf2::Vector3(fpos.x, fpos.y, fpos.z) - tf2::Vector3(bpos.x, bpos.y, bpos.z)).normalized();
  // float position = acos(dpos.getZ());
  float armAnglVel = a.x;  // measured wrt world's x axis
  float flywheelVel = flyA.x;  // measured wrt world's x axis

  if (q.x < 0)  // leaning in the clockwise direction
    armAnglPos *= -1;

  if (!last_collided) {
    geometry_msgs::Vector3 v = msg.twist[idxOf(msg.name, FLY_LINK_NAME)].linear;
    geometry_msgs::Point p = msg.pose[idxOf(msg.name, FLY_LINK_NAME)].position;
    float vh = v.y; // sqrt(v.x * v.x = v.y * v.y); // although really only the rotation in the direction we control matters
    // float tf = p.z + v.z / G;
    float pzmax = p.z + fabs(v.z * v.z / G / 2);
    if (pzmax > 0) {
      if (fabs(vh) > NUM_EPS) {
        float vzf = sqrt(2 * fabs(G * (pzmax - std::min(ARM_LENGTH, pzmax)))); // COE
        // iterated root finding to find the actual height and landing angle (estimate without compensation can be quite bad for shallow landing angles)
        const uint16_t MAX_VZ_ITER = 32;
        for (uint16_t i = 0; i < MAX_VZ_ITER; i++) {
          vzf = sqrt(2 * fabs(G * (pzmax - fabs(vzf) * ARM_LENGTH / sqrt(vzf * vzf + vh * vh))));
        }

        last_land_angle = atan2(vzf, vh) - M_PI_2; // rotate relative to vertical
        // ROS_INFO("SET %.3f, %.3f, %.3f, %.3f, %.3f", p.z, vh, v.z, vzf, pzmax);
      } else {
        last_land_angle = 0.0f; // no horizontal velocity: landing perfectly vertical
      }
    }
    // else {} // the robot is underground and ain't getting back up
  }

  // if(last_collided)
  //   ROS_INFO("%.3f", last_joint_states.velocity[1]);

  if (last_collided && last_joint_states_valid &&
      last_joint_states.effort[idxOf(last_joint_states.name, PRISM_JOINT_NAME)] > 1E-3) {
    // second joint expanding: point in target direction
    float delta = msg.pose[idxOf(msg.name, TRACK_LINK_NAME)].position.y
                  - msg.pose[idxOf(msg.name, FLY_LINK_NAME)].position.y;
    armAnglOffset += sgn(delta) * fmin(fabs(delta) / P_TAKEOFF, M_PI_4);
    ROS_INFO("SET %.3f, %.3f, %.3f, %.3f", delta, armAnglOffset, last_land_angle, flywheelVel);
  } else {
    armAnglOffset += last_land_angle;
  }

  // if(last_joint_states_valid)

  // compute and publish control command
  float command = P * (armAnglPos + armAnglOffset) + I * armAnglVel;
  float sign = (command > 0) - (command < 0);
  float maxTorque = fmax(0.0f, (1 - std::abs(flywheelVel) / noLoadSpeed) * stallTorque);

  command = sign * std::min(maxTorque, std::abs(command));

  commandMsg.data = command;
  flywheelCommandPub.publish(commandMsg);
  
  // send 0 and let PID controller emulate spring
  // armSpringMsg.data = 0.0f;
  // armSpringPub.publish(armSpringMsg);
  
  if(last_joint_states_valid) {
    // Publish pendulum state for debug
    posMsg.x = msg.pose[idxOf(msg.name, ARM2_LINK_NAME)].position.z;
    posMsg.y = msg.pose[idxOf(msg.name, BASE_LINK_NAME)].position.z;
    posMsg.z = last_joint_states.position[idxOf(last_joint_states.name, SPRING_JOINT_NAME)]; // msg.twist[idxOf(msg.name, ARM2_LINK_NAME)].linear.z;
    posMsg.w = msg.twist[idxOf(msg.name, FLY_LINK_NAME)].linear.z;
    velMsg.data = armAnglVel;
    flywheelVelMsg.data = flywheelVel / 1000.0; // so it fits nicely on graph
    
    posPub.publish(posMsg);
    velPub.publish(velMsg);
    flywheelVelPub.publish(flywheelVelMsg);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "my_node");

  ros::NodeHandle n;

  flywheelCommandPub = n.advertise<std_msgs::Float64>("flywheel_controller/command", 1000);
  velPub = n.advertise<std_msgs::Float64>("pendulum_vel", 1000);
  flywheelVelPub = n.advertise<std_msgs::Float64>("pendulum_flywheel_vel", 1000);
  posPub = n.advertise<geometry_msgs::Quaternion>("pendulum_pos", 1000);

  ros::Subscriber jointSub = n.subscribe("/joint_states", 1000, jointCallback);
  ros::Subscriber sub = n.subscribe("gazebo/link_states", 1000, linkStateCallback);
  ros::Subscriber subBaseContact = n.subscribe("base_contact", 1000, baseContactCallback);

  ros::spin();

  return 0;
}
