//
// Created by nam on 2021-08-17.
//

#include "yaw_controll.h"

#include <stdio.h>

#include "ros/ros.h"
#include "std_msgs/Float64.h"

ros::Publisher yawPub;

int main(int argc, char **argv) {
  ros::init(argc, argv, "way_node");
  ros::NodeHandle n;
  yawPub = n.advertise<std_msgs::Float64>("yaw_flywheel_controller/command", 1000);
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
