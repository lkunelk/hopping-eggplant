#include "ros/ros.h"
#include "gazebo_msgs/LinkStates.h"
#include "geometry_msgs/Quaternion.h"
#include "tf2/LinearMath/Quaternion.h"

#include <iostream>

void chatterCallback(const gazebo_msgs::LinkStates& msg)
{
  geometry_msgs::Quaternion q = msg.pose[2].orientation;
  ROS_INFO("I heard: [%f %f %f %f]", q.x, q.y, q.z, q.w);
  
  // convert to radians
  tf2::Quaternion myQ(q.x, q.y, q.z, q.w);
  ROS_INFO("I heard: [%f]", myQ.getAngle());
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "my_node");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("gazebo/link_states", 1000, chatterCallback);
  
  ros::spin();
  
  return 0;
}
