#include <string>

const std::string
  BASE_LINK_NAME("robot::base_link"),
  ARM_LINK_NAME("robot::arm_link"),
  ARM2_LINK_NAME("robot::arm2_link"),
  FLY_LINK_NAME("robot::flywheel_link"),
  TRACK_LINK_NAME("tracker::tracking_dot"),
  
  PRISM_JOINT_NAME("servo_joint"),
  SPRING_JOINT_NAME("spring_joint"),
  FLY_JOINT_NAME("flywheel_joint");

const float
	ARM_LENGTH = 0.1f,
	NUM_EPS = 1E-3f;