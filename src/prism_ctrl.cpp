#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "gazebo_msgs/LinkStates.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include <boost/circular_buffer.hpp>
#include "util.hpp"
#include "consts.hpp"

#include <algorithm>

#define NUM_DSTL0829 49
const float DSTL0829[NUM_DSTL0829][2] = { {0.0000673f,3.5282f}, {0.0002231f,3.5282f}, {0.0002374f,3.5282f}, {0.0003524f,3.5282f}, {0.0007758f,3.5282f}, {0.0009524f,3.5282f}, {0.0012585f,3.5282f}, {0.0017533f,3.5418f}, {0.0020009f,3.6028f}, {0.0022486f,3.6434f}, {0.0025082f,3.6773f}, {0.0033304f,3.6841f}, {0.0033894f,3.8197f}, {0.0036649f,3.8264f}, {0.0038826f,3.8264f}, {0.0041630f,3.9213f}, {0.0044232f,3.9891f}, {0.0046834f,4.1179f}, {0.0049436f,4.1586f}, {0.0052039f,4.2738f}, {0.0054641f,4.3755f}, {0.0057244f,4.4839f}, {0.0059847f,4.6127f}, {0.0062529f,4.7415f}, {0.0065052f,4.8838f}, {0.0067655f,5.0939f}, {0.0070258f,5.1888f}, {0.0072861f,5.3447f}, {0.0075464f,5.5277f}, {0.0078067f,5.6904f}, {0.0080670f,5.9412f}, {0.0083274f,6.1919f}, {0.0085877f,6.4495f}, {0.0088480f,6.7274f}, {0.0091083f,7.3093f}, {0.0093687f,7.7239f}, {0.0096290f,8.1965f}, {0.0098893f,7.9241f}, {0.0101497f,10.6457f}, {0.0104100f,12.0548f}, {0.0106704f,13.6083f}, {0.0109307f,15.3828f}, {0.0111910f,19.7071f}, {0.0114514f,22.6574f}, {0.0117118f,24.3762f}, {0.0119721f,29.3125f}, {0.0122325f,35.5877f}, {0.0124929f,31.0248f}, {0.0126783f,32.9223f} };
const float HALF_IN = 0.0126783f;
const float Kspring = 787; // 0.3850E3f; // 0.1925; // N/m
const float F0 = 5; // motor stall force (N)
ros::Publisher commandPub, springPub;
std_msgs::Float64 commandMsg, springMsg;
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

boost::circular_buffer<float> zvel_buf(32);
void link_cb(const gazebo_msgs::LinkStates& msg) {
	zvel_buf.push_back(msg.twist[idxOf(msg.name, FLY_LINK_NAME)].linear.z);
}

const float // ARM_LENGTH = 0.0127f, // m
            PISTON_LENGTH = 0.0127f, // m
            MAX_ANGV = 10.472f, // rad/s, 0.1s/60deg 
            T_STALL = 7.2f, // N-m
            P_PARAM = 0.01f; // N-m/rad, arbitrary but small, compensate by this
void cb(const sensor_msgs::JointState& msg) {
	// uint8_t dstl_idx = 0;
	// for(; dstl_idx < NUM_DSTL0829; dstl_idx++) {
	// 	if(msg.position[idxOf(msg.name, PRISM_JOINT_NAME)] < DSTL0829[dstl_idx][0])
	// 		break;
	// }
	// int joint_idx = idxOf(msg.name, PRISM_JOINT_NAME);
	// float pos = fmin(fmax(msg.position[joint_idx], -ARM_LENGTH), ARM_LENGTH),
	//       angle = asin(pos / ARM_LENGTH),
	//       v = msg.twist[joint_idx].linear.z
	//       angV = fabs(pos) == ARM_LENGTH ? 0 : v / cos(angle); // check limit hit
	
	// float Fsp = -Kspring * msg.position[0];
	// float Fm = T_STALL * fmax(0.0f, cos(angle) - fabs(v) / MAX_ANGV); // DSTL0829[std::min((int)dstl_idx, NUM_DSTL0829 - 1)][1] * 2;
	float Fm = 0.0f;
	
	bool enF = last_collided;
	if(!zvel_buf.full())
		enF = false;
	else {
		// moving average filter, consider better FIR filters
		float zvel_filt = 0.0f;
		for(float zvel : zvel_buf) {
			zvel_filt += zvel / zvel_buf.size();
		}
		enF &= zvel_filt > 0;
	}
	if(enF) {
		// if(msg.position[0] < -HALF_IN && Fm < Fsp) {
		// 	commandMsg.data = 0.0f;
		// }
		// else {
			commandMsg.data = 10.0f; // T_STALL / P_PARAM; // -Fm; // Fm + Fsp;
		// }
	}
	else {
		commandMsg.data = -10.0f; // -T_STALL / P_PARAM; // Fsp;
	}
	
	printf("%.3f\t%.3f\t%.3f\t%d\t%.3f\n", msg.position[0], msg.position[1], Fm, last_collided, commandMsg.data);
	commandPub.publish(commandMsg);
	
	springMsg.data = 0.0f; // -Kspring * msg.position[1];
	springPub.publish(springMsg);
}
void base_contact_cb(const std_msgs::Bool msg) {
	last_collided = msg.data;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "prism_ctrl");
  ros::NodeHandle n;
  
  commandPub = n.advertise<std_msgs::Float64>("arm_controller/command", 1000);
  springPub = n.advertise<std_msgs::Float64>("arm_spring/command", 1000);
  ros::Subscriber sub = n.subscribe("/joint_states", 1000, cb);
  ros::Subscriber sub_base_contact = n.subscribe("base_contact", 1000, base_contact_cb);
  ros::Subscriber sub_links = n.subscribe("/gazebo/link_states", 1000, link_cb);
  
  ros::spin();
  
  // Make sure to shut everything down.
	return 0;
}