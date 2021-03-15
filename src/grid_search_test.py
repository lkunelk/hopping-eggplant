#!/usr/bin/env python3
import os
import rospy
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64

TOLERANCE = 0.0001  # value we consider close enough to zero
arm_piston_pos = 1
spring_pos = 0
total_energy = 0
pub_spring_k = None
pub_piston_t = None
pub_piston_v = None


def callback(data):
    global spring_pos, arm_piston_pos, total_energy

    spring_pos = data.x
    arm_piston_pos = data.z
    total_energy = data.w


def main():
    global pub_spring_k
    rospy.init_node('grid_test', anonymous=True)
    rospy.Subscriber("pendulum_pos", Quaternion, callback)
    pub_spring_k = rospy.Publisher('update_spring_k', Float64, queue_size=10)

    # read csv data

    # loop through different parameters
    experiment(800, 1, 1)

    rospy.spin()


def experiment(spring_k, motor_vmax, motor_tmax, initial_z=-0.05918):
    """ spring_k [N/m]
    motor_vmax [m/s^2]
    motor_tmax [Nm]
    initial_energy [J] """

    # simulation reset
    rospy.loginfo("Reset sim")
    reset_sim = 'rosservice call /gazebo/reset_simulation'
    os.system(reset_sim)

    # reset controllers (simulation has to be running for some reason)
    rospy.loginfo("Reset joint_states controller")
    stop_ctrl = 'rosrun controller_manager controller_manager stop joint_state_controller'
    unload_ctrl = 'rosrun controller_manager controller_manager unload joint_state_controller'
    spawn_ctrl = 'rosrun controller_manager controller_manager spawn joint_state_controller'
    os.system(stop_ctrl)
    os.system(unload_ctrl)
    os.system(spawn_ctrl)

    # pause sim
    rospy.loginfo("Pause sim")
    pause_sim = 'rosservice call /gazebo/pause_physics'
    os.system(pause_sim)

    # reset positions
    rospy.loginfo("Set Initial z=" + str(initial_z))
    cmd = "rosservice call /gazebo/set_model_configuration "
    args = "\"{'model_name':'robot', " \
           "'urdf_param_name':'robot_description'," \
           "'joint_names':['arm_spring', 'arm_piston_driver']," \
           "'joint_positions':[" + str(initial_z) + ", 0.0]}\""
    os.system(cmd + args)

    # update params
    rospy.loginfo("Update Spring K={}".format(spring_k))
    pub_spring_k.publish(spring_k)

    # unpause
    pause_sim = 'rosservice call /gazebo/unpause_physics'
    os.system(pause_sim)

    # wait for experiment to be done
    while abs(spring_pos) > TOLERANCE:
        pass

    rospy.loginfo("Max energy:{}".format(total_energy))

    return total_energy


if __name__ == '__main__':
    main()
