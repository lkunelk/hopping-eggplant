#!/usr/bin/env python3
import os
import rospy
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64, Float64MultiArray
import time

# interrupt handling
from threading import Event
import sys

exit = Event()
exit_signal = 0

TOLERANCE = 0.0001  # value we consider close enough to zero
arm_piston_pos = 1
spring_pos = 0
total_energy = 0
pub_spring_k = None
pub_piston_t = None
pub_piston_v = None
pub_piston_specs = None


def callback(data):
    global spring_pos, arm_piston_pos, total_energy

    spring_pos = data.x
    arm_piston_pos = data.z
    total_energy = data.w

def quit(signo, _frame):
    global exit_signal
    exit_signal = signo
    print("Interrupted by %d, shutting down" % signo)
    exit.set()

def main():
    # set interrupt handlers
    import signal
    for sig in ('TERM', 'HUP', 'INT'):
        signal.signal(getattr(signal, 'SIG'+sig), quit);
        
    global pub_spring_k, pub_piston_specs
    rospy.init_node('grid_test', anonymous=True)
    rospy.Subscriber("pendulum_pos", Quaternion, callback)
    pub_spring_k = rospy.Publisher('update_spring_k', Float64, queue_size=10)
    pub_piston_specs = rospy.Publisher('piston_specs', Float64MultiArray, queue_size=10)

    t_v_maxs = [(4.27368421052632E+00, 5.92165281926648E+00), (1.65714285714286E+00, 1.52716309549504E+01), (1.02784810126582E+00, 2.46216090906343E+01)]
    z_ks = {
        'E=0.289J' : [(8.15069062856409E-02, 1.44375E+02), (3.93235630189914E-02, 4.33125E+02), (2.87658744048376E-02, 7.21875E+02)],
        'E=0.566J' : [(1.14644484066114E-01, 1.44375E+02), (5.91836971769453E-02, 4.33125E+02), (4.42708576996474E-02, 7.21875E+02)],
        'E=1.000J' : [(1.43829372024188E-01, 1.44375E+02), (7.63320431845617E-02, 4.33125E+02), (5.76016775618413E-02, 7.21875E+02)]
    }

    file_path = 'data.csv'
    with open(file_path, 'w') as file:
        file.write('time(s), energy, z, k, t, v, final_energy(J)\n')

    start_time = time.time()

    # loop through different parameters
    for key in z_ks:
        for power_i in range(3):
            for spring_i in range(3):

                z, k = z_ks[key][spring_i]
                t, v = t_v_maxs[power_i]

                result = experiment(
                    initial_z=z,
                    spring_k=k,
                    motor_tmax=t,
                    motor_vmax=v
                )

                curr_time = time.time() - start_time

                with open(file_path, 'a') as file:
                    line = '{},{},{},{},{},{},{}\n'.format(curr_time, key, z, k, t, v, result)
                    file.write(line)


    if exit.is_set():
        sys.exit(exit_signal)
        
    rospy.spin()


def experiment(initial_z, spring_k, motor_tmax, motor_vmax):
    """ spring_k [N/m]
    motor_vmax [m/s]
    motor_tmax [Nm]
    initial_energy [J] """

    # unpause
    pause_sim = 'rosservice call /gazebo/unpause_physics'
    os.system(pause_sim)

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
           "'joint_positions':[" + str(-initial_z) + ", 0.0]}\""
    os.system(cmd + args)

    # update params
    rospy.loginfo("Update Spring_K={} motor_vmax={} motor_tmax={}".format(spring_k, motor_vmax, motor_tmax))
    
    pub_spring_k.publish(spring_k)
    pub_piston_specs.publish(data=[motor_vmax, motor_tmax])

    # unpause
    pause_sim = 'rosservice call /gazebo/unpause_physics'
    os.system(pause_sim)

    # wait for experiment to be done
    while spring_pos < 0 and not exit.is_set():
        exit.wait(0.1)
    
    rospy.loginfo("Max energy:{}".format(total_energy))

    return total_energy


if __name__ == '__main__':
    main()
