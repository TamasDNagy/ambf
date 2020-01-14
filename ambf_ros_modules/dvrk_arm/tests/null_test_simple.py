import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import time
import sys
from argparse import ArgumentParser


js_msg = JointState()
js_cmd = JointState()
qs = np.zeros(7)
vs = np.zeros(7)
Kp = 0.5
Kd = 0.1
arm_name = 'MTMR'
robot_ready = False
namespace = '/dvrk/'


def js_cb(msg):
    global qs, js_msg, vs
    js_msg = msg
    qs = js_msg.position
    vs = js_msg.velocity

def state_cb(msg):
    global robot_ready
    if msg.data == 'READY':
        robot_ready = True
    else:
        robot_ready = False

    print(msg.data)


def rot_z(q):
    r = np.mat([[np.cos(q),-np.sin(q), 0],
                  [np.sin(q), np.cos(q), 0],
                  [0, 0, 1]])
    return r


def main():
    global arm_name, Kp, Kd, robot_ready
    # Begin Argument Parser Code
    parser = ArgumentParser()

    parser.add_argument('-a', action='store', dest='arm_name', help='Specify Arm Name',
                        default='MTMR')
    parser.add_argument('-p', action='store', dest='Kp', help='Specify Linear Gain',
                        default=0.5)
    parser.add_argument('-d', action='store', dest='Kd', help='Specify Damping Gain',
                        default=0.1)
    parser.add_argument('-n', action='store', dest='namespace', help='ROS Namespace',
                        default='dvrk')

    parsed_args = parser.parse_args()
    print('Specified Arguments')
    print(parsed_args)

    arm_name = parsed_args.arm_name

    if arm_name != 'MTMR' and arm_name != 'MTML':
        print('Specified arm is: ', arm_name)
        raise Exception('Error, specify MTML or MTMR as input arg')

    Kp = float(parsed_args.Kp)
    Kd = float(parsed_args.Kd)

    node_name = arm_name + '_null_space_test'
    rospy.init_node(node_name)
    joint_state_sub = rospy.Subscriber(namespace + arm_name + '/state_joint_current', JointState, js_cb)
    robot_state_sub = rospy.Subscriber(namespace + arm_name + '/current_state', String, state_cb, queue_size=1)

    joint_state_pub = rospy.Publisher(namespace + arm_name + '/set_effort_joint', JointState, queue_size=10)
    js_cmd.effort = [0, 0, 0, 0, 0, 0, 0]

    e = 0.0
    e_pre = e
    r = rospy.Rate(1000)

    while not rospy.is_shutdown():
        
        if robot_ready:

            lim1 = -1.5
            lim2 = 1.3
            lim3 = 1.7

            sign = 1
            if lim1 < qs[4] <= lim2 :
                sign = 1
            elif lim2 < qs[4] < lim3:
                sign = 0
            else:
                sign = -1

            e = qs[5]
            tau4 = Kp * e * sign - Kd * vs[3]
            tau4 = np.clip(tau4, -0.3, 0.3)
            js_cmd.effort[3] = tau4
            joint_state_pub.publish(js_cmd)
            print 'PITCH JOINT: ', qs[4]
            # print(round(e, 2))
            # print('YAW :', qs[3], 'PITCH: ', qs[4], ' | YAW: ', qs[5])
        r.sleep()

if __name__ == "__main__":
    main()




