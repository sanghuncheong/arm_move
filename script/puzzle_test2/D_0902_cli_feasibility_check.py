#!/usr/bin/env python

import rospy
import D_0902_client_function as CLF
from arm_move.srv._arm_move_srv import *
from tf.transformations import quaternion_from_euler
import numpy as np

def feasible_check_client():
    rospy.wait_for_service('feasibile_check_srv')
    try:
        planner_name = 'RRTConnect'
        # planner_name = 'BiTRRT'
        n_attempt = 10
        c_time = 2
        n_repeat = 5

        start_state = moveit_msgs.msg.RobotState()
        joint_state = sensor_msgs.msg.JointState()
        joint_state.header = std_msgs.msg.Header()
        joint_state.name = ['j2n6s300_joint_1', 'j2n6s300_joint_2', 'j2n6s300_joint_3', 'j2n6s300_joint_4', 'j2n6s300_joint_5', 'j2n6s300_joint_6']
        joint_state.position = [-1.8060397407677595, 4.216119699628392, 4.09790370144882, 0.007598511489397008, -0.4205686124604682, -1.1475175752501585]
        # joint_state.position = [3.1415927410125732, 4.537856101989746, 5.93411922454834, -0.6108652353286743, 1.7453292608261108, -0.5235987901687622]
        start_state.joint_state = joint_state
        print start_state, "start"
        # goal_pose:
        # goal_orientation:
        goal_pose = [-0.010, -0.0046, 0.673]
        goal_orientation = [-0.1869, 0.125044, 0.69344, 0.6845144]
        # goal_orientation = [-0.006, 0.044, 0.705, 0.707]

        # goal_orientation = quaternion_from_euler(np.deg2rad(30), 0, np.deg2rad(90), axes='rxyz')
        print "goal orientation", goal_orientation
        nTrial = 0
        feasibility1 = 0
        while nTrial < 1 and feasibility1 == 0:
            [feasibility1, trajectory1] = CLF.feasible_check_obj_joint_client('arm', 'hand', start_state, goal_pose, goal_orientation, [], planner_name, n_attempt + 10 * nTrial, c_time, n_repeat)
            nTrial = nTrial + 1
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

if __name__ == '__main__':
    print "Requesting for feasibility check in moveIT"
    feasible_check_client()
