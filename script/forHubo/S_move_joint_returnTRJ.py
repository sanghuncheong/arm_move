#!/usr/bin/env python

import rospy
import math
from arm_move.srv import *

def in_joints_client():
    rospy.wait_for_service('move_goalJoint_srv')
    try:
        i_joints_srv = rospy.ServiceProxy('move_goalJoint_srv', arm_move_joint_goal_srv)
        # pub_msg = box_info_msg()
        pub_msg = arm_move_joint_goal_srvRequest()
        pub_msg.arm_name.append('R_arm')
        # pub_msg.goalJoint.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # initial joint states for HUBO
        # pub_msg.goalJoint.position = [0.69132, -0.0872, 0.0, -2.6179, 0.0, 0.3491, 0.0] # "move ready" joint states
        pub_msg.goalJoint.position = [0.69132, -0.0872, 0.0, -2.6179, 0.0, 0.3491, 3.141592/2.0] # "move ready" joint states
        # pub_msg.goalJoint.position = [0.837537525182775, -0.8065800180796977, -0.31139572453698006, -2.455033983033206, 0.8193830818654515, -0.0042662650544393835, 0.0]  # "walk ready" joint states

        resp1 = i_joints_srv(pub_msg)
        print "response", resp1
        if resp1.w_flag == 1:
            print "work done"
            return resp1.w_flag
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

if __name__ == '__main__':
    print "Requesting for initiating joints of a robot in moveIT"
    in_joints_client()
