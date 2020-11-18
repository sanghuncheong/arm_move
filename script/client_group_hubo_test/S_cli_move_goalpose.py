#!/usr/bin/env python

import rospy
from arm_move.srv._arm_move_srv import *

def move_goalpose_client():
    rospy.wait_for_service('move_goalpose_srv')
    try:
        f_check_srv = rospy.ServiceProxy('move_goalpose_srv', arm_move_srv)
        pub_msg = arm_move_srvRequest()
        # pub_msg.arm_name.append('r_arm')
        pub_msg.arm_name.append('R_arm')
        # go to home position
        pub_msg.goal_position.x = 0.54
        pub_msg.goal_position.y = -0.2
        pub_msg.goal_position.z = 0.7
        pub_msg.goal_orientation.x = 0.0
        pub_msg.goal_orientation.y = 0.0
        pub_msg.goal_orientation.z = 0.0
        pub_msg.goal_orientation.w = 0.0
        resp1 = f_check_srv(pub_msg)
        # f_ori[0]
        if resp1.w_flag == 1:
            print "work done"
        if resp1.feasibility == 1:
            print "Plan is found successfully with", len(resp1.r_trj.joint_trajectory.points), "steps"
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

if __name__ == '__main__':
    print "Requesting for feasibility check in moveIT"
    move_goalpose_client()
