#!/usr/bin/env python

import rospy
from arm_move.srv._arm_move_srv import *

def feasible_check_client():
    rospy.wait_for_service('feasibile_check_srv')
    try:
        f_check_srv = rospy.ServiceProxy('feasibile_check_srv', arm_move_srv)
        pub_msg = arm_move_srvRequest()
        # pub_msg.arm_name.append('r_arm')
        pub_msg.arm_name.append('arm')
        pub_msg.goal_position.x = 0.0
        pub_msg.goal_position.y = -0.1
        pub_msg.goal_position.z = 0.80
        pub_msg.goal_orientation.x = -0.00145713772037
        pub_msg.goal_orientation.y = -0.998970756926
        pub_msg.goal_orientation.z = 0.0364956710831
        pub_msg.goal_orientation.w = 0.0268955302573
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
    feasible_check_client()
