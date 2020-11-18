#!/usr/bin/env python

import rospy
from arm_move.srv._arm_move_srv import *

def feasible_check_client():
    rospy.wait_for_service('feasibile_check_srv')
    try:
        f_check_srv = rospy.ServiceProxy('feasibile_check_srv', arm_move_srv)
        pub_msg = arm_move_srvRequest()
        # pub_msg.arm_name.append('r_arm')
        pub_msg.arm_name.append('R_arm')
        # go to home position
        f_pos = [0.07476961612701416, -0.020279844257987047, 1.1099996440321116]
        f_ori = [-1.5682522853416894, 1.3108451923926623e-07, -0.7072301506996155, -5.103168518871826e-07]

        # x: -0.00145713772037
        # y: -0.998970756926
        # z: 0.0364956710831
        # w: 0.0268955302573

        # [0.07476961612701416, -0.018975496292114258, 0.8619123697280884]
        # [-0.00145713772037, -0.998970756926, 0.0364956710831, 0.0268955302573]
        pub_msg.goal_position.x = 0.3
        pub_msg.goal_position.y = -0.01
        pub_msg.goal_position.z = 0.50
        pub_msg.goal_orientation.x = -0.0
        pub_msg.goal_orientation.y = -0.0
        pub_msg.goal_orientation.z = 0.707
        pub_msg.goal_orientation.w = 0.707
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
