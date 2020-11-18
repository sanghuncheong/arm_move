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
        # go to home position
        ws_zero_pos = [0.73, -0.39]
        grid_i = [4, 39]
        GRID_SIZE = 0.01
        xi, yi = ws_zero_pos[0] + grid_i[0] * GRID_SIZE, ws_zero_pos[1] + grid_i[1] * GRID_SIZE
        p_ori = [-0.00145713772037, -0.998970756926, 0.0364956710831,  0.0268955302573]

        pub_msg.goal_position.x = 0.07
        pub_msg.goal_position.y = -yi
        pub_msg.goal_position.z = xi
        pub_msg.goal_orientation.x = p_ori[0]
        pub_msg.goal_orientation.y = p_ori[1]
        pub_msg.goal_orientation.z = p_ori[2]
        pub_msg.goal_orientation.w = p_ori[3]
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
