#!/usr/bin/env python

import rospy
from arm_move.srv._arm_move_srv import *

def feasible_check_client():
    rospy.wait_for_service('feasibile_check_srv')
    try:
        f_check_srv = rospy.ServiceProxy('feasibile_check_srv', arm_move_srv)
        # Grasp the object if obj
        pub_msg = arm_move_srvRequest()
        pub_msg.arm_name.append('panda_arm')
        pub_msg.goal_position.x = 0.940
        pub_msg.goal_position.y = 0.179
        pub_msg.goal_position.z = 0.505
        pub_msg.goal_orientation.x = -0.638360135776
        pub_msg.goal_orientation.y = 0.251460095054
        pub_msg.goal_orientation.z = -0.672955856495
        pub_msg.goal_orientation.w = 0.276395681689
        pub_msg.planner_name = 'RRTConnect'
        pub_msg.n_attempt = 1000
        pub_msg.c_time = 5
        pub_msg.n_repeat = 5
        # pub_msg.start_state = []
        resp_f_check = f_check_srv(pub_msg)
        if resp_f_check.feasibility == 1:
            if len(resp_f_check.r_trj.joint_trajectory.points) > 0:
                print "\t\tPlan :", len(resp_f_check.r_trj.joint_trajectory.points), "steps",
                return [1, resp_f_check.r_trj]
            else:
                print "\t\tNP",
                return [0, resp_f_check.r_trj]
        else:
            print "\t\tNP",
            return [0, resp_f_check.r_trj]
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

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
