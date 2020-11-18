#!/usr/bin/env python

import rospy
from arm_move.srv._box_info_srv import *
from arm_move.srv._arm_move_srv import *
from arm_move.srv._work_start_srv import *
from arm_move.srv._att_hand_box_srv import *

def add_box_client(box_name, box_xyz, box_xyzw, box_wdh):
    rospy.wait_for_service('add_box_srv')
    try:
        add_box_srv = rospy.ServiceProxy('add_box_srv', box_info_srv)
        # pub_msg = box_info_msg()
        pub_msg = box_info_srvRequest()
        # pub_srv.header = 0
        pub_msg.object_name.append(box_name)

        pub_msg.object_position.x = box_xyz[0]
        pub_msg.object_position.y = box_xyz[1]
        pub_msg.object_position.z = box_xyz[2]
        pub_msg.object_orientation.x = box_xyzw[0]
        pub_msg.object_orientation.y = box_xyzw[1]
        pub_msg.object_orientation.z = box_xyzw[2]
        pub_msg.object_orientation.w = box_xyzw[3]
        pub_msg.object_scale.x = box_wdh[0]
        pub_msg.object_scale.y = box_wdh[1]
        pub_msg.object_scale.z = box_wdh[2]

        resp1 = add_box_srv(pub_msg)

        print "response", resp1
        if resp1.w_flag == 1:
            print "work done"
            return resp1.w_flag
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
def feasible_check_client(move_group_name, goal_xyz, goal_xyzw):
    rospy.wait_for_service('feasibile_check_srv')
    try:
        f_check_srv = rospy.ServiceProxy('feasibile_check_srv', arm_move_srv)
        pub_msg = arm_move_srvRequest()
        # pub_msg.arm_name.append('r_arm')
        pub_msg.arm_name.append(move_group_name)

        # before table
        pub_msg.goal_position.x = goal_xyz[0]
        pub_msg.goal_position.y = goal_xyz[1]
        pub_msg.goal_position.z = goal_xyz[2]
        pub_msg.goal_orientation.x = goal_xyzw[0]
        pub_msg.goal_orientation.y = goal_xyzw[1]
        pub_msg.goal_orientation.z = goal_xyzw[2]
        pub_msg.goal_orientation.w = goal_xyzw[3]
        resp1 = f_check_srv(pub_msg)

        # print "response", resp1
        if resp1.w_flag == 1:
            print "work done"
        if resp1.feasibility == 1:
            print "Plan is found successfully with", len(resp1.r_trj.joint_trajectory.points), "steps"
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
def move_goal_pose_client(move_group_name, goal_xyz, goal_xyzw):
    rospy.wait_for_service('move_goalpose_srv')
    try:
        f_check_srv = rospy.ServiceProxy('move_goalpose_srv', arm_move_srv)
        pub_msg = arm_move_srvRequest()
        # pub_msg.arm_name.append('r_arm')
        pub_msg.arm_name.append(move_group_name)

        # before table
        pub_msg.goal_position.x = goal_xyz[0]
        pub_msg.goal_position.y = goal_xyz[1]
        pub_msg.goal_position.z = goal_xyz[2]
        pub_msg.goal_orientation.x = goal_xyzw[0]
        pub_msg.goal_orientation.y = goal_xyzw[1]
        pub_msg.goal_orientation.z = goal_xyzw[2]
        pub_msg.goal_orientation.w = goal_xyzw[3]
        resp1 = f_check_srv(pub_msg)

        # print "response", resp1
        if resp1.w_flag == 1:
            print "work done"
        if resp1.feasibility == 1:
            print "Plan is found successfully with", len(resp1.r_trj.joint_trajectory.points), "steps"
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def rem_all_client():
    rospy.wait_for_service('remove_all_srv')
    try:
        rem_all_srv = rospy.ServiceProxy('remove_all_srv', work_start_srv)
        # pub_msg = box_info_msg()
        pub_msg = work_start_srvRequest()
        pub_msg.w_start =1

        resp1 = rem_all_srv(pub_msg)

        print "response", resp1
        if resp1.w_flag == 1:
            print "work done"
            return resp1.w_flag
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
def att_box_client(box_name, ee_name):
    rospy.wait_for_service('att_box_srv')
    try:
        att_box_srv = rospy.ServiceProxy('att_box_srv', att_hand_box_srv)
        pub_msg = att_hand_box_srvRequest()
        pub_msg.object_name.append(box_name)
        pub_msg.hand_name.append(ee_name)

        resp1 = att_box_srv(pub_msg)

        print "response", resp1
        if resp1.w_flag == 1:
            print "work done"
            return resp1.w_flag
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
def det_box_client(box_name):
    rospy.wait_for_service('det_box_srv')
    try:
        det_box_srv = rospy.ServiceProxy('det_box_srv', box_info_srv)
        pub_msg = box_info_srvRequest()
        pub_msg.object_name.append(box_name)

        resp1 = det_box_srv(pub_msg)

        print "response", resp1
        if resp1.w_flag == 1:
            print "work done"
            return resp1.w_flag
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
if __name__ == '__main__':
    print "Test scenario starts"

    rem_all_client()
    add_box_client('table', [0.6, 0.0, 0.2], [0, 0, 0, 0], [0.3, 1.0, 0.4])
    add_box_client('side1', [0.6, -0.5+0.005, 0.55], [0, 0, 0, 0], [0.3, 0.01, 0.3])
    add_box_client('side2', [0.6, 0.5-0.005, 0.55], [0, 0, 0, 0], [0.3, 0.01, 0.3])
    add_box_client('up',    [0.6, 0.0, 0.7+0.005], [0, 0, 0, 0], [0.3, 1.0, 0.01])
    add_box_client('box1',  [0.6, 0.0, 0.475], [0, 0, 0, 0], [0.06, 0.06, 0.15])

    # feasible_check_client('arm', [0.45, 0, 0.5], [0.707, 0.0, -0.707, 0.0])
    move_goal_pose_client('arm', [0.45, 0, 0.478], [0.707, 0.0, -0.707, 0.0])
    att_box_client('box1', 'hand')
    # rearrange pose
    move_goal_pose_client('arm', [0.45, 0, 0.8], [0.707, 0.0, -0.707, 0.0])
    det_box_client('box1')
    move_goal_pose_client('arm', [0.25, 0, 0.4], [0.707, 0.0, -0.707, 0.0]) # home
    move_goal_pose_client('arm', [0.45, 0, 0.8], [0.707, 0.0, -0.707, 0.0])
    att_box_client('box1', 'hand')
    move_goal_pose_client('arm', [0.25, 0, 0.4], [0.707, 0.0, -0.707, 0.0]) # home
    move_goal_pose_client('arm', [0.45, 0, 0.478], [0.707, 0.0, -0.707, 0.0])
    det_box_client('box1')
    move_goal_pose_client('arm', [0.25, 0, 0.4], [0.707, 0.0, -0.707, 0.0])