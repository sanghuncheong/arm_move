#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt! interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##
import time
import sys
import copy
import rospy
import moveit_msgs.msg
import sys
import numpy as np
import cv2 as cv
from arm_move.srv import *
import S_client_function as CLF


def test_ws_3(x,y,z,i,j,t_w,t_d,t_h,tr_d,tr_h):
    # This part is for test without TM
    rospy.init_node('check_ws', anonymous=True)

    # make a table for checking the workspace of the JACO
    table_pos = [tr_d + t_d/2.0, 0.0, tr_h]
    table_ori = [0.0, 0.0, 0.0, 0.0]
    table_scale = [t_d, t_w, t_h]
    CLF.add_box_client('table', table_pos, table_ori, table_scale, 'green')

    # make a shelf on the table
    shelf_env = 0
    shelf_w = 0.005
    shelf_h = 0.38  # this is the height of the shelf in the lab
    if shelf_env == 1:
        shelf_r_pos = [table_pos[0], table_pos[1] - t_w/2 - shelf_w, (shelf_h+t_h)/2.0+table_pos[2]]
        shelf_l_pos = [table_pos[0], table_pos[1] + t_w/2 + shelf_w, (shelf_h+t_h)/2.0+table_pos[2]]
        shelf_b_pos = [table_pos[0] + t_w/2 + shelf_w, table_pos[1], (shelf_h+t_h)/2.0+table_pos[2]]
        shelf_u_pos = [table_pos[0], table_pos[1], shelf_h+(t_h)/2.0+table_pos[2] + shelf_w]

        shelf_r_ori = table_ori
        shelf_l_ori = table_ori
        shelf_b_ori = table_ori
        shelf_u_ori = table_ori

        shelf_r_scale = [t_d, shelf_w*2, shelf_h]
        shelf_l_scale = [t_d, shelf_w*2, shelf_h]
        shelf_b_scale = [shelf_w*2, t_w, shelf_h]
        shelf_u_scale = [t_d, t_w, shelf_w*2]

        CLF.add_box_client('shelf_l', shelf_r_pos, shelf_r_ori, shelf_r_scale, 'green')
        CLF.add_box_client('shelf_r', shelf_l_pos, shelf_l_ori, shelf_l_scale, 'green')
        CLF.add_box_client('shelf_u', shelf_u_pos, shelf_u_ori, shelf_u_scale, 'green')
        CLF.add_box_client('shelf_b', shelf_b_pos, shelf_b_ori, shelf_b_scale, 'green')

    # make an object to check the accessibility
    target_pos = [x,y,z]
    # target_pos = [0.5, 0.0, 0.36]
    target_ori = [0.0, 0.0, 0.0, 0.0]
    target_scale = [0.06, 0.06, 0.12]
    CLF.add_box_client('target', target_pos, target_ori, target_scale, 'red')

    RC = 1
    obs_dis=0.15
    obstacle_xyz_1 = [x + np.random.random_integers(-RC, RC) / 1000.0, y+obs_dis + np.random.random_integers(-RC, RC) / 1000.0, z]
    obstacle_xyz_2 = [x + np.random.random_integers(-RC, RC) / 1000.0, y-obs_dis + np.random.random_integers(-RC, RC) / 1000.0, z]

    box_ori = [0, 0, 0, 0]
    CLF.add_box_client('obstacle1', obstacle_xyz_1, box_ori, [0.06, 0.06, 0.12], 'green')
    CLF.add_box_client('obstacle2', obstacle_xyz_2, box_ori, [0.06, 0.06, 0.12], 'green')

    goal_pos = target_pos
    goal_ori1 = [0.50, 0.5, 0.5, 0.5]
    goal_ori2 = [0.0, 0.0, -0.383, 0.924] # left_45degree
    goal_ori3 = [0.0, 0.0, 0.383, 0.924] # right_45degree

    result1 = try_path_plan_dir(goal_pos, app_angle=0, angle_interval=8, n_angle_try=3)

    print "%d,%d,%d" % (i,j,result1[0]) #from right lower in robot view


    # goal_pos = [0.1, -0.3, 0.3]
    # goal_ori = [0.50, 0.5, 0.5, 0.5]
    # CLF.move_goalpose_client('arm', 'gripper', start_state, goal_pos, goal_ori, [], planner_name, n_attempt, c_time, n_repeat)

    return result1[0]


def try_path_plan_dir(goal_pos, app_angle, angle_interval, n_angle_try):
    #  The number of total try angle = 1 + 2*n_angle_try
    import math
    from tf.transformations import quaternion_from_euler
    import time
    # planner_name = 'RRTstar'
    planner_name = 'RRTConnect'
    # planner_name = 'BiTRRT'
    n_attempt = 1
    c_time = 0.1
    n_repeat = 2
    start_state = moveit_msgs.msg.RobotState()
    joint_state = sensor_msgs.msg.JointState()
    joint_state.header = std_msgs.msg.Header()
    joint_state.name = ['j2n6s300_joint_base', 'j2n6s300_joint_1', 'j2n6s300_joint_2', 'j2n6s300_joint_3', 'j2n6s300_joint_4', 'j2n6s300_joint_5', 'j2n6s300_joint_6', 'j2n6s300_joint_end_effector']

    # Set the grasp pose: substract 17cm from the z value of the object centroid
    goal_pitches = []
    goal_pitch = np.deg2rad(app_angle)
    goal_pitches.append(goal_pitch)
    for i in range(n_angle_try):
        goal_pitches.append(goal_pitch + (i + 1) * math.radians(angle_interval))
        goal_pitches.append(goal_pitch - (i + 1) * math.radians(angle_interval))
    # Get the grasp orientation (currently the front direction)
    goal_orientations = []
    for i in goal_pitches:
        goal_orientations.append(quaternion_from_euler(math.radians(90.0), math.radians(90.0) + i, 0, axes='rxyz'))
        # goal_orientations.append(quaternion_from_euler(math.radians(-90.0), math.radians(-90.0) + i, 0, axes='rxyz'))

    l = 0.03  #  for jaco_moveit_4/demo.launch
    goal_poses = []
    for i in goal_pitches:
        dx = math.cos(i) * l
        dy = math.sin(i) * l
        goal_poses.append([goal_pos[0] - dx, goal_pos[1] + dy, goal_pos[2]])

    feasibility1 = 0
    i = 0
    while not feasibility1 and i < len(goal_pitches):
        # print i, "th try", goal_pitches[i], "added"
        [feasibility1, trajectory1] = CLF.feasible_check_obj_joint_client('arm', 'gripper', start_state, goal_poses[i], goal_orientations[i], [], planner_name, n_attempt, c_time, n_repeat)
        idx = i
        i = i + 1

    return [feasibility1, trajectory1]


def test_ws_2(x,y,z,i,j,t_w,t_d,t_h,tr_d,tr_h):
    # This part is for test without TM
    rospy.init_node('check_ws', anonymous=True)

    # make a table for checking the workspace of the JACO
    table_pos = [tr_d + t_d/2.0, 0.0, tr_h]
    table_ori = [0.0, 0.0, 0.0, 0.0]
    table_scale = [t_d, t_w, t_h]
    CLF.add_box_client('table', table_pos, table_ori, table_scale, 'green')

    # make a shelf on the table
    shelf_env = 1
    shelf_w = 0.005
    shelf_h = 0.38  # this is the height of the shelf in the lab
    if shelf_env == 1:
        shelf_r_pos = [table_pos[0], table_pos[1] - t_w/2 - shelf_w, (shelf_h+t_h)/2.0+table_pos[2]]
        shelf_l_pos = [table_pos[0], table_pos[1] + t_w/2 + shelf_w, (shelf_h+t_h)/2.0+table_pos[2]]
        shelf_b_pos = [table_pos[0] + t_w/2 + shelf_w, table_pos[1], (shelf_h+t_h)/2.0+table_pos[2]]
        shelf_u_pos = [table_pos[0], table_pos[1], shelf_h+(t_h)/2.0+table_pos[2] + shelf_w]

        shelf_r_ori = table_ori
        shelf_l_ori = table_ori
        shelf_b_ori = table_ori
        shelf_u_ori = table_ori

        shelf_r_scale = [t_d, shelf_w*2, shelf_h]
        shelf_l_scale = [t_d, shelf_w*2, shelf_h]
        shelf_b_scale = [shelf_w*2, t_w, shelf_h]
        shelf_u_scale = [t_d, t_w, shelf_w*2]

        CLF.add_box_client('shelf_l', shelf_r_pos, shelf_r_ori, shelf_r_scale, 'green')
        CLF.add_box_client('shelf_r', shelf_l_pos, shelf_l_ori, shelf_l_scale, 'green')
        CLF.add_box_client('shelf_u', shelf_u_pos, shelf_u_ori, shelf_u_scale, 'green')
        CLF.add_box_client('shelf_b', shelf_b_pos, shelf_b_ori, shelf_b_scale, 'green')

    # make an object to check the accessibility
    target_pos = [x,y,z]
    # target_pos = [0.5, 0.0, 0.36]
    target_ori = [0.0, 0.0, 0.0, 0.0]
    target_scale = [0.06, 0.06, 0.12]
    CLF.add_box_client('target', target_pos, target_ori, target_scale, 'red')

    goal_pos = target_pos
    goal_ori1 = [0.50, 0.5, 0.5, 0.5]
    goal_ori2 = [0.0, 0.0, -0.383, 0.924] # left_45degree
    goal_ori3 = [0.0, 0.0, 0.383, 0.924] # right_45degree

    result1 = try_path_plan_dir(goal_pos, app_angle=0, angle_interval=8, n_angle_try=3)

    print "%d,%d,%d" % (i,j,result1[0]) #from right lower in robot view


    # goal_pos = [0.1, -0.3, 0.3]
    # goal_ori = [0.50, 0.5, 0.5, 0.5]
    # CLF.move_goalpose_client('arm', 'gripper', start_state, goal_pos, goal_ori, [], planner_name, n_attempt, c_time, n_repeat)

    return result1[0]


def test_ws(x,y,z,i,j,t_w,t_d,t_h,tr_d,tr_h):
    # This part is for test without TM
    rospy.init_node('check_ws', anonymous=True)

    # make a table for checking the workspace of the JACO
    table_pos = [tr_d + t_d/2.0, 0.0, tr_h]
    table_ori = [0.0, 0.0, 0.0, 0.0]
    table_scale = [t_d, t_w, t_h]
    CLF.add_box_client('table', table_pos, table_ori, table_scale, 'green')

    # make an object to check the accessibility
    target_pos = [x,y,z]
    # target_pos = [0.5, 0.0, 0.36]
    target_ori = [0.0, 0.0, 0.0, 0.0]
    target_scale = [0.06, 0.06, 0.12]
    CLF.add_box_client('target', target_pos, target_ori, target_scale, 'red')

    goal_pos = target_pos
    goal_ori1 = [0.50, 0.5, 0.5, 0.5]
    goal_ori2 = [0.0, 0.0, -0.383, 0.924] # left_45degree
    goal_ori3 = [0.0, 0.0, 0.383, 0.924] # right_45degree

    # planner_name = 'RRTstar'
    planner_name = 'RRTConnect'
    # planner_name = 'BiTRRT'
    n_attempt = 1
    c_time = 1
    n_repeat = 2
    start_state = moveit_msgs.msg.RobotState()
    joint_state = sensor_msgs.msg.JointState()
    joint_state.header = std_msgs.msg.Header()
    joint_state.name = ['j2n6s300_joint_base', 'j2n6s300_joint_1', 'j2n6s300_joint_2', 'j2n6s300_joint_3', 'j2n6s300_joint_4', 'j2n6s300_joint_5', 'j2n6s300_joint_6', 'j2n6s300_joint_end_effector']

    # CLF.move_goalpose_client('arm', 'gripper', start_state, goal_pos, goal_ori1, [], planner_name, n_attempt, c_time, n_repeat)
    result1 = CLF.feasible_check_obj_joint_client('arm', 'gripper', start_state, goal_pos, goal_ori1, [], planner_name, n_attempt, c_time, n_repeat)

    print "%d,%d,%d" % (i,j,result1[0]) #from right lower in robot view


    # goal_pos = [0.1, -0.3, 0.3]
    # goal_ori = [0.50, 0.5, 0.5, 0.5]
    # CLF.move_goalpose_client('arm', 'gripper', start_state, goal_pos, goal_ori, [], planner_name, n_attempt, c_time, n_repeat)

    return result1[0]


def draw_workspace(t_r,table,t_w,t_h):
    blue_color = (255, 0, 0)
    green_color = (0, 255, 0)
    red_color = (0, 0, 255)
    white_color = (255, 255, 255)

    # set canvas size to draw workspace
    img_size = 100
    img_bound = 20
    img = np.zeros((img_size+img_bound, img_size+img_bound, 3), np.uint8) + 255 # index 3 for RGB
    for i in range(len(table[0])):
        for j in range(len(table[1])):
            if table[i][j] == 0:
                img = cv.line(img, (i*int(img_size/(t_r-1))+img_bound/2, j*int(img_size/(t_r-1))+img_bound/2), (i*int(img_size/(t_r-1))+img_bound/2, j*int(img_size/(t_r-1))+img_bound/2), red_color, 4)

    img = cv.line(img, (img_bound/2, img_bound/2), (img_bound/2 + img_size, img_bound/2), (0, 0, 0), 2)
    img = cv.line(img, (img_bound/2 + img_size, img_bound/2), (img_bound/2 + img_size, img_bound/2 + img_size), (0, 0, 0), 2)
    img = cv.line(img, (img_bound/2 + img_size, img_bound/2 + img_size), (img_bound/2, img_bound/2 + img_size), (0, 0, 0), 2)
    img = cv.line(img, (img_bound/2, img_bound/2 + img_size), (img_bound/2, img_bound/2), (0, 0, 0), 2)

    cv.imshow('image', img)
    cv.waitKey(0)
    cv.destroyAllWindows()
    cv.imwrite('workspace_2.jpg', img)

if __name__ == '__main__':

    sys.stdout = open('output_2.txt', 'a')
    table_resolution = 3  # need to define width, depth resolution
    table_width = 1.0
    table_depth = 1.0
    table_height = 0.2
    tablerobot_distance = 0.25
    tablerobot_height = 0.3

    print "table_resolution, table_width, table_depth, table_height, table_robot_distance, table_robot_height"
    print table_resolution, table_width, table_depth, table_height, tablerobot_distance, tablerobot_height

    CLF.move_hand_joint_client([0.3, 0.3, 0.3])
    table = np.ones((table_resolution, table_resolution))
    for i in range(0,table_resolution):
        for j in range(0,table_resolution):
            x = tablerobot_distance + table_width/(table_resolution-1)*i
            y = -table_width/2.0 + table_depth/(table_resolution-1)*j
            z = (tablerobot_height + table_height/2.0) + 0.06 # 0.06=object height/2.0
            # test_ws_2 : can make environment with or without shelf and many approaching angles.
            # test_ws_3 : target object with obstacles next to it.
            feasiblility = test_ws_2(x,y,z,i,j,table_width,table_depth,table_height,tablerobot_distance,tablerobot_height)  # This one has objects that we  have to considered the approaching direction.
            # feasiblility = test_ws_3(x,y,z,i,j,table_width,table_depth,table_height,tablerobot_distance,tablerobot_height)  # This one has objects that we  have to considered the approaching direction.
            # feasiblility = test_ws(x,y,z,i,j,table_width,table_depth,table_height,tablerobot_distance,tablerobot_height)  # This one has objects that we  have to considered the approaching direction.

            if feasiblility == 0:
                table[(table_resolution-1)-j,(table_resolution-1)-i] = 0
                # table[j,i] = 0

    draw_workspace(table_resolution,table,table_width,table_height)

    print "End node!!"

