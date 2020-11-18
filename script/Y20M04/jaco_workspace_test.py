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

from arm_move.srv import *
import S_client_function as CLF

def test_ws():
    # This part is for test without TM
    rospy.init_node('check_ws', anonymous=True)

    # make a table for checking the workspace of the JACO
    table_pos = [0.5, 0.0, 0.2]
    table_ori = [0.0, 0.0, 0.0, 0.0]
    table_scale = [0.5, 0.5, 0.2]
    CLF.add_box_client('table', table_pos, table_ori, table_scale, 'gray')

    # make an object to check the accessibility
    target_pos = [0.5, 0.0, 0.36]
    target_ori = [0.0, 0.0, 0.0, 0.0]
    target_scale = [0.06, 0.06, 0.12]
    CLF.add_box_client('target', target_pos, target_ori, target_scale, 'red')

    goal_pos = target_pos
    goal_ori = [0.50, 0.5, 0.5, 0.5]

    planner_name = 'RRTConnect'
    # planner_name = 'BiTRRT'
    n_attempt = 1000
    c_time = 2
    n_repeat = 1
    start_state = moveit_msgs.msg.RobotState()
    joint_state = sensor_msgs.msg.JointState()
    joint_state.header = std_msgs.msg.Header()
    joint_state.name = ['j2n6s300_joint_base', 'j2n6s300_joint_1', 'j2n6s300_joint_2', 'j2n6s300_joint_3', 'j2n6s300_joint_4', 'j2n6s300_joint_5', 'j2n6s300_joint_6', 'j2n6s300_joint_end_effector']
    CLF.move_goalpose_client('arm', 'gripper', start_state, goal_pos, goal_ori, [], planner_name, n_attempt, c_time, n_repeat)
    # CLF.feasible_check_obj_joint_client('arm', 'gripper', start_state, goal_pos, goal_ori, [], planner_name, n_attempt, c_time, n_repeat)

    goal_pos = [0.1, -0.3, 0.3]
    goal_ori = [0.50, 0.5, 0.5, 0.5]
    CLF.move_goalpose_client('arm', 'gripper', start_state, goal_pos, goal_ori, [], planner_name, n_attempt, c_time, n_repeat)


if __name__ == '__main__':

    test_ws()  # This one has objects that we  have to considered the approaching direction.
    print "End node!!"

