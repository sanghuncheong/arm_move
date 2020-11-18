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
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import numpy as np


from arm_move.msg._arm_move_msg import arm_move_msg
from arm_move.msg._box_info_msg import box_info_msg
from arm_move.msg._attach_hand_box import attach_hand_box

from arm_move.srv import *
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

import matplotlib.pyplot as plt
from std_msgs.msg import String
import sensor_msgs.msg
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

from moveit_msgs.msg._RobotTrajectory import RobotTrajectory
from arm_move.msg._arm_move_msg import arm_move_msg


import actionlib
import actionlib_tutorials.msg
import actionlib_tutorials.msg._JointData
import actionlib_tutorials.msg._JointDataSet

# import ros_podo_connector.msg
from arm_move.srv._arm_move_srv import *
import D_0902_client_function as CLF
from obj_msg.msg import *

# ACTION TEST

def convert2action_arm(plan, move_group):
    # print type(plan), plan.joint_names, plan.joint_names
    # for i in range(10):
    #     print i, "th time:", type(plan.points[i].time_from_start.to_sec()), plan.points[i].time_from_start.to_sec()
    #     print i, "th pos:", plan.points[i].positions
    time.sleep(1)
    act_client = actionlib.SimpleActionClient("rospodo_trajectory", ros_podo_connector.msg.RosPODO_TrajectoryAction)
    act_client.wait_for_server()

    action_goal = actionlib_tutorials.msg.RosPODO_TrajectoryGoal()
    print "action length:", len(action_goal.via_point)

    action_goal.num_points = len(plan.points)
    action_goal.planGroup = move_group

    # for i in range(4):
    #     for j in range(18):
    #         print action_goal.via_point[i].joint[j]
    NUM_JOINTS = 18
    joint_buffer_list = ["RSP", "RSR", "RSY", "REB", "RWY", "RWP", "RWY2","LSP", "LSR", "LSY", "LEB", "LWY", "LWP", "LWY2", "WST", "RWH", "LWH", "BWH"]
    for point_i in range(len(plan.points)):
        for joint_i in range(NUM_JOINTS):
            for joint_tra_i in range(len(plan.joint_names)):
                # only if joint name matches
                if joint_buffer_list[joint_i] == plan.joint_names[joint_tra_i]:
                    action_goal.via_point[point_i].joint[joint_i].OnOffControl = 1
                    action_goal.via_point[point_i].joint[joint_i].reference = plan.points[point_i].positions[joint_tra_i]
                    if point_i > 0:
                        action_goal.via_point[point_i].joint[joint_i].GoalmsTime = plan.points[point_i].time_from_start.to_sec()-plan.points[point_i-1].time_from_start.to_sec()
                    else:
                        action_goal.via_point[point_i].joint[joint_i].GoalmsTime = 0.0
                    break
                else:
                    action_goal.via_point[point_i].joint[joint_i].OnOffControl = 0
                    action_goal.via_point[point_i].joint[joint_i].reference = 0.
                    action_goal.via_point[point_i].joint[joint_i].GoalmsTime = 0.
    for i in range(5):
        print "action goal", action_goal.via_point[i].joint[0].GoalmsTime
    #time.sleep(1)
    act_client.send_goal(action_goal)
    act_client.wait_for_result()
    global finish
    finish = True
    return 1


def try_path_plan(goal_pos, app_angle):
    import math
    from tf.transformations import quaternion_from_euler, quaternion_multiply

    # planner_name = 'RRTConnect'
    planner_name = 'BiTRRT'
    n_attempt = 1000
    c_time = 0.5
    n_repeat = 10
    start_state = moveit_msgs.msg.RobotState()
    joint_state = sensor_msgs.msg.JointState()
    joint_state.header = std_msgs.msg.Header()
    joint_state.name = ['j2n6s300_joint_1', 'j2n6s300_joint_2', 'j2n6s300_joint_3', 'j2n6s300_joint_4', 'j2n6s300_joint_5', 'j2n6s300_joint_6']
    # joint_state.position = current_joints
    joint_state.position = [3.1415927410125732, 4.537856101989746, 5.93411922454834, -0.6108652353286743, 1.7453292608261108, -0.5235987901687622]

    # joint_state.position = [3.1415927410125732, 4.537856101989746, 5.93411922454834, -0.6108652353286743, 1.7453292608261108, -0.5235987901687622]
    start_state.joint_state = joint_state
    # goal_pose:
    # goal_orientation:
    # goal_pose = [self.object_z, -yi, xi - 0.05]
    # goal_orientation = [-0.00145713772037, -0.998970756926, 0.0364956710831, 0.0268955302573]
    # z = goal_pos[2]
    # goal_pose = [z + 0.02, -yi, xi]
    # Set the grasp pose: substract 17cm from the z value of the object centroid
    goal_pitches = []
    goal_pitch = np.deg2rad(app_angle)
    # goal_pitch = vfh[-1] + math.pi/2
    goal_pitches.append(goal_pitch)  # approaching_angle: vfh[-1] from the input
    for i in range(3):
        goal_pitches.append(goal_pitch + (i + 1) * math.radians(8.0))
        goal_pitches.append(goal_pitch - (i + 1) * math.radians(8.0))
    # Get the grasp orientation (currently the front direction)
    goal_orientations = []

    # for i in goal_pitches:
    #     no_pitch_ori = quaternion_from_euler(math.radians(-90.0), math.radians(-90.0) + i, 0, axes='rxyz')
    #     goal_orientations.append(no_pitch_ori)

    for i in goal_pitches:
        no_pitch_ori = quaternion_from_euler(math.radians(-90.0), math.radians(-90.0) + i, 0, axes='rxyz')
        pitch_ori = quaternion_from_euler(math.radians(5), 0.0, 0.0, axes='rxyz')
        fin_ori = quaternion_multiply(no_pitch_ori, pitch_ori)
        goal_orientations.append(fin_ori)

    l = 0.24
    # l = 0.27
    # l = 0.17
    goal_poses = []
    for i in goal_pitches:
        dx = math.sin(i - math.pi) * l
        dy = math.cos(i - math.pi) * l
        goal_poses.append([goal_pos[0] + dy, goal_pos[1] - dx, goal_pos[2]])

    feasibility1 = 0
    i = 0
    while not feasibility1 and i < len(goal_pitches):
        # CLF.add_box_client('can_bottom_x', [goal_poses[i][0], goal_poses[i][1], goal_poses[i][2]], goal_orientations[i], [0.1, 0.005, 0.005], 'red')
        # CLF.add_box_client('can_bottom_y', [goal_poses[i][0], goal_poses[i][1], goal_poses[i][2]], goal_orientations[i], [0.005, 0.1, 0.005], 'green')
        # CLF.add_box_client('can_bottom_z', [goal_poses[i][0], goal_poses[i][1], goal_poses[i][2]], goal_orientations[i], [0.005, 0.005, 0.1], 'blue')
        print i, "th try", goal_pitches[i], "added"
        [feasibility1, trajectory1] = CLF.move_goalpose_client('R_arm', 'R_Hand', start_state, goal_poses[i], goal_orientations[i], [], planner_name, n_attempt, c_time, n_repeat)
	idx = i
        i = i + 1
        # time.sleep(1)
        # CLF.del_box_client('can_bottom_x')
        # CLF.del_box_client('can_bottom_y')
        # CLF.del_box_client('can_bottom_z')

    #retGoal = convert2action_arm(trajectory1.joint_trajectory, 'R_arm')
    return math.degrees(goal_pitches[idx])#[goal_poses[i-1], goal_orientations[i-1]]


class tmi(object):
    """MoveGroupPythonIntefaceTutorial"""

    def __init__(self):
        super(tmi, self).__init__()

    def do_Hand_Closeact(self, data, timeout=4):
        rospy.sleep(0.1)
        gripper_cmd = 'close'
        gripper_close(gripper_cmd)
        #rep1 = ActionServiceResponse(result=0)
        return 1

    def do_Hand_Openact(self, data, timeout=4):
        rospy.sleep(0.1)
        gripper_cmd = 'open'
        gripper_open(gripper_cmd)
        #rep1 = ActionServiceResponse(result=0)
        return 1

    def make_env_shelf(self, dx, dy):

        self.env_name = ['shelf0', 'shelf1', 'shelf2', 'side0', 'side1', 'side2']
        env_info = []
        env_info.append([[0 + dx, 0 + dy, 0.90/2.0], [0, 0, 0, 0], [0.45 + 0.1, 0.95 + 0.1, 0.90]])  # information for shelf0
        env_info.append([[0 + dx, 0 + dy, 1.30 - 0.018/2.0], [0, 0, 0, 0], [0.45 + 0.1, 0.95 + 0.1, 0.018]])  # information for shelf1
        env_info.append([[0 + dx, 0 + dy, 1.50 - 0.018/2.0], [0, 0, 0, 0], [0.45 + 0.1, 0.95 + 0.1, 0.018]])  # information for shelf2

        env_info.append([[0 + dx, 0 + dy - 0.95/2.0 + 0.018/2.0, 1.50/2.0], [0, 0, 0, 0], [0.45 + 0.1, 0.018 + 0.05, 1.5]])  # information for side0
        env_info.append([[0 + dx + 0.45/2.0 - 0.018/2.0, 0 + dy, 1.50/2.0], [0, 0, 0, 0], [0.018 + 0.05, 0.95 + 0.1, 1.5]])  # information for side1
        env_info.append([[0 + dx, 0 + dy + 0.95/2.0 - 0.018/2.0, 1.50/2.0], [0, 0, 0, 0], [0.45 + 0.1, 0.018 + 0.05, 1.5]])  # information for side2
        for i in range(len(env_info)):
            CLF.add_box_client(self.env_name[i], env_info[i][0], env_info[i][1], env_info[i][2], 'gray')
        return 1

    def make_env_vis_shelf(self, dx, dy):

        self.env_name = ['shelf0', 'shelf1', 'shelf2', 'side0', 'side1', 'side2']
        env_info = []
        env_info.append([[0 + dx, 0 + dy, 0.90/2.0], [0, 0, 0, 0], [0.45, 0.95, 0.90]])  # information for shelf0
        env_info.append([[0 + dx, 0 + dy, 1.30 - 0.018/2.0], [0, 0, 0, 0], [0.45, 0.95, 0.018]])  # information for shelf1
        env_info.append([[0 + dx, 0 + dy, 1.50 - 0.018/2.0], [0, 0, 0, 0], [0.45, 0.95, 0.018]])  # information for shelf2

        env_info.append([[0 + dx, 0 + dy - 0.95/2.0 + 0.018/2.0, 1.50/2.0], [0, 0, 0, 0], [0.45, 0.018, 1.5]])  # information for side0
        env_info.append([[0 + dx + 0.45/2.0 - 0.018/2.0, 0 + dy, 1.50/2.0], [0, 0, 0, 0], [0.018 , 0.95, 1.5]])  # information for side1
        env_info.append([[0 + dx, 0 + dy + 0.95/2.0 - 0.018/2.0, 1.50/2.0], [0, 0, 0, 0], [0.45, 0.018, 1.5]])  # information for side2
        for i in range(len(env_info)):
            CLF.add_box_client(self.env_name[i], env_info[i][0], env_info[i][1], env_info[i][2], 'gray')
        return 1
    def del_shelf(self):
        for i in self.env_name:
            CLF.del_box_client(i)

    def make_env_counter(self, dx, dy):
        env_name = ['counter']
        env_info = []
        env_info.append([[0 + dx, 0 + dy, 0.90/2.0], [0, 0, 0, 0], [0.41+0.17, 0.8+0.17, 0.90]])  # information for counter
        for i in range(len(env_info)):
            CLF.add_box_client(env_name[i], env_info[i][0], env_info[i][1], env_info[i][2], 'gray')
        return 1
    
    def get_data_topic(self, data):
        # print "data", data
        for i in range(len(data.gr)):
            if data.gr[i].id == 1:
                data_return = [data.gr[i].grasp_cx, data.gr[i].grasp_cy + 0.04, data.gr[i].grasp_cz]
                self.target_xyz = data_return
            else:
                self.obs_xyz = [data.gr[i].grasp_cx, data.gr[i].grasp_cy + 0.04, data.gr[i].grasp_cz]
    
        # data_return = [data.gr[0].grasp_cx, data.gr[0].grasp_cy, 0.96]
    

def listener():

    #moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('tmi_node', anonymous=True)

    # =================== service!! =======================

    rospy.Service('Generate_pose', PoseService, tutorial.gen_pose)
    rospy.Service('Find_trajectory', PredicateService, tutorial.fin_tra)
    rospy.Service('Do_BaseAction', ActionService, tutorial.do_Baseact)
    rospy.Service('Do_PreGraspAction', ActionService, tutorial.do_PreGraspact)
    rospy.Service('Do_GraspAction', ActionService, tutorial.do_Graspact)
    rospy.Service('Do_OpenAction', ActionService, tutorial.do_Hand_Openact)
    rospy.Service('Do_CloseAction', ActionService, tutorial.do_Hand_Closeact)

    rospy.Service('Do_ReleaseBaseAction', ActionService, tutorial.do_Base_Returnact)
    rospy.Service('Do_ReleaseGraspAction', ActionService, tutorial.do_Rel_Graspact)

    rospy.Service('Do_StorageBaseAction', ActionService, tutorial.do_Base_storageact)
    rospy.Service('Do_StorageBaseReturnAction', ActionService, tutorial.do_Base_Return_storageact)

    rospy.Service('Do_MOVEit_env_create', ActionService, tutorial.do_MOVEit_env_create)
    rospy.Service('Do_MOVEit_env_create_2', ActionService, tutorial.do_MOVEit_env_create_2)
    rospy.Service('Do_MOVEit_env_apply', ActionService, tutorial.do_MOVEit_env_apply)
    # rospy.Service('Do_MOVEit_env_delete', ActionService, tutorial.do_MOVEit_env_delete)
    rospy.Service('Do_MOVEit_env_remove_all', ActionService, tutorial.do_MOVEit_env_removeall)

    rospy.spin()

def get_data():
        time.sleep(0.2)
        a = rospy.Subscriber('/gr_infos', GraspArray, tutorial.get_data_topic)
        time.sleep(0.2)

def test_navigation(gx, gy):
    import rospy
    import geometry_msgs.msg
    
    pub = rospy.Publisher('/move_base_simple/goal_opc', geometry_msgs.msg.PoseStamped)
    time.sleep(0.1)
    pub_msg = geometry_msgs.msg.PoseStamped()
   
    pub_msg.header.stamp.secs = 0.0
    pub_msg.header.stamp.nsecs = 0.0
    pub_msg.header.frame_id = ""

    pub_msg.pose.position.x = gx
    pub_msg.pose.position.y = gy
    pub_msg.pose.position.z = 0.0

    pub_msg.pose.orientation.x = 0.0
    pub_msg.pose.orientation.y = 0.0
    pub_msg.pose.orientation.z = 0.0
    pub_msg.pose.orientation.w = 0.0

    rospy.loginfo(pub_msg)
    for i in range(100):
        pub.publish(pub_msg)


def walk_ready():
    import copy
    # This part is for test without TM

    gp3 = [0.1660 + 0.27, -0.2465, 0.7629]
    # home
    try_path_plan(gp3, -25)


def test_tmi():
    import copy
    # This part is for test without TM
    rospy.init_node('tmi_node', anonymous=True)

    walk_ready()
    tutorial.target_xyz = [0.80, -0.25, 0.963]
    app_angles = [25, -5, -35]
    for i in app_angles:
        tutorial.make_env_vis_shelf(0.9, -0.12)    #make shelf at point
        # tutorial.make_env_shelf(0.9, -0.12)    #make shelf at point
        gp1 = copy.deepcopy(tutorial.target_xyz)
        CLF.add_box_client('target', gp1, [0, 0, 0, 0], [0.065+0.02, 0.065+0.02, 0.17+0.02], 'red') #cantata

        try_path_plan(gp1, i)
        time.sleep(0.01)
        CLF.att_box_client('R_hand', 'target')

        gp3 = [0.1660+0.27, -0.2465, 0.7629]
        #home
        try_path_plan(gp3, -25)
        time.sleep(3)

        CLF.det_box_client('target', gp1, [0, 0, 0, 0], [0.065+0.02, 0.065+0.02, 0.17+0.02], 'red')
        CLF.del_box_client('target')


if __name__ == '__main__':

    global currentJointPose
    global currentBasePose
    global currentGrasperPose
    global currentObjectPose
    global currentGrasperOpenClose
    global joint_traj
    global planned_path
    global count
    global pointcloud
    global bounded_pointcloud
    
    ### Initialize variables ###
    print "node tmi start!"
    feasibility = 4
    rospy.init_node('tmi_node', anonymous=True)

    WORKSPACE_RADIUS = 1.2 #in meters
    PREGRASP_DISTANCE = 0.1 #in meters

    sample_range_base_distance = WORKSPACE_RADIUS/5
    sample_range_base_angle = np.deg2rad(360) #in radians

    sample_range_grasper_distance = PREGRASP_DISTANCE/2 #in meters
    sample_range_grasper_angle = np.deg2rad(60) #in radians

    Nsample_base = 100
    Nsample_grasper = 100
    
    tutorial = tmi()
    object_list = []

    # listener()
    test_tmi()
    # test_navigation(0.0, 1.5)
    # test_navigation(0.0, -0.2)
    print "End node!!"

