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
from arm_move.msg._arm_move_msg import arm_move_msg
from arm_move.msg._box_info_msg import box_info_msg
from arm_move.msg._attach_hand_box import attach_hand_box

from arm_move.srv._box_info_srv import *
from arm_move.srv._att_hand_box_srv import *
from arm_move.srv._arm_move_srv import *
from arm_move.srv._work_start_srv import *
from arm_move.srv._arm_goalJoint_srv import *

from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

## END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):
    """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class MoveGroupPythonIntefaceTutorial(object):
    """MoveGroupPythonIntefaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonIntefaceTutorial, self).__init__()

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        self.group_name = 'r_arm' # this is just for the initialization
        move_group = moveit_commander.MoveGroupCommander(self.group_name)
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)
        traj_arm_publisher = rospy.Publisher('/traj_arm',moveit_msgs.msg.RobotTrajectory, queue_size=100)
        feasibility_flag = rospy.Publisher('/arm_feasibility',String,queue_size=20)

        planning_frame = move_group.get_planning_frame()
        # eef_link = move_group.get_end_effector_link()
        eef_link = 'Body_RF2_c3'
        print "end eef link list\n", eef_link
        group_names = robot.get_group_names()

        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.traj_arm_publisher = traj_arm_publisher
        self.feasibility_flag = feasibility_flag
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.object_list = []

    def go_to_pose_goal_m(self, data):

        move_group = moveit_commander.MoveGroupCommander(data.arm_name[0])
        # move_group = self.move_group
        pose_goal = geometry_msgs.msg.Pose()

        pose_goal.position = data.goal_position
        pose_goal.orientation = data.goal_orientation
        # move_group.set_planner_id('SPARStwo')
        # move_group.set_planner_id('RRTstar')
        # move_group.set_planner_id('BiTRRT')

        move_group.set_num_planning_attempts(10000)
        move_group.set_planning_time(5)
        move_group.set_goal_position_tolerance(0.01)
        move_group.set_goal_orientation_tolerance(0.01)

        move_group.set_pose_target(pose_goal)

        print "goal pose:", pose_goal

        plan = move_group.plan()
        traj_arm_pub = self.traj_arm_publisher
        traj_arm_pub.publish(plan)

        move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def goalPose_feasibility_check_m(self, data):

        # move_group = self.move_group
        move_group = moveit_commander.MoveGroupCommander(data.arm_name[0])
        pose_goal = geometry_msgs.msg.Pose()

        pose_goal.position = data.position
        pose_goal.orientation = data.orientation
        # move_group.set_planner_id('SPARStwo')
        # move_group.set_planner_id('RRTstar')
        # move_group.set_planner_id('BiTRRT')

        move_group.set_num_planning_attempts(10000)
        move_group.set_planning_time(5)
        move_group.set_goal_position_tolerance(0.01)
        move_group.set_goal_orientation_tolerance(0.01)

        move_group.set_pose_target(pose_goal)
        plan = move_group.plan()
        # print plan
        print "plan.joint_trajectory.joint_names :", plan.joint_trajectory.joint_names

        feasibility_flag_pub = self.feasibility_flag
        feasible_flag_msg = String()
        if len(plan.joint_trajectory.joint_names) == 0:
            print "no plan found"
            feasible_flag_msg = '0'
            feasibility_flag_pub.publish(feasible_flag_msg)
        elif len(plan.joint_trajectory.joint_names) > 0:
            print "plan found"
            feasible_flag_msg = '1'
            feasibility_flag_pub.publish(feasible_flag_msg)
            time.sleep(3)
            traj_arm_pub = self.traj_arm_publisher
            traj_arm_pub.publish(plan)

        time.sleep(2)
        move_group.stop()
        move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def pickup(self, upCM, scale=1):

        move_group = self.move_group
        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z -= -0.05  # First move up (z)
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold

        move_group.execute(plan, wait=True)
        return plan, fraction

    def display_trajectory(self, plan):

        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory);


    def execute_plan(self, plan):

        move_group = self.move_group
        move_group.execute(plan, wait=True)

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):

        box_name = self.box_name
        scene = self.scene

        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            is_known = box_name in scene.get_known_object_names()

            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            rospy.sleep(0.1)
            seconds = rospy.get_time()

        return False
        ## END_SUB_TUTORIAL

    def add_box(self, timeout=4):

        box_name = self.box_name
        scene = self.scene
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "/base_footprint"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.07  # slightly above the end effector
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def add_box_m(self, data, timeout=4):

        print "Start 'add box_m'", data.object_name[0]
        box_name = data.object_name[0]
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "/base_footprint"

        box_pose.pose.position = data.object_position
        box_pose.pose.orientation = data.object_orientation

        box_scale = (data.object_scale.x, data.object_scale.y, data.object_scale.z)

        self.scene.add_box(box_name, box_pose, box_scale)

        self.box_name = box_name
        self.object_list.append(box_name)

        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):

        box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        grasping_group = 'hand'
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)

        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)


    def attach_box_m(self, data, timeout=4):

        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        grasping_group = data.hand_name[0]
        touch_links = robot.get_link_names(group=grasping_group)
        print "touch links list\n", touch_links
        scene.attach_box(eef_link, data.box_name[0], touch_links=touch_links)

        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach_box_m(self, data, timeout=4):

        scene = self.scene
        eef_link = self.eef_link
        scene.remove_attached_object(eef_link, name=data.box_name[0])

        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def detach_box(self, timeout=4):

        box_name = self.box_name
        scene = self.scene
        eef_link = self.eef_link
        scene.remove_attached_object(eef_link, name=box_name)

        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def remove_box_m(self, data, timeout=4):

        scene = moveit_commander.PlanningSceneInterface()
        self.scene = scene
        self.box_name = data.object_name[0]
        scene.remove_world_object(self.box_name)

        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

    def remove_box(self, timeout=4):

        box_name = self.box_name
        scene = self.scene
        scene.remove_world_object(box_name)

        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

    def setjoint_m(self, data):
        print "go to initial pose"

        self.group_name = 'l_arm'  # this is just for the initialization
        move_group = moveit_commander.MoveGroupCommander(self.group_name)
        joint_goal = move_group.get_current_joint_values()

        joint_goal[0] = 0.60
        joint_goal[1] = +0.3
        joint_goal[2] = -0.054
        joint_goal[3] = -2.25
        joint_goal[4] = -1.59
        joint_goal[5] = -0.3
        joint_goal[6] = 0.01

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        self.group_name = 'r_arm' # this is just for the initialization
        move_group = moveit_commander.MoveGroupCommander(self.group_name)
        joint_goal = move_group.get_current_joint_values()

        joint_goal[0] = 0.60
        joint_goal[1] = -0.3
        joint_goal[2] = -0.054
        joint_goal[3] = -2.25
        joint_goal[4] = -1.59
        joint_goal[5] = 0.3
        joint_goal[6] = 0.01

        move_group.go(joint_goal, wait=True)
        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def move_joints_m(self, data):

        self.group_name = data.name[0]  # this is just for the initialization
        print self.group_name,"planning group!!!!!!!!!!1"
        move_group = moveit_commander.MoveGroupCommander(self.group_name)
        joint_goal = move_group.get_current_joint_values()

        joint_goal[0] = data.position[0]
        joint_goal[1] = data.position[1]
        joint_goal[2] = data.position[2]
        joint_goal[3] = data.position[3]
        joint_goal[4] = data.position[4]
        joint_goal[5] = data.position[5]
        joint_goal[6] = data.position[6]

        move_group.go(joint_goal, wait=True)
        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def remove_all_obj_m(self, data):
        print "remove all objects_m if 1, data:", data, type(data)
        if data.data == '1':
            print "remove all start"
            for i in tutorial.object_list:
                scene = moveit_commander.PlanningSceneInterface()
                self.scene = scene
                self.box_name = i

                scene.remove_world_object(self.box_name)

    '''
    (function)_s means that it is an server which gets ROS service messages.
    '''

    def add_box_s(self, data, timeout=4):
        print "add_box_s received"

        print "Start 'add box_m'", data.object_name[0]
        box_name = data.object_name[0]
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "/base_footprint"

        box_pose.pose.position = data.object_position
        box_pose.pose.orientation = data.object_orientation
        box_scale = (data.object_scale.x, data.object_scale.y, data.object_scale.z)

        self.scene.add_box(box_name, box_pose, box_scale)
        self.box_name = box_name
        self.object_list.append(box_name)

        return box_info_srvResponse(
            w_flag=1
        )

    def del_box_s(self, data, timeout=4):

        scene = moveit_commander.PlanningSceneInterface()
        self.scene = scene
        self.box_name = data.object_name[0]
        scene.remove_world_object(self.box_name)

        return box_info_srvResponse(
            w_flag=1
        )

    def att_box_s(self, data, timeout=4):

        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        grasping_group = data.hand_name[0]
        touch_links = robot.get_link_names(group=grasping_group)
        print "touch links list\n", touch_links
        scene.attach_box(eef_link, data.object_name[0], touch_links=touch_links)

        return att_hand_box_srvResponse(
            w_flag=1
        )

    def det_box_s(self, data, timeout=4):

        scene = self.scene
        eef_link = self.eef_link

        scene.remove_attached_object(eef_link, name=data.object_name[0])
        return box_info_srvResponse(
            w_flag=1
        )

    def goalPose_feasibility_check_s(self, data):

        # move_group = self.move_group
        move_group = moveit_commander.MoveGroupCommander(data.arm_name[0])
        pose_goal = geometry_msgs.msg.Pose()

        pose_goal.position = data.goal_position
        pose_goal.orientation = data.goal_orientation
        # move_group.set_planner_id('SPARStwo')
        # move_group.set_planner_id('RRTstar')
        # move_group.set_planner_id('BiTRRT')

        move_group.set_num_planning_attempts(10000)
        move_group.set_planning_time(5)
        move_group.set_goal_position_tolerance(0.01)
        move_group.set_goal_orientation_tolerance(0.01)

        move_group.set_pose_target(pose_goal)
        plan = move_group.plan()
        # print plan
        print "plan.joint_trajectory.joint_names :", plan.joint_trajectory.joint_names

        if len(plan.joint_trajectory.joint_names) == 0:
            print "no plan found"
            move_group.stop()
            move_group.clear_pose_targets()
            return arm_move_srvResponse(
                w_flag=1,
                feasibility=0,
                r_trj=plan
            )
        elif len(plan.joint_trajectory.joint_names) > 0:
            print "plan found"
            move_group.stop()
            move_group.clear_pose_targets()
            return arm_move_srvResponse(
                w_flag=1,
                feasibility=1,
                r_trj=plan
            )

    def init_joints_s(self, data):
        if data.w_start == 1:
            print "go to initial pose"

            self.group_name = 'l_arm'  # this is just for the initialization
            move_group = moveit_commander.MoveGroupCommander(self.group_name)
            joint_goal = move_group.get_current_joint_values()

            joint_goal[0] = 0.60
            joint_goal[1] = +0.3
            joint_goal[2] = -0.054
            joint_goal[3] = -2.25
            joint_goal[4] = -1.59
            joint_goal[5] = -0.3
            joint_goal[6] = 0.01

            # The go command can be called with joint values, poses, or without any
            # parameters if you have already set the pose or joint target for the group
            move_group.go(joint_goal, wait=True)

            self.group_name = 'r_arm' # this is just for the initialization
            move_group = moveit_commander.MoveGroupCommander(self.group_name)
            joint_goal = move_group.get_current_joint_values()

            joint_goal[0] = 0.60
            joint_goal[1] = -0.3
            joint_goal[2] = -0.054
            joint_goal[3] = -2.25
            joint_goal[4] = -1.59
            joint_goal[5] = 0.3
            joint_goal[6] = 0.01

            move_group.go(joint_goal, wait=True)
            move_group.stop()

            current_joints = move_group.get_current_joint_values()
            return work_start_srvResponse(
                w_flag=1
            )

    def remove_all_s(self, data):
        print "data:", data
        if data.w_start == 1:
            print "remove all start"
            for i in tutorial.object_list:
                scene = moveit_commander.PlanningSceneInterface()
                self.scene = scene
                self.box_name = i
                scene.remove_world_object(self.box_name)
            return work_start_srvResponse(
                w_flag=1
            )

    def move_joints_s(self, data):

        self.group_name = data.goalPose.name[0]  # this is just for the initialization
        print self.group_name,"planning group!!!!!!!!!!1"
        move_group = moveit_commander.MoveGroupCommander(self.group_name)
        joint_goal = move_group.get_current_joint_values()

        joint_goal[0] = data.goalPose.position[0]
        joint_goal[1] = data.goalPose.position[1]
        joint_goal[2] = data.goalPose.position[2]
        joint_goal[3] = data.goalPose.position[3]
        joint_goal[4] = data.goalPose.position[4]
        joint_goal[5] = data.goalPose.position[5]
        joint_goal[6] = data.goalPose.position[6]

        move_group.go(joint_goal, wait=True)
        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return arm_goalJoint_srvResponse(
                w_flag=1
            )

def listener():

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_arm_controller', anonymous=True)

    # =================== message!! ======================
    rospy.Subscriber('arm_goalPose', arm_move_msg, tutorial.go_to_pose_goal_m)
    rospy.Subscriber('feasibility_check', arm_move_msg, tutorial.goalPose_feasibility_check_m)

    rospy.Subscriber('arm_initJoint', String, tutorial.setjoint_m)
    rospy.Subscriber('remove_all_objects', String, tutorial.remove_all_obj_m)
    rospy.Subscriber('arm_goalJoint', sensor_msgs.msg.JointState, tutorial.move_joints_m)

    rospy.Subscriber('add_box_info', box_info_msg, tutorial.add_box_m)
    rospy.Subscriber('del_box_info', box_info_msg, tutorial.remove_box_m)
    rospy.Subscriber('det_box_info', box_info_msg, tutorial.detach_box_m)
    rospy.Subscriber('att_box_info', attach_hand_box, tutorial.attach_box_m)

    # =================== service!! =======================
    rospy.Service('feasibile_check_srv', arm_move_srv, tutorial.goalPose_feasibility_check_s)
    rospy.Service('arm_goalJoint_srv', arm_goalJoint_srv, tutorial.move_joints_s)

    rospy.Service('arm_initJoint_srv', work_start_srv, tutorial.init_joints_s)
    rospy.Service('remove_all_srv', work_start_srv, tutorial.remove_all_s)

    rospy.Service('add_box_srv', box_info_srv, tutorial.add_box_s)
    rospy.Service('del_box_srv', box_info_srv, tutorial.del_box_s)
    rospy.Service('det_box_srv', box_info_srv, tutorial.det_box_s)
    rospy.Service('att_box_srv', att_hand_box_srv, tutorial.att_box_s)

    rospy.spin()


if __name__ == '__main__':

    print "------------------------------"
    print "Arm trajectory NODE starts!!!!"
    print "------------------------------"
    print "Press Ctrl-D to exit at any time"
    tutorial = MoveGroupPythonIntefaceTutorial()
    object_list = []
    listener()

    print "end node!!"
