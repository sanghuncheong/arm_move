#!/usr/bin/env python

import time
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import math
import random
import numpy as np
from numpy.linalg import inv

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

from arm_move.srv._Update_task_state_srv import *
from arm_move.srv._Generate_pose_srv import *
from arm_move.srv._Find_trajectory_srv import *
from arm_move.srv._PoseService import *
from arm_move.srv._PredicateService import *
from arm_move.srv._ActionService import *

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

import ros_podo_connector.msg
from arm_move.srv._arm_move_srv import *
import action_manager_hubo as am
from tf.transformations import euler_from_quaternion, quaternion_from_euler

HUBO_HEIGHT = 0.7


# C. NAM
# Clients requesting various states to Hubo and KM
def get_base_pose_Hubo_abs():  # Hubo
    # TODO (KIST): implement the interface function
    # ROS topic: tf/child_frame_id: "base_link"
    # see mobile_path_planning.cpp -> set_start_pose function
    # base_pose = [x, y, z, qx, qy, qz, qw]
    base_pose = [0.0, 0.0, 0.0, 0, 0, 0, 0]
    ori_euler = euler_from_quaternion(base_pose[3:7])
    base_pose = [base_pose[0], base_pose[1], ori_euler[2]]
    return base_pose


def get_joint_state_Hubo():  # Hubo
    # TODO (KIST): implement the interface function
    # TODO (KAIST)
    # joint_state = [joint1, joint2, joint3, joint4, joint5, joint6]
    joint_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    return joint_state


def get_ee_pose_Hubo_rel():  # Hubo
    # TODO (KIST): implement the interface function
    # ros_podo_connector -> rospodo_arm/goal
    # ee_pose = [x, y, z, qx, qy, qz, qw]
    ee_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    return ee_pose


def get_gripper_state_Hubo():  # Hubo
    # TODO (KIST): implement the interface function
    # ros_podo_connector -> rospodo_gripper/feedback (goal: the command sent, feedback: the command recieved, result: returned if finished)
    # val = 0 # close
    # val = 1 # open
    val = 0
    return val


def get_object_pose_KM_abs(object_name):  # KM
    # TODO (KIST): implement the interface function
    # grasp_position = [x, y, z]
    # grasp_orientation = [roll, pitch, yaw]
    # grasp_pose = [x, y, z, roll, pitch, yaw]
    object_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] #dummy value
    return object_pose


def get_grasp_pose_KM_abs(object_name):  # KM  #[grip.app.x, grip.app.y, grip.app.z]
    # TODO (KIST): implement the interface function
    # grasp_position = [x, y, z]
    # grasp_orientation = [roll, pitch, yaw]
    # grasp_pose = [x, y, z, roll, pitch, yaw]
    grasp_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] #dummy value
    return grasp_pose


def get_place_size_KM(place_name):  # width, length, height
    # TODO (KIST): implement the interface function
    place_size = [1.0, 1.0, 1.0] #dummy value
    return place_size


def get_place_position_KM_abs(place_name):
    # TODO (KIST): implement the interface function
    place_position = [0.0, 0.0, 0.5] #dummy value
    return place_position


def get_plane_size_KM(plane_name):  # width, length, height
    # TODO (KIST): implement the interface function
    plane_size = [1.0, 1.0, 1.0] #dummy value
    return plane_size


def get_plane_position_KM_abs(plane_name):
    # TODO (KIST): implement the interface function
    plane_position = [0.0, 0.0, 0.5] #dummy value
    return plane_position


def get_object_size_KM(object_name):
    # TODO (KIST): implement the interface function
    object_size = [0.1, 0.1, 0.1] #dummy value
    return object_size


def get_object_name_KM(place_name):
    # TODO (KIST): implement the interface function
    # TODO (KIST): provide information about the bounding box of the env (e.g., [x_min, x_max, y_min, y_max, z_min, z_max])
    # TODO (KGU): implement the predicate for this query
    # Ask all object names in the place
    # Input: place_name ("table_env" or "storage_env" or "display_env")
    # Output: place_name (e.g., "Shelf01", "CrackerBox01")
    # place_name = "table_env"
    object_names = ["Shelf01", "Shelf02", "CrackerBox01", "SnackCan01"] #dummy value
    return object_names


def get_place_object_in_KM(object_name):
    # TODO (KIST): implement the interface function
    # TODO (KGU): implement the predicate for this query
    # Ask the place (e.g., "table_env") where the object is located in
    # Input: object_name (e.g., "Shelf01", "CrackerBox01")
    # Output: place_name (e.g., "table_env")
    place_name = "table_env" #dummy value
    return place_name


def get_empty_slot_in_display_KM_abs(plane_name, object_name):
    # TODO (KIST): implement the interface function
    # TODO (KGU): implement the predicate for this query
    # Ask an empty slot in the display (or display shelf) to release the object
    # Input: object_name (e.g., "CrackerBox01")
    # Output: goal_position ([x, y, z])
    goal_position = [0, 0, 0] #dummy value
    return goal_position


# distance(x, y): compute the Euclidean distance between 2D points x and y
def distance(point_one, point_two):
    return ((point_one[0] - point_two[0]) ** 2 +
            (point_one[1] - point_two[1]) ** 2) ** 0.5


# Functions computing poses
def compute_grasp_positions_rel(object_name):
    grasp_pose_abs = get_grasp_pose_KM_abs(object_name)
    current_base_pose = get_base_pose_Hubo_abs()
    transform_mat = np.array([[np.cos(current_base_pose[2]), - np.sin(current_base_pose[2]), current_base_pose[0]],
                              [np.sin(current_base_pose[2]), np.cos(current_base_pose[2]), current_base_pose[1]],
                              [0, 0, 1]])
    grasp_position_2d_rel = inv(transform_mat).dot([grasp_pose_abs[0], grasp_pose_abs[1], 1])
    grasp_position_rel = [grasp_position_2d_rel[0], grasp_position_2d_rel[1], grasp_pose_abs[2] - HUBO_HEIGHT]

    goal_pitches = []
    goal_pitch = grasp_pose_abs[5]
    goal_pitches.append(goal_pitch)
    for i in range(3):
        goal_pitches.append(goal_pitch + (i + 1) * (math.pi / 18))
        goal_pitches.append(goal_pitch - (i + 1) * (math.pi / 18))

    goal_quaternions = []
    goal_positions = []
    for i in goal_pitches:
        goal_quaternions.append(quaternion_from_euler(grasp_pose_abs[3], grasp_pose_abs[4], i))
        goal_positions.append([grasp_position_rel[0], grasp_position_rel[1], grasp_position_rel[2]])
    return [goal_positions, goal_quaternions]


def compute_prepost_grasp_poses_rel(object_name):
    grasp_pose_abs = get_grasp_pose_KM_abs(object_name)
    current_base_pose = get_base_pose_Hubo_abs()
    transform_mat = np.array([[np.cos(current_base_pose[2]), - np.sin(current_base_pose[2]), current_base_pose[0]],
                              [np.sin(current_base_pose[2]), np.cos(current_base_pose[2]), current_base_pose[1]],
                              [0, 0, 1]])
    grasp_position_2d_rel = inv(transform_mat).dot([grasp_pose_abs[0], grasp_pose_abs[1], 1])
    grasp_position_rel = [grasp_position_2d_rel[0], grasp_position_2d_rel[1], grasp_pose_abs[2] - HUBO_HEIGHT]
    goal_pitches = []
    goal_pitch = grasp_pose_abs[5]
    goal_pitches.append(goal_pitch)
    for i in range(3):
        goal_pitches.append(goal_pitch + (i + 1) * (math.pi / 18))
        goal_pitches.append(goal_pitch - (i + 1) * (math.pi / 18))

    # Get the grasp orientation (currently the front direction)
    l = 0.1 + 0.2  # TODO (KIST): replace 0.2 with the actual offset for the robot hand
    goal_quaternions = []
    goal_positions = []
    for i in goal_pitches:
        goal_quaternions.append(quaternion_from_euler(grasp_pose_abs[3], grasp_pose_abs[4], i))
        dx = math.sin(i - math.pi) * l
        dy = math.cos(i - math.pi) * l
        goal_positions.append([grasp_position_rel[0] + dx, grasp_position_rel[1] - dy, grasp_position_rel[2]])
    return [goal_positions, goal_quaternions]


def compute_release_positions_rel(plane_name, object_name):
    # NOTE: Just use some fixed positions for releasing objects (display only)
    # Use the below for the other places
    place_name = get_place_object_in_KM(plane_name)
    if place_name == "display_env":
        release_position_abs = get_empty_slot_in_display_KM_abs(plane_name, object_name)
    else:
        plane_size = get_plane_size_KM(plane_name)
        plane_position_abs = get_plane_position_KM_abs(plane_name)
        object_size = get_object_size_KM(object_name)
        object_size_max = max(object_size[0], object_size[1])
        z_abs = plane_size[2] + object_size[2] / 2 + 0.02  # place height + object height/2 + 2cm

        # Find all objects placed in place_id
        x_min = plane_position_abs[0] - plane_size[0] / 2
        x_max = plane_position_abs[0] + plane_size[0] / 2
        y_min = plane_position_abs[1] - plane_size[1] / 2
        y_max = plane_position_abs[1] + plane_size[1] / 2

        x_abs = random.uniform(x_min + 0.1 + object_size_max, x_max - 0.1 - object_size_max)
        y_abs = random.uniform(y_min + 0.1 + object_size_max, y_max - 0.1 - object_size_max)

        release_position_abs = [x_abs, y_abs, z_abs]

    current_base_pose = get_base_pose_Hubo_abs()
    transform_mat = np.array([[np.cos(current_base_pose[2]), - np.sin(current_base_pose[2]), current_base_pose[0]],
                              [np.sin(current_base_pose[2]), np.cos(current_base_pose[2]), current_base_pose[1]],
                              [0, 0, 1]])
    release_position_2d_rel = inv(transform_mat).dot([release_position_abs[0], release_position_abs[1], 1])
    release_position_rel = [release_position_2d_rel[0], release_position_2d_rel[1],
                            release_position_abs[2] - HUBO_HEIGHT]

    goal_pitches = []
    goal_pitch = 0.0  # robot's heading relative to the robot itself
    goal_pitches.append(goal_pitch)
    for i in range(3):
        goal_pitches.append(goal_pitch + (i + 1) * (math.pi / 18))
        goal_pitches.append(goal_pitch - (i + 1) * (math.pi / 18))

    # Get the grasp orientation (currently the front direction)
    goal_quaternions = []
    goal_positions = []
    for i in goal_pitches:
        goal_quaternions.append(quaternion_from_euler(0.0, 0.0, i))  # , axes='rxyz'))
        goal_positions.append([release_position_rel[0], release_position_rel[1], release_position_rel[2]])
    return [goal_positions, goal_quaternions]


def compute_release_position_abs(plane_name, object_name):  # KM
    plane_size = get_plane_size_KM(plane_name)
    plane_position_abs = get_plane_position_KM_abs(plane_name)
    object_size = get_object_size_KM(object_name)
    object_size_max = max(object_size[0], object_size[1])
    z_abs = plane_size[2] + object_size[2] / 2 + 0.02  # place height + object height/2 + 2cm

    # Find all objects placed in place_id
    x_min = plane_position_abs[0] - plane_size[0] / 2
    x_max = plane_position_abs[0] + plane_size[0] / 2
    y_min = plane_position_abs[1] - plane_size[1] / 2
    y_max = plane_position_abs[1] + plane_size[1] / 2

    x_abs = random.uniform(x_min + 0.1 + object_size_max, x_max - 0.1 - object_size_max)
    y_abs = random.uniform(y_min + 0.1 + object_size_max, y_max - 0.1 - object_size_max)

    release_position_abs = [x_abs, y_abs, z_abs]
    return release_position_abs


def compute_prepost_release_poses_rel(release_position_abs):
    current_base_pose = get_base_pose_Hubo_abs()
    transform_mat = np.array([[np.cos(current_base_pose[2]), - np.sin(current_base_pose[2]), current_base_pose[0]],
                              [np.sin(current_base_pose[2]), np.cos(current_base_pose[2]), current_base_pose[1]],
                              [0, 0, 1]])
    release_position_2d_rel = inv(transform_mat).dot([release_position_abs[0], release_position_abs[1], 1])
    release_position_rel = [release_position_2d_rel[0], release_position_2d_rel[1],
                            release_position_abs[2] - HUBO_HEIGHT]

    goal_pitches = []
    goal_pitch = 0.0  # robot's heading relative to the robot itself
    goal_pitches.append(goal_pitch)
    for i in range(3):
        goal_pitches.append(goal_pitch + (i + 1) * (math.pi / 18))
        goal_pitches.append(goal_pitch - (i + 1) * (math.pi / 18))

    # Get the grasp orientation (currently the front direction)
    l = 0.1
    goal_quaternions = []
    goal_positions = []
    for i in goal_pitches:
        goal_quaternions.append(quaternion_from_euler(0.0, 0.0, i))  # , axes='rxyz'))
        dx = math.sin(i - math.pi) * l
        dy = math.cos(i - math.pi) * l
        goal_positions.append([release_position_rel[0] + dx, release_position_rel[1] - dy, release_position_rel[2]])
    return [goal_positions, goal_quaternions]


def compute_base_pose_abs(target_name):
    # TODO (KIST): replace the below coordinates with real ones
    # TODO (KIST): may compute the pose for each object (currently, use a fixed location for each place where the object is located in)
    # Begin: For each place
    # if target_name != "table_env" and target_name != "display_env" and target_name != "storage_env":
    #     target_name = get_place_object_in_KM(target_name)
    # if target_name == "table_env":
    #     base_pose = [0.0, 0.0, math.radians(-90.0)]
    # elif target_name == "display_env":
    #     base_pose = [0.0, 2.0, math.radians(90.0)]
    # elif target_name == "storage_env":
    #     base_pose = [0.5, 1.0, math.radians(0.0)]
    # End: For each place

    # Begin: For each object
    #TODO (KIST): adjust the x and y offset values considering the actual robot workspace size
    x_offset = 0.5
    y_offset = 0.5
    if target_name != "table_env" and target_name != "display_env" and target_name != "storage_env":
        object_pose = get_object_pose_KM_abs(target_name)
        place_name = get_place_object_in_KM(target_name)
        if place_name == "table_env":
            base_pose = [object_pose[0] + x_offset, object_pose[1] + y_offset, math.radians(-90.0)]
        elif place_name == "display_env":
            base_pose = [object_pose[0] - x_offset, object_pose[1] - y_offset, math.radians(90.0)]
        elif place_name == "storage_env":
            base_pose = [object_pose[0] - x_offset, object_pose[1] + y_offset, math.radians(0.0)]
    elif target_name == "table_env":
        base_pose = [0.0, 0.0, math.radians(-90.0)]
    elif target_name == "display_env":
        base_pose = [0.0, 2.0, math.radians(90.0)]
    elif target_name == "storage_env":
        base_pose = [0.5, 1.0, math.radians(0.0)]

    # End: For each object

    return base_pose


def construct_moveit_env(place_name):
    object_names = get_object_name_KM(place_name)
    for object_name in object_names:
        add_object(object_name)


def add_object(object_name):
    current_base_pose = get_base_pose_Hubo_abs()
    object_size = get_object_size_KM(object_name)
    object_pose_abs = get_object_pose_KM_abs(object_name)

    transform_mat = np.array([[np.cos(current_base_pose[2]), - np.sin(current_base_pose[2]), current_base_pose[0]],
                              [np.sin(current_base_pose[2]), np.cos(current_base_pose[2]), current_base_pose[1]],
                              [0, 0, 1]])
    object_position_2d_rel = inv(transform_mat).dot([object_pose_abs[0], object_pose_abs[1], 1])
    object_position_rel = [object_position_2d_rel[0], object_position_2d_rel[1], object_pose_abs[2] - HUBO_HEIGHT]
    object_quaternions = quaternion_from_euler(object_pose_abs[3], object_pose_abs[4], object_pose_abs[5])
    am.add_box_client(object_name, object_position_rel, object_quaternions, object_size, 'red')


def del_objects(object_name):
    am.del_box_client(object_name)


def del_all_objects():
    am.del_all_client()


def plan_base_pose(goal_pose):
    # current_base_pose = get_base_pose_Hubo()
    #goal_pose = goal_position + goal_orientation
    result = am.feasiblity_check_base(goal_pose)
    return result


def plan_arm_pose(start_state, goal_position, goal_orientation, object_name):
    planner_name = 'RRTConnect'
    # planner_name = 'BiTRRT'
    n_attempt = 100
    c_time = 2
    n_repeat = 5
    goal_position = [0.3, -0.4, 0.7]
    goal_orientation = [0.5, -0.5, -0.5, 0.5]
    result, _ = am.feasibility_check_arm_obj(start_state, goal_position, goal_orientation, object_name, planner_name,
                                             n_attempt, c_time, n_repeat)
    # result = am.feasibility_check_arm(goal_position, goal_orientation, planner_name, n_attempt, c_time, n_repeat)
    return result


def execute_base_pose(goal_pose):
    # current_base_pose = get_base_pose_Hubo()
    result = am.execute_base(goal_pose)
    return result


def execute_arm_pose(goal_position, goal_orientation):
    planner_name = 'RRTConnect'
    # planner_name = 'BiTRRT'
    n_attempt = 100
    c_time = 2
    n_repeat = 5
    goal_position = [0.3, -0.4, 0.7]
    goal_orientation = [0.5, -0.5, -0.5, 0.5]
    result = am.execute_arm(goal_position, goal_orientation, planner_name, n_attempt, c_time, n_repeat)
    return result


finish = False


class tmi(object):
    def __init__(self):
        super(tmi, self).__init__()

    ###################################
    # Begin feasibility check functions
    def compute_arm_pose_service(self, data, timeout=4):
        hand_name = data.param1  # "R_hand"
        object_name = data.param2
        plane_name = data.param3
        predicate = data.predicate
        if predicate == "graspHandPose":
            [goal_positions_rel, goal_orientations_rel] = compute_grasp_positions_rel(object_name)
        elif predicate == "preGraspableHandPose":
            [goal_positions_rel, goal_orientations_rel] = compute_prepost_grasp_poses_rel(object_name)
        elif predicate == "postGraspHandPose":
            [goal_positions_rel, goal_orientations_rel] = compute_prepost_grasp_poses_rel(object_name)

        elif predicate == "releaseHandPose":  # !!!CHANGE: placeableObjectLocation!!!
            [goal_positions_rel, goal_orientations_rel] = compute_release_positions_rel(plane_name, object_name)

        elif predicate == "preReleaseHandPose":  # !!!NEW!!!
            release_position_abs = compute_release_position_abs(plane_name, object_name)
            [goal_positions_rel, goal_orientations_rel] = compute_prepost_release_poses_rel(release_position_abs)
        elif predicate == "postReleaseHandPose":
            release_position_abs = compute_release_position_abs(plane_name, object_name)
            [goal_positions_rel, goal_orientations_rel] = compute_prepost_release_poses_rel(release_position_abs)
        elif predicate == "workReadyHandPose": #TODO(KGU): add a predicate for going back to the work ready pose (when moving to other places)
            goal_positions_rel = [0, 0, 0] #TODO(KIST): put the actual work ready pose
            goal_orientations_rel = [0, 0, 0, 0]
        else:
            sys.exit("[ERROR] compute_arm_pose: No matching predicate")

        # goals[0--6] = [x, y, z, qx, qy, qz, qw] in relative coordinate
        goals = []
        for i in range(len(goal_positions_rel)):
            goals.append(goal_positions_rel[i] + goal_orientations_rel[i])

        ### IMPLEMENT SERVICE
        rospy.wait_for_service('Update_task_state')
        tmi_srv = rospy.ServiceProxy('Update_task_state', PredicateService)

        pub_msg = PredicateServiceRequest()
        pub_msg.predicate = [predicate] * len(goal_positions_rel)
        pub_msg.param1 = [hand_name] * len(goal_positions_rel)
        pub_msg.param2 = [object_name] * len(goal_positions_rel)
        pub_msg.param3 = [plane_name] * len(goal_positions_rel)
        pub_msg.param4 = goals

        response1 = tmi_srv(pub_msg)
        response2 = 1  # fail
        if response1:
            response2 = PoseServiceResponse(result=0)
        return response2  # [goal_positions_rel, goal_orientations_rel]

    def compute_base_pose_service(self, data, timeout=4):
        base_name = data.param1  # "Hubo_base"
        target_name = data.param2
        predicate = data.predicate
        if predicate == "detectableBasePose":
            # object or place name
            base_pose_abs = compute_base_pose_abs(target_name)
        elif predicate == "reachableBasePose":
            # object name
            base_pose_abs = compute_base_pose_abs(target_name)
        elif predicate == "placeableBasePose":
            # plane name
            base_pose_abs = compute_base_pose_abs(target_name)

        ### IMPLEMENT SERVICE
        rospy.wait_for_service('Update_task_state')
        tmi_srv = rospy.ServiceProxy('Update_task_state', PredicateService)

        pub_msg = PredicateServiceRequest()
        pub_msg.predicate = [predicate] * len(base_pose_abs)
        pub_msg.param1 = [base_name] * len(base_pose_abs)
        pub_msg.param2 = [target_name] * len(base_pose_abs)
        pub_msg.param3 = base_pose_abs

        response1 = tmi_srv(pub_msg)
        response2 = 1  # fail
        if response1:
            response2 = PoseServiceResponse(result=0)
        return response2  # [goal_positions_rel, goal_orientations_rel]

    def compute_arm_motion_service(self, data, timeout=4):
        predicate = data.predicate  # "validHandPath":
        hand_name = data.param1
        start_state = data.param2
        goal_poses = data.param3
        # goal_orientations = data.param4
        # object_name = data.param5#!!!New parameter!!!
        feasible_goals = []

        for i in range(len(goal_poses)):
            goal_pose = goal_poses[i]
            goal_position = goal_pose[0:3]
            goal_orientation = goal_pose[3:7]
            feasibility = plan_arm_pose(start_state, goal_position, goal_orientation, [])
            if feasibility:
                feasible_goals.append(goal_pose)

        ### IMPLEMENT SERVICE
        rospy.wait_for_service('Update_task_state')
        tmi_srv = rospy.ServiceProxy('Update_task_state', PredicateService)

        pub_msg = PredicateServiceRequest()
        pub_msg.predicate = [predicate]
        pub_msg.param1 = [hand_name]
        pub_msg.param2 = [start_state]
        pub_msg.param3 = feasible_goals

        response1 = tmi_srv(pub_msg)
        response2 = 1  # fail
        if response1:
            response2 = PoseServiceResponse(result=0)
        return response2  # [goal_positions_rel, goal_orientations_rel]

    def compute_base_motion_service(self, data, timeout=4):
        # predicate = data.param4 #"validBasePath":
        predicate = data.predicate  # "validHandPath":
        base_name = data.param1
        start_state = data.param2
        goal_pose = data.param3
        #goal_position = goal_pose[0:3]
        #goal_orientation = goal_pose[3:7]

        feasibility = plan_base_pose(goal_pose)
        feasible_goal = []
        if feasibility:
            feasible_goal.append(goal_pose)

        ### IMPLEMENT SERVICE
        rospy.wait_for_service('Update_task_state')
        tmi_srv = rospy.ServiceProxy('Update_task_state', PredicateService)

        pub_msg = PredicateServiceRequest()
        pub_msg.predicate = [predicate]
        pub_msg.param1 = [base_name]
        pub_msg.param2 = [start_state]
        pub_msg.param3 = feasible_goal

        response1 = tmi_srv(pub_msg)
        response2 = 1  # fail
        if response1:
            response2 = PoseServiceResponse(result=0)
        return response2  # [goal_positions_rel, goal_orientations_rel]

    ###################################
    # Begin action functions

    # do_Graspact, do_PreGraspact, do_Rel_Graspact
    def move_arm_service(self, data, timeout=4):
        predicate = data.predicate  # "validHandPath":
        hand_name = data.param1
        goal_pose = data.param3
        goal_position = goal_pose[0:3]
        goal_orientation = goal_pose[3:7]
        result = execute_arm_pose(goal_position, goal_orientation)
        if result:
            response = ActionServiceResponse(result=0)
        else:
            response = ActionServiceResponse(result=1)
        return response

    # do_Baseact, do_BaseFrontBack, do_base_Returnact, do_Base_stroageact, do_Base_Return_storageact
    def move_base_service(self, data, timeout=4):
        predicate = data.predicate  # "validHandPath":
        base_name = data.param1  # TODO (KGU): send the robot base name when request (i.e., "Hubo_base")
        goal_pose = data.param3
        result = execute_base_pose(goal_pose)
        if result:
            response = ActionServiceResponse(result=0)
        else:
            response = ActionServiceResponse(result=1)
        return response

    # do_Hand_Closeact
    def close_gripper_service(self, data, timeout=4):
        am.gripper_close()

    # do_Hand_Openact
    def open_gripper_service(self, data, timeout=4):
        am.gripper_open()

    # do_MOVEit_env_create, do_MOVEit_env_create_2
    def moveit_env_create_service(self, data, timeout=4):
        place_name = data.param1  # TODO (KGU): send env name (e.g., "table_env" ) when request
        construct_moveit_env(place_name)

    # do_MOVEit_env_apply (After close hand)
    def moveit_object_grasp_service(self, data, timeout=4):
        object_name = data.param1  # TODO (KGU): send obj name (e.g., "CrakerBox01") when request
        hand_name = data.param2  # TODO (KGU): send hand name  (i.e., "R_hand")
        am.att_box_client(object_name, hand_name)

    def moveit_env_clear_all_service(self, data, timeout=4):
        del_all_objects()

    # do_MOVEit_removeall
    def moveit_env_clear_except_obj_service(self, data, timeout=4):
        grasped_object_name = data.param1  # TODO (KGU): send grasped obj name (e.g., "CrakerBox01") when request
        place_name = data.param2  # TODO (KGU): send env name (e.g., "table_env" ) when request
        object_names = get_object_name_KM(place_name)
        for object_name in object_names:
            if object_name != grasped_object_name:
                del_objects(object_name)


def listener():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('tmi_node', anonymous=True)

    # =================== service!! =======================
    # TODO (KGU): remap the service name (see below for the previously used service names)

    # gen_pose
    rospy.Service('compute_arm_pose', PoseService, tmi1.compute_arm_pose_service)
    rospy.Service('compute_base_pose', PoseService, tmi1.compute_base_pose_service)

    # fin_tra
    rospy.Service('compute_arm_motion', PredicateService, tmi1.compute_arm_pose_service)
    rospy.Service('compute_base_motion', PredicateService, tmi1.compute_base_pose_service)

    # do_Graspact, do_PreGraspact, do_Rel_Graspact
    rospy.Service('execute_arm', ActionService, tmi1.move_arm_service)

    # do_Baseact, do_BaseFrontBack, do_base_Returnact, do_Base_stroageact, do_Base_Return_storageact
    rospy.Service('execute_base', ActionService, tmi1.move_base_service)

    # do_Hand_Openact
    rospy.Service('open_gripper', ActionService, tmi1.open_gripper_service)

    # do_Hand_Closeact
    rospy.Service('close_gripper', ActionService, tmi1.close_gripper_service)

    # do_MOVEit_env_create, do_MOVEit_env_create_2
    rospy.Service('moveit_env_create', ActionService, tmi1.moveit_env_create_service)

    # do_MOVEit_env_apply (After close hand)
    rospy.Service('moveit_grasp', ActionService, tmi1.moveit_object_grasp_service)

    # do_MOVEit_removeall
    rospy.Service('moveit_clear_except_obj', ActionService, tmi1.moveit_env_clear_except_obj_service)

    # A new service (just in case it is needed)
    rospy.Service('moveit_clear_all', ActionService, tmi1.moveit_env_clear_all_service)

    rospy.spin()


if __name__ == '__main__':
    print("node tmi start!")
    tmi1 = tmi()
    listener()

    print("End node!!")
