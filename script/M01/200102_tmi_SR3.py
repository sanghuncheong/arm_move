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
import numpy as np
import math
import random
#from math import pi

import actionlib
import actionlib_tutorials.msg
import actionlib_tutorials.msg._JointData
import actionlib_tutorials.msg._JointDataSet

import ros_podo_connector.msg

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf.transformations as ttf

from arm_move.msg import *
from arm_move.srv import * 
#from arm_move.msg._arm_move_msg import arm_move_msg
#from arm_move.msg._box_info_msg import box_info_msg
#from arm_move.msg._attach_hand_box import attach_hand_box
#from arm_move.srv._box_info_srv import *
#from arm_move.srv._att_hand_box_srv import *
#from arm_move.srv._arm_move_srv import *
#from arm_move.srv._work_start_srv import *
#from arm_move.srv._arm_goalJoint_srv import *
#from arm_move.srv._Update_task_state_srv import *
#from arm_move.srv._Generate_pose_srv import *
#from arm_move.srv._Find_trajectory_srv import *
#from arm_move.srv._PoseService import *
#from arm_move.srv._PredicateService import *
#from arm_move.srv._ActionService import *

from rosjava_custom_format.msg._MonitorServiceResponse import MonitorServiceResponse
from rosjava_custom_format.srv._MonitorSimilarService import *

import sensor_msgs.msg
from sensor_msgs.msg import JointState

from std_msgs.msg import String
from std_msgs.msg import Header
from std_msgs.msg import Float32MultiArray

from moveit_commander.conversions import pose_to_list

from obj_msg.msg import *

import geometry_msgs.msg
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist

import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg._RobotTrajectory import RobotTrajectory

import D_0902_client_function as CLF
import S_custom_function as CUF

import relocate_planner as rp

#import matplotlib.pyplot as plt
#import action_manager_hubo as am

############### KM queries

# Clients requesting various states to KM
def get_object_pose_KM_rel(object_name):  # KM
    rospy.wait_for_service('context_manager/monitor/service')
    try:
        query_srv = rospy.ServiceProxy('context_manager/monitor/service', MonitorSimilarService)
        pub_msg = MonitorSimilarServiceRequest()
        query_url = "http://www.arbi.com/ontologies/arbi.owl#"
        object_url = query_url + object_name        
        
        pub_msg.predicate = "worldPosition"
        pub_msg.param1 = object_url
        pub_msg.param2 = "ObjectPose"
        pub_msg.param3 = "0"
        pub_msg.param4 = "0"
        pub_msg.status = 100
        pub_msg.manager = "tmi"
        
        resp1 = query_srv(pub_msg)
        #print resp1.response[0].param1
        #resp1 = resp1.param1
        resp1 = resp1.response[0].param1
        resp1 = resp1[1:-1]
        resp1 = map(float, resp1.split())
        #print resp1 #[x, y, z, vx_x, vx_y, vx_z, vy_x, vy_y, vy_z, vz_x, vz_y, vz_z ]
        resp1 = [resp1[0], resp1[1], resp1[2], 0, 0, 0, 1]#compute_object_orientation(object_name, resp1[0:3])
        #resp2 = []
        return  resp1#resp1.extend(resp2)# object_position = [x, y, z, qx, qy, qz, qw]
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def get_object_size_KM(object_name):
    rospy.wait_for_service('context_manager/monitor/service')
    try:
        query_srv = rospy.ServiceProxy('context_manager/monitor/service', MonitorSimilarService)
        pub_msg = MonitorSimilarServiceRequest()
        query_url = "http://www.arbi.com/ontologies/arbi.owl#"
        object_url = query_url + object_name

        pub_msg.predicate = "currentBboxSize"
        pub_msg.param1 = object_url
        pub_msg.param2 = "ObjectSize"
        pub_msg.param3 = "0"
        pub_msg.param4 = "0"
        pub_msg.status = 100
        pub_msg.manager = "tmi"

        resp1 = query_srv(pub_msg)
        resp1 = resp1.response[0].param1# one query may have multiple results so we assigned MSG arraylist. also, result of param may be like this ['0.032','0.11','0.09'].
        resp1 = resp1[1:-1]
        return map(float, resp1.split())# object_size = [w, d, h]
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def get_object_list_KM():
    rospy.wait_for_service('context_manager/monitor/service')
    try:
        query_srv = rospy.ServiceProxy('context_manager/monitor/service', MonitorSimilarService)
        pub_msg = MonitorSimilarServiceRequest()
        query_url = "http://www.arbi.com/ontologies/arbi.owl#"
        place_url = query_url + place_name

        pub_msg.predicate = "currentObjectPerception"
        pub_msg.param1 = "Object"
        pub_msg.param2 = "Perception"
        pub_msg.param3 = "0"
        pub_msg.param4 = "0"
        pub_msg.status = 100
        pub_msg.manager = "tmi"

        resp1 = query_srv(pub_msg)
        resp1 = resp1.response[0].param1
        resp1 = resp1[1:-1]
        return resp1.split()
        # object_list = []
        # for object_infos in resp1.response:
        #    object_list.append(object_infos.param1)
        #return object_list #object_list = ['object_name1', 'object_name2',..]
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def get_grasp_positions_KM_rel(object_name):  # KM  #[grip.app.x, grip.app.y, grip.app.z]

    rospy.wait_for_service('context_manager/monitor/service')        
    try:
        query_srv = rospy.ServiceProxy('context_manager/monitor/service', MonitorSimilarService)
        pub_msg = MonitorSimilarServiceRequest()
        query_url = "http://www.arbi.com/ontologies/arbi.owl#"
        object_url = query_url + object_name

        pub_msg.predicate = "currentGraspPose"
        pub_msg.param1 = object_url
        pub_msg.param2 = "GraspPose"
        pub_msg.param3 = "0"
        pub_msg.param4 = "0"
        pub_msg.status = 100
        pub_msg.manager = "tmi"
        
        resp1 = query_srv(pub_msg)
        resp1 = resp1.response[0].param1
        resp1 = resp1[1:-1]
        resp1 = map(float, resp1.split())
        resp1 = [resp1[i:i+12] for i in xrange(0, len(resp1), 12)]
        return resp1 #map(float, resp1.split()) # grasp_positions = [x, y, z, x, y, z, x, y, z, x, y, z]
        #grasp_position_center = [x, y, z]
        #grasp_position_side0 = [x, y, z]
        #grasp_position_side1 = [x, y, z]
        # grasp_position_approach = [x, y, z]
        # return [grasp_position_center, grasp_position_side0, grasp_position_side1, grasp_position_approach]
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def get_place_size_KM(place_name):  # width, length, height
    rospy.wait_for_service('context_manager/monitor/service')
    try:
        query_srv = rospy.ServiceProxy('context_manager/monitor/service', MonitorSimilarService)
        pub_msg = MonitorSimilarServiceRequest()
        query_url = "http://www.arbi.com/ontologies/arbi.owl#"
        place_url = query_url + place_name

        pub_msg.predicate = "placeSize"
        pub_msg.param1 = place_url
        pub_msg.param2 = "PlaceSize"
        pub_msg.param3 = "0"
        pub_msg.param4 = "0"
        pub_msg.status = 100
        pub_msg.manager = "tmi"

        resp1 = query_srv(pub_msg)
        resp1 = resp1.response[0].param1
        resp1 = resp1[1:-1]
        return map(float, resp1.split()) # place_size = [w, d, h]
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def get_place_pose_KM_abs(place_name):
    rospy.wait_for_service('context_manager/monitor/service')
    try:
        query_srv = rospy.ServiceProxy('context_manager/monitor/service', MonitorSimilarService)
        pub_msg = MonitorSimilarServiceRequest()
        query_url = "http://www.arbi.com/ontologies/arbi.owl#"
        place_url = query_url + place_name

        pub_msg.predicate = "placePose"
        pub_msg.param1 = place_url
        pub_msg.param2 = "PlacePose"
        pub_msg.param3 = "0"
        pub_msg.param4 = "0"
        pub_msg.status = 100
        pub_msg.manager = "tmi"

        resp1 = query_srv(pub_msg)
        resp1 = resp1.response[0].param1
        resp1 = resp1[1:-1]
        return map(float, resp1.split()) # place_pose = [x, y, z, qx, qy, qz, qw]
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def get_plane_size_KM(plane_name):  # width, length, height
    rospy.wait_for_service('context_manager/monitor/service')
    try:
        query_srv = rospy.ServiceProxy('context_manager/monitor/service', MonitorSimilarService)
        pub_msg = MonitorSimilarServiceRequest()
        query_url = "http://www.arbi.com/ontologies/arbi.owl#"
        object_url = query_url + plane_name

        pub_msg.predicate = "planeSize"
        pub_msg.param1 = object_url
        pub_msg.param2 = "PlaneSize"
        pub_msg.param3 = "0"
        pub_msg.param4 = "0"
        pub_msg.status = 100
        pub_msg.manager = "tmi"

        resp1 = query_srv(pub_msg)
        resp1 = resp1.response[0].param1
        resp1 = resp1[1:-1]
        return map(float, resp1.split()) # plane_size = [w, d, h]
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def get_plane_pose_KM_abs(plane_name):
    rospy.wait_for_service('context_manager/monitor/service')
    try:
        query_srv = rospy.ServiceProxy('context_manager/monitor/service', MonitorSimilarService)
        pub_msg = MonitorSimilarServiceRequest()
        query_url = "http://www.arbi.com/ontologies/arbi.owl#"
        plane_url = query_url + plane_name

        pub_msg.predicate = "semanticObjectPose"
        pub_msg.param1 = plane_url
        pub_msg.param2 = "PlanePose"
        pub_msg.param3 = "0"
        pub_msg.param4 = "0"
        pub_msg.status = 100
        pub_msg.manager = "tmi"

        resp1 = query_srv(pub_msg)
        resp1 = resp1.response[0].param1
        return resp1#map(float, resp1.split()) # plane_pose = [x, y, z, qx, qy, qz, qw]
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def get_object_name_KM(place_name):
    rospy.wait_for_service('context_manager/monitor/service')
    try:
        query_srv = rospy.ServiceProxy('context_manager/monitor/service', MonitorSimilarService)
        pub_msg = MonitorSimilarServiceRequest()
        query_url = "http://www.arbi.com/ontologies/arbi.owl#"
        place_url = query_url + place_name

        pub_msg.predicate = "objectsName"
        pub_msg.param1 = place_url
        pub_msg.param2 = "ObjectName"
        pub_msg.param3 = "0"
        pub_msg.param4 = "0"
        pub_msg.status = 100
        pub_msg.manager = "tmi"

        resp1 = query_srv(pub_msg)
        #print "rerere",resp1
        #print "=============================="       
        resp1 = resp1.response[0].param1
        resp1 = resp1[2:-2]
        return resp1.split() # object_names = ["string1", "string2", "all", "objects", "names"]
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def get_place_object_in_KM(object_name):
    rospy.wait_for_service('context_manager/monitor/service')
    try:
        query_srv = rospy.ServiceProxy('context_manager/monitor/service', MonitorSimilarService)
        pub_msg = MonitorSimilarServiceRequest()
        query_url = "http://www.arbi.com/ontologies/arbi.owl#"
        object_url = query_url + object_name

        pub_msg.predicate = "ObjectPlaceName"
        pub_msg.param1 = object_url
        pub_msg.param2 = "PlaceName"
        pub_msg.param3 = "0"
        pub_msg.param4 = "0"
        pub_msg.status = 100
        pub_msg.manager = "tmi"

        resp1 = query_srv(pub_msg)
        #print "get_place_object_in_KM",resp1
        #print "=============================="
        resp1 = resp1.response[0].param1
        resp1 = resp1[1:-1]
        return resp1  # place_name = "place_name_string"
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def get_empty_slot_in_display_KM_abs(plane_name):
    rospy.wait_for_service('context_manager/monitor/service')
    try:
        query_srv = rospy.ServiceProxy('context_manager/monitor/service', MonitorSimilarService)
        pub_msg = MonitorSimilarServiceRequest()
        query_url = "http://www.arbi.com/ontologies/arbi.owl#"
        plane_url = query_url + plane_name
        #object_url = query_url + object_name

        pub_msg.predicate = "emptySlot"
        pub_msg.param1 = plane_url
        #pub_msg.param2 = object_url
        pub_msg.param2 = "EmptySlot"
        pub_msg.param3 = "0"
        pub_msg.param4 = "0"
        pub_msg.status = 100
        pub_msg.manager = "tmi"

	#print pub_msg
        resp1 = query_srv(pub_msg)
	#print 'empty slot query done'
	print resp1
	for i in range(len(resp1.response)):
	    if resp1.response[i].predicate == 'emptySlot':
        	resp1 = int(resp1.response[i].param1)
		break
	print resp1
        return resp1#map(float, resp1.split()) #goal_position = "1" (or 2 or 3 from right to left)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def compute_empty_slots(ws_size, ws_center_pos, tar_pos, target_r, objs_pos, objs_r):
    GRID_SIZE = 0.01
    ws_width = int(ws_size[0]*100)
    ws_depth = int(ws_size[1]*100)

    grid_init = np.zeros([ws_width, ws_depth])
    grid_act = CUF.mark_edge_grid(grid_init)
    for obj_i in range(len(objs_pos)):
        obj_i_grid = [round(ws_width/2 - (ws_center_pos[0] - objs_pos[obj_i][0])*100, 1), round(ws_depth/2 - (ws_center_pos[1] - objs_pos[obj_i][1])*100, 1)]
        obstacle_xyr_grid = [obj_i_grid[0], obj_i_grid[1], objs_r[obj_i]]
        grid_act = CUF.obstacle_circle(grid_act, obstacle_xyr_grid, 2)

    # Considering the y axis of the target while packing circles
    tar_grid = [round(ws_width/2 - (ws_center_pos[0] - tar_pos[0])*100, 1), round(ws_depth/2 - (ws_center_pos[1] - tar_pos[1])*100, 1)]
    for x_i in range(ws_width):
        for y_i in range(int(target_r*100*1.5)):
            if int(tar_grid[1])+y_i < ws_depth and int(tar_grid[1])+y_i > 0:
                grid_act[x_i][int(tar_grid[1])+y_i] = 2
                grid_act[x_i][int(tar_grid[1])-y_i] = 2

    grid_ori = grid_act
    bt_num = 3
    trial_num = 2000

    bt_circle = []
    circle_r = target_r
    for bt in range(bt_num):
        can_grid = []
        grid_can = copy.deepcopy(grid_ori)  # get original scene from the grid_set
        empt_grid, occu_grid = CUF.getEmpOcc(grid_can)
        for i in range(trial_num):
            pick_cen = np.random.randint(0, len(empt_grid))
            check_sum = 0
            for oc in range(len(occu_grid)):
                d_w = empt_grid[pick_cen][0] - occu_grid[oc][0]
                d_d = empt_grid[pick_cen][1] - occu_grid[oc][1]
                d_c = (d_w * d_w + d_d * d_d) ** 0.5 * GRID_SIZE
                if d_c <= circle_r:
                    check_sum = 1

            if check_sum == 0:
                can_grid.append(empt_grid[pick_cen])
                for em in range(len(empt_grid)):
                    d_w = empt_grid[pick_cen][0] - empt_grid[em][0]
                    d_d = empt_grid[pick_cen][1] - empt_grid[em][1]
                    d_c = (d_w * d_w + d_d * d_d) ** 0.5 * GRID_SIZE
                    if d_c <= circle_r:
                        grid_can[empt_grid[em][0]][empt_grid[em][1]] = 3
                        grid_can[empt_grid[pick_cen][0]][empt_grid[pick_cen][1]] = 3
                        occu_grid.append([empt_grid[em][0], empt_grid[em][1]])
        bt_circle.append([can_grid, grid_can])

    max_cir_num = []
    for i in range(len(bt_circle)):
        max_cir_num.append([len(bt_circle[i][0])])

    # print(max_cir_num.index(max(max_cir_num)))
    max_trial = max_cir_num.index(max(max_cir_num))

    grid_max_can = copy.deepcopy(bt_circle[max_trial][1])
    t_can_grid = bt_circle[max_trial][0]

    can_pos = []
    #ws_zero = [round(ws_cen[0] - ws_width * GRID_SIZE * 0.5, 2), round(ws_cen[1] - ws_depth * GRID_SIZE * 0.5, 2)]
    ws_zero = [ws_center_pos[0] - ws_size[0] * 0.5, ws_center_pos[1] - ws_size[1] * 0.5]

    for i in t_can_grid:
        xi, yi = i
        can_pos.append([ws_zero[0] + xi * GRID_SIZE, ws_zero[1] + yi * GRID_SIZE])
    return can_pos

#####################

## Functions computing poses
def compute_grasp_poses_rel(grasp_info, obj_info):
    cp = [grasp_info.grasp_cx, grasp_info.grasp_cy  - 0.0, grasp_info.grasp_cz]    
    pitch = np.arctan2(obj_info.bb_uv[0].x, obj_info.bb_uv[0].y) + math.pi/2
    if pitch > math.pi/2 or pitch < -math.pi/2:
	pitch = pitch - math.pi
    ap = [cp[0] - ((obj_info.bb_sc.x-0.02)/2)*math.cos(pitch), cp[1] + ((obj_info.bb_sc.x-0.02)/2)*math.sin(pitch), cp[2]]
    print 'cp', cp
    print 'ap (grasp position)', ap
    print 'pitch (deg)', math.degrees(pitch)
    return ap, pitch


def compute_grasp_poses_rel_obsolete(grasp_info, obj_info):
    #grasp_positions_rel = grasp_positions_KM_rel(object_name)
    cp = [grasp_info.grasp_cx, grasp_info.grasp_cy  - 0.0, grasp_info.grasp_cz]    
    bbox_info = grasp_info.gr_elements
    grasp_positions_rel = []
    for gr_info in bbox_info:
        grasp_positions_rel.append([cp[0], cp[1], cp[2], gr_info.grasp_0x, gr_info.grasp_0y, gr_info.grasp_0z, gr_info.grasp_1x, gr_info.grasp_1y, gr_info.grasp_1z, gr_info.grasp_app_x, gr_info.grasp_app_y, gr_info.grasp_app_z])

    grasp_poses_rel = []
    grasp_distances = []
    #print grasp_positions_rel
    for grasp_position_rel in grasp_positions_rel:
        cp = grasp_position_rel[0:3]
        gp = grasp_position_rel[3:6]#or [6:9] TODO (KIST-CHEONG): need to find
        ap = grasp_position_rel[9:12]
        grasp_pose_rel, rot_check = get_approaching_pose(cp, ap, gp)
        #grasp_euler = euler_from_quaternion(grasp_pose_rel[3:7]) 
	dist_ap_cp = distance(ap[0:2], cp[0:2])
	dist_ap_gp = distance(ap[0:2], gp[0:2])
	if dist_ap_cp > dist_ap_gp:
	    grasp_width = obj_info.bb_sc.y
	    pitch = np.arctan2(obj_info.bb_uv[0].x, obj_info.bb_uv[0].y) + math.pi/2
	    if pitch > math.pi/2 or pitch < -math.pi/2:
	        pitch = pitch - math.pi
	else:
	    grasp_width = obj_info.bb_sc.x
	    pitch = np.arctan2(obj_info.bb_uv[1].x, obj_info.bb_uv[1].y) + math.pi/2
	    if pitch > math.pi/2 or pitch < -math.pi/2:
	        pitch = pitch - math.pi

	    #print pitch
	#print grasp_euler
	#grasp_euler[1] = pitch
#distance(grasp_position_rel[3:6], grasp_position_rel[6:9])  # length of the grasping side (gripper open jaw size)
	print 'cp', cp
	print 'ap', ap
	print 'gp', gp
	print 'pitch (deg)', math.degrees(pitch)
	print 'width (m)', grasp_width
	#print rot_check

	if math.radians(-85) < pitch and math.radians(85) > pitch and grasp_width < 0.10 and ap[2] < cp[2] + 0.01 and ap[2] > cp[2] - 0.01 and not rot_check:  # Condition 1    
        #if math.radians(-85) < grasp_euler[1] and math.radians(85) > grasp_euler[1] and grasp_width < 0.10 and ap[2] < cp[2] + 0.01 and ap[2] > cp[2] - 0.01 and not rot_check:  # Condition 1
	    print 'ADDED'
            grasp_poses_rel.append(grasp_pose_rel)
            #grasp_center = grasp_position_rel[0:3]
            grasp_app = grasp_position_rel[9:12]            
            grasp_distances.append(grasp_app[0])
	    pitch_final = copy.deepcopy(pitch)
    #print grasp_poses_rel
    if not grasp_poses_rel:
      sys.exit("No possible grasps")
    grasp_poses_rel = [x for _,x in sorted(zip(grasp_distances, grasp_poses_rel))] # sort grasp_poses_abs based on the values in grasp_distances (in the ascending order)
    grasp_pose_rel = grasp_poses_rel[0]
    print 'grasp_position_rel', grasp_pose_rel[0:3], pitch_final
    return grasp_pose_rel[0:3], pitch_final


def compute_pregrasp_poses_direction(grasp_position_rel, angle):
    #x_offset = -0.15
    #y_offset = 0.0
    #z_offset = 0.05
    #print grasp_pose_rel

    #grasp_position_rel = [grasp_pos_rel[0], grasp_pose_rel[1], grasp_pose_rel[2]]
    goal_pitch = angle# grasp_pose_rel[4]
    arm_offset = 0.09
    
    dx = math.cos(goal_pitch - math.pi) * (arm_offset)
    dy = math.sin(goal_pitch - math.pi) * (arm_offset)
    goal_position = [grasp_position_rel[0] + dx, grasp_position_rel[1] - dy, grasp_position_rel[2]+ 0.05]
    #dx = math.cos(goal_pitch) * (arm_offset + 0.05)
    #dy = math.sin(goal_pitch) * (arm_offset + 0.05)
    #goal_position = [grasp_position_rel[0] - dx, grasp_position_rel[1] + dy, grasp_position_rel[2] + 0.05]
    print 'pregrasp (directional)', goal_position
    return goal_position


def compute_pregrasp_poses_round(grasp_position_rel, angle):
    goal_pitch = angle
    arm_offset = 0.14
    
    dx = math.cos(goal_pitch - math.pi) * (arm_offset )
    dy = math.sin(goal_pitch - math.pi) * (arm_offset )
    goal_position = [grasp_position_rel[0] + dx, grasp_position_rel[1] - dy, grasp_position_rel[2]+ 0.05]
    print 'pregrasp (round)', goal_position
    return goal_position


def compute_release_positions_rel(plane_name, target_position, grasp_info):
    goal_position = []
    plane_name = 'tablePlane'
    if plane_name == "displayMiddleShelf":
	print 'ask empty slot'
        release_position_id = get_empty_slot_in_display_KM_abs(plane_name)
	#release_position_id = 3
	print 'ask empty slot done'
        if release_position_id == 1:
	    print 'Place to empty slot 1'
            goal_position.append(0.80) 
            goal_position.append(0.22)
        elif release_position_id == 2:
	    print 'Place to empty slot 2'
            goal_position.append(0.80) 
            goal_position.append(-0.02)
        elif release_position_id == 3:
	    print 'Place to empty slot 3'
            goal_position.append(0.80) 
            goal_position.append(-0.26)
	else:
	    sys.exit('ERROR: Compute release, no such empty slot')
	goal_position.append(0.98)
	goal_positions = [goal_position]
    else:

	w = 0.355
	d = 0.335 / 2.0
	phi = math.atan2(-w, d)# back markers
	diagonal = math.sqrt((w**2 + d**2))
	marker_1, marker_2 = get_markers()
	if not marker_1 or not marker_2:
	    print "Failed to get markers"
	    marker_1 = [0.674191157902, 0.248624965549, 0.893736226085]
	    marker_2 = [1.00831483875, 0.249981209636, 0.902455484087]
	l = math.sqrt(((marker_1[0]-marker_2[0])**2)+((marker_1[1]-marker_2[1])**2))
	
	theta_get = math.asin((marker_2[1]-marker_1[1])/(2*d))
	z_axis = (0.0, 0.0, 1)
	Rz = ttf.rotation_matrix(theta_get, z_axis) 
	plane_x = marker_1[0]+(diagonal)*math.cos(theta_get+phi)
	plane_y = marker_1[1]+(diagonal)*math.sin(theta_get+phi)
	plane_size = [0.25, 0.4]#depth, width
	plane_center = [plane_x - 0.03, plane_y - 0.1]
	target_size = 0.06
	goal_positions = []
	obstacle_positions = []
	obstacle_sizes = []
	if not grasp_info.grsarr[0].gr:
	    print "No object detected"
	    num_objs = 0
	else:
	    num_objs = len(grasp_info.grsarr[0].gr)
	for i in range(0, num_objs):
	    obstacle_positions.append([grasp_info.grsarr[0].gr[i].grasp_cx, grasp_info.grsarr[0].gr[i].grasp_cy])
	    obstacle_sizes.append(max(abs(grasp_info.objarr[0].pt[i].bb_sc.x), abs(grasp_info.objarr[0].pt[i].bb_sc.y)))

	relocate_positions = compute_empty_slots(plane_size, plane_center, target_position, target_size, obstacle_positions, obstacle_sizes)
	if not relocate_positions:
		relocate_positions = compute_empty_slots(plane_size, plane_center, target_position, target_size, obstacle_positions, obstacle_sizes)


	if relocate_positions:
	    relocate_positions_x = []
	    for i in range(0, len(relocate_positions)):
		relocate_positions_x.append(relocate_positions[i][0])
	    relocate_positions = [x for _,x in sorted(zip(relocate_positions_x, relocate_positions))]
	else:
	    relocate_positions = [[random.uniform(0.71, 0.73), random.uniform(-0.20, 0.0)]]

	for relocate_position in relocate_positions:
	    goal_positions.append([relocate_position[0], relocate_position[1], 0.98])
    return goal_positions
	
        #plane_size = [0.4, 0.8, 0.90]#get_plane_size_KM(plane_name)        
	
	#x_rel = random.uniform(0.71, 0.73)
        #y_rel = random.uniform(-0.20, 0.10)

        #goal_position.append(x_rel)
        #goal_position.append(y_rel)
    #goal_position.append(0.90 + 0.17/2 )
    #print 'release position', goal_position
    #return goal_position


def get_approaching_pose(cp, ap, gp):    
    gp = np.array(gp)#np.array([0.0, 1.0, 0.0])#
    cp = np.array(cp)#np.array([0.0, 0.0, 0.0])#
    ap = np.array(ap)#np.array([-1.0, 0.0, 0.0])#
    #print gp, cp, ap
    l_x = np.linalg.norm(gp - cp)   # size of vector x
    l_z = np.linalg.norm(ap - cp)   # size of vector z
    #print "dx, dz", l_x, l_z
    u_x = (gp - cp) / l_x   # unit vector x from the given data
    u_z = (ap - cp) / l_z   # unit vector z from the given data
    u_y = np.cross(u_z, u_x)    # compute unit vector y from crossing the unit vector z to x
    #print u_x, u_y, u_z
    rot_mat = [u_x, u_y, u_z]
    #print "rotation matrix:\n", rot_mat
    #print "transposed:\n", np.transpose(rot_mat)
    ret = rotationMatrixToEulerAngles(np.transpose(rot_mat))    # compute the euler angle from the rotation matrix
    #print "return:", ret
    ans = quaternion_from_euler(ret[0], ret[1], ret[2], 'ryxz') # get quaternion from euler angle
    # print "answer:", ans
    
    t_pos = list(ap)#[0.80, -0.17, 0.97]#
    t_ori = list(ans)
    #print "get_approaching (position, euler): ", t_pos, ret[0:3]
    t_pos.extend(t_ori)
    return t_pos, isRotationMatrix(ret)     # [x, y, z, qx, qy, qz, qw]

def get_approaching_angle(hand_pos, obj_pos):
    dx = obj_pos[0] - hand_pos[0]
    dy = obj_pos[1] - hand_pos[1]
    theta = math.atan2(dy, dx)
    angle_deg = math.degrees(-1*theta)
    return angle_deg


# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    #assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


# distance(x, y): compute the Euclidean distance between 2D points x and y
def distance(point_one, point_two):
    #print point_one, point_two
    if len(point_one) == 2:
        return ((point_one[0] - point_two[0]) ** 2 +
                (point_one[1] - point_two[1]) ** 2) ** 0.5
    elif len(point_one) == 3:
        return ((point_one[0] - point_two[0]) ** 2 + (point_one[1] - point_two[1]) ** 2 + (
                    point_one[2] - point_two[2]) ** 2) ** 0.5
    else:
        sys.exit("ERROR: distance")


######################
def get_r_hand_encoder_callback(data):
    global encoder_joints
    encoder_joints = data
    #print "data:", encoder_joints.position[-2], type(encoder_joints)

def get_r_hand_encoder():
    rospy.sleep(0.3)
    a = rospy.Subscriber('/encoder_joint_states', JointState, get_r_hand_encoder_callback)


def get_obj_info():
    import rospy
    from obj_msg.srv import *
    rospy.wait_for_service('/grs_signal_opc')
    try:
        srv_check = rospy.ServiceProxy('/grs_signal_opc', GenGrsInfo)
        pub_msg = GenGrsInfoRequest()
        pub_msg.grs_flg = True

        resp1 = srv_check(pub_msg)
        return resp1
        #print "response", resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
	return 0


def make_obj2rviz(object_name, obj_pos, obj_scale, obj_info):
    #obj_info = get_obj_info()
    #bb_cp = obj_info.grsarr[0].gr[0]
    #obj_pos = [bb_cp.grasp_cx, bb_cp.grasp_cy, bb_cp.grasp_cz]
    # print obj_info.objarr[0].pt[0].bb_uv[0], type(obj_info.objarr[0].pt[0].bb_uv[0])
    bb_uv_x = obj_info.bb_uv[0]
    bb_uv_y = obj_info.bb_uv[1]
    bb_uv_z = obj_info.bb_uv[2]

    u_x = [bb_uv_x.x, bb_uv_x.y, bb_uv_x.z]
    u_y = [bb_uv_y.x, bb_uv_y.y, bb_uv_y.z]
    u_z = [bb_uv_z.x, bb_uv_z.y, bb_uv_z.z]
    
    rot_mat = [u_x, u_y, u_z]
    
    ret = rotationMatrixToEulerAngles(np.transpose(rot_mat))    # compute the euler angle from the rotation matrix
    ans = quaternion_from_euler(ret[0], ret[1], ret[2], 'ryxz') # get quaternion from euler angle
    
    obj_ori = list(ans)

    #bb_scale = obj_info.objarr[0].pt[0].bb_sc
    ##obj_scale = [bb_scale.x, bb_scale.y, bb_scale.z]
    
    CLF.add_box_client(object_name, obj_pos, obj_ori, obj_scale, 'red') #cantata
    # grasp_positions_rel.grsarr[0].gr[0].gr_elements  


def get_markers():
    import rospy
    from obj_msg.srv import *
    rospy.wait_for_service('/AR_Marker_Service')
    try:
        m1 = 0
        m2 = 0
        print "Get AR markers"
        srv_check = rospy.ServiceProxy('/AR_Marker_Service', CameraRequests)
        pub_msg = CameraRequestsRequest()
        pub_msg.a = 0

        resp1 = srv_check(pub_msg)
	if not resp1:
            resp1 = srv_check(pub_msg)
	if not resp1:
            resp1 = srv_check(pub_msg)
	if not resp1:
            m1s = []
	    m2s = []
	else:
            m1 = resp1.pose_array.poses[0].position
            m2 = resp1.pose_array.poses[1].position
	    if m1.x < m2.x:
                m1s = [m1.x, m1.y, 0, 0]
                m2s = [m2.x, m2.y, 0, 0]
       	    else:
                m1s = [m2.x, m2.y, 0, 0]
                m2s = [m1.x, m1.y, 0, 0]
        #print "marker 1:\n", resp1.pose_array.poses[0].position
        #print "marker 2:\n", resp1.pose_array.poses[1].position
        return m1s, m2s
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def gripper_close(gripper_cmd):
    #return 0
    act_client = actionlib.SimpleActionClient('rospodo_gripper', ros_podo_connector.msg.RosPODO_GripperAction)
    act_client.wait_for_server()
    
    goal_gripper = ros_podo_connector.msg.RosPODO_GripperGoal()
    goal_gripper.grippermove_cmd = 3
    goal_gripper.mode = 1
    #print "type", goal_gripper, type(goal_gripper)

    act_client.send_goal(goal_gripper)
    act_client.wait_for_result()

    return act_client.get_result()


def gripper_open(gripper_cmd):
    #return 0
    act_client = actionlib.SimpleActionClient('rospodo_gripper', ros_podo_connector.msg.RosPODO_GripperAction)
    act_client.wait_for_server()
    
    goal_gripper = ros_podo_connector.msg.RosPODO_GripperGoal()
    goal_gripper.grippermove_cmd = 2
    goal_gripper.mode = 1
    #print "type", goal_gripper, type(goal_gripper)

    act_client.send_goal(goal_gripper)
    act_client.wait_for_result()

    return act_client.get_result()


def convert2action_arm(plan, move_group):
    import time
    # print type(plan), plan.joint_names, plan.joint_names
    # for i in range(10):
    #     print i, "th time:", type(plan.points[i].time_from_start.to_sec()), plan.points[i].time_from_start.to_sec()
    #     print i, "th pos:", plan.points[i].positions
    #tic = time.time()
    flag_action = 0
    rospy.sleep(0.1)
    act_client = actionlib.SimpleActionClient("rospodo_trajectory", ros_podo_connector.msg.RosPODO_TrajectoryAction)
    act_client.wait_for_server()

    action_goal = actionlib_tutorials.msg.RosPODO_TrajectoryGoal()
    #print "Action length:", len(action_goal.via_point)

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
    #for i in range(5):
    #    print "action goal", action_goal.via_point[i].joint[0].GoalmsTime
    #rospy.sleep(1)
    #toc = time.time()
    #print 'Preprocessing time for execution (sec):', toc - tic
    act_client.send_goal(action_goal)
    act_client.wait_for_result()
    #global finish
    #finish = True
    flag_action = 1
    print 'Convert2Action done', flag_action
    return flag_action


def make_env_shelf(dx, dy, theta):

    z_axis = (0.0, 0.0, 1)
    Rz = ttf.rotation_matrix(theta, z_axis)
    rot_mat = Rz
    #print "rotation matrix:\n", rot_mat
    #print "transposed:\n", np.transpose(rot_mat)
    ret = rotationMatrixToEulerAngles(np.transpose(rot_mat))    # compute the euler angle from the rotation matrix
    # print "rotation matrix:\n", ret
    ans = quaternion_from_euler(ret[0], ret[1], ret[2], 'ryxz') # get quaternion from euler angle
    # print "answer:", ans
    
    r_side_0 = 0.475
    r_side_1 = 0.225
    r_side_2 = 0.475

    #print "input theta:", theta
    #print "input theta:", math.degrees(theta)
    #print "input theta:", theta
    
    side0 = [dx-r_side_0*math.sin(math.radians(180)-theta), dy+r_side_0*math.cos(math.radians(180)-theta)]
    side1 = [dx+r_side_1*math.sin(math.radians(90)-theta), dy-r_side_1*math.cos(math.radians(90)-theta)]
    side2 = [dx-r_side_2*math.sin(math.radians(0)-theta), dy+r_side_2*math.cos(math.radians(0)-theta)]

    # side0 = np.matmul(np.array(rot_mat), np.array([0, 0 - 0.95/2.0 + 0.018/2.0, 0, 0]))
    # side1 = np.matmul(np.array(rot_mat), np.array([0 + 0.45/2.0 - 0.018/2.0, 0, 0, 0]))
    # side2 = np.matmul(np.array(rot_mat), np.array([0, 0 + 0.95/2.0 - 0.018/2.0, 0, 0]))
    
    t_ori = list(ans)

    env_name = ['shelf0', 'shelf1', 'shelf2', 'side0', 'side1', 'side2']
    env_info = []
    env_info.append([[0 + dx, 0 + dy, 0.90/2.0], t_ori, [0.45 + 0.1 + 0.1, 0.95 + 0.1, 0.90]])  # information for shelf0
    env_info.append([[0 + dx, 0 + dy, 1.30 - 0.018/2.0], t_ori, [0.45 + 0.1 + 0.1, 0.95 + 0.1, 0.018 + 0.08]])  # information for shelf1
    env_info.append([[0 + dx, 0 + dy, 1.50 - 0.018/2.0], t_ori, [0.45 + 0.1 + 0.1, 0.95 + 0.1, 0.018]])  # information for shelf2

    env_info.append([[side0[0], side0[1], 1.50/2.0], t_ori, [0.45 + 0.1 + 0.1, 0.018 + 0.05, 1.5]])  # information for side0
    env_info.append([[side1[0], side1[1], 1.50/2.0], t_ori, [0.018 + 0.05, 0.95 + 0.1, 1.5]])  # information for side1
    env_info.append([[side2[0], side2[1], 1.50/2.0], t_ori, [0.45 + 0.1 + 0.1, 0.018 + 0.05, 1.5]])  # information for side2

    # env_info.append([[side0[0], side0[1], 1.50/2.0], t_ori, [0.45 + 0.1, 0.018 + 0.05, 1.5]])  # information for side0
    # env_info.append([[side1[0], side1[1], 1.50/2.0], t_ori, [0.018 + 0.05, 0.95 + 0.1, 1.5]])  # information for side1
    # env_info.append([[side2[0], side2[1], 1.50/2.0], t_ori, [0.45 + 0.1, 0.018 + 0.05, 1.5]])  # information for side2
   
    # env_info.append([[0 + dx, 0 + dy - 0.95/2.0 + 0.018/2.0, 1.50/2.0], [0, 0, 0, 0], [0.45 + 0.1, 0.018 + 0.05, 1.5]])  # information for side0
    # env_info.append([[0 + dx + 0.45/2.0 - 0.018/2.0, 0 + dy, 1.50/2.0], [0, 0, 0, 0], [0.018 + 0.05, 0.95 + 0.1, 1.5]])  # information for side1
    # env_info.append([[0 + dx, 0 + dy + 0.95/2.0 - 0.018/2.0, 1.50/2.0], [0, 0, 0, 0], [0.45 + 0.1, 0.018 + 0.05, 1.5]])  # information for side2

    for i in range(len(env_info)):
        CLF.add_box_client(env_name[i], env_info[i][0], env_info[i][1], env_info[i][2], 'gray')


def del_shelf_env():
    env_name = ['shelf0', 'shelf1', 'shelf2', 'side0', 'side1', 'side2']
    for i in env_name:
        CLF.del_box_client(i)


def make_env_counter(dx, dy, theta):

    z_axis = (0.0, 0.0, 1)
    Rz = ttf.rotation_matrix(theta, z_axis)
    rot_mat = Rz
    #print "rotation matrix:\n", rot_mat
    #print "transposed:\n", np.transpose(rot_mat)
    ret = rotationMatrixToEulerAngles(np.transpose(rot_mat))    # compute the euler angle from the rotation matrix
    #print "return:", ret
    ans = quaternion_from_euler(ret[0], ret[1], ret[2], 'ryxz') # get quaternion from euler angle
    # print "answer:", ans
    
    t_ori = list(ans)

    env_name = ['counter']
    env_info = []
    env_info.append([[0 + dx, 0 + dy, 0.90 / 2.0], t_ori, [0.41 + 0.17 + 0.1, 0.8 + 0.17, 0.90]])  # information for counter
    for i in range(len(env_info)):
        CLF.add_box_client(env_name[i], env_info[i][0], env_info[i][1], env_info[i][2], 'gray')
    return 1


def get_env_fur(env_furniture): # env_furniture: 'table_env' or 'display_env' or 'storage_env'
    import numpy as np
    import math    
    import matplotlib.pyplot as plt
    import D_0902_client_function as CLF

    print "Environment name: ", env_furniture
    if env_furniture == 'table_env':
        w = 0.295
        d = 0.335 / 2.0
        
        #print "w,d ", w, d
        phi = math.atan2(-w, d)  # back markers
        #print "phi:", math.degrees(phi)
        diagonal = math.sqrt((w**2 + d**2))
        #print "w, d, phi, dig:", w, d, math.degrees(phi), diagonal

        marker_1, marker_2 = get_markers()
        if not marker_1 or not marker_2:
            print "Failed to get markers"
	    marker_1 = [0.684040672294, 0.157788410783, 0.886796156013]
            marker_2 = [1.01665247898, 0.150018811226, 0.882896661472]
        #print "marker_1 (x,y):", marker_1
        #print "marker_2 (x,y):", marker_2
        # marker_c = [marker_1[0]+(diagonal/2.0)*math.cos(theta+phi), marker_1[1]+(diagonal/2.0)*math.sin(theta+phi), 0, 0]

        l = math.sqrt(((marker_1[0]-marker_2[0])**2)+((marker_1[1]-marker_2[1])**2))
        #print "depth:", l
        theta_get = math.asin((marker_2[1]-marker_1[1])/(2*d))

        # theta = math.radians(10)
        #print "get_theta", math.degrees(theta_get)

        z_axis = (0.0, 0.0, 1)
        Rz = ttf.rotation_matrix(theta_get, z_axis)

        #print "total angle:", math.degrees(theta_get+phi), "cos, sin:", math.cos(theta_get+phi), math.sin(theta_get+phi)
        #print "add x:", (diagonal)*math.cos(theta_get+phi), "add y:", (diagonal)*math.sin(theta_get+phi)
        marker_c = [marker_1[0]+(diagonal)*math.cos(theta_get+phi), marker_1[1]+(diagonal)*math.sin(theta_get+phi), 0, 0]
        #print "center (x,y):", marker_c

        make_env_counter(marker_c[0], marker_c[1], -theta_get)

    elif env_furniture == 'display_env':
        w = 0.355
        d = 0.335 / 2.0
        
        #print "w,d ", w, d
        phi = math.atan2(-w, d)  # back markers
        #print "phi:", math.degrees(phi)
        diagonal = math.sqrt((w**2 + d**2))
        #print "dig:", diagonal

        marker_1, marker_2 = get_markers()
        if not marker_1 or not marker_2:
            print "Failed to get markers"
            marker_1 = [0.674191157902, 0.248624965549, 0.893736226085]
            marker_2 = [1.00831483875, 0.249981209636, 0.902455484087]
        # marker_c = [marker_1[0]+(diagonal/2.0)*math.cos(theta+phi), marker_1[1]+(diagonal/2.0)*math.sin(theta+phi), 0, 0]

        l = math.sqrt(((marker_1[0]-marker_2[0])**2)+((marker_1[1]-marker_2[1])**2))
        #print "depth:", l
        theta_get = math.asin((marker_2[1]-marker_1[1])/(2*d))

        # theta = math.radians(10)
        #print "get_theta", math.degrees(theta_get)

        z_axis = (0.0, 0.0, 1)
        Rz = ttf.rotation_matrix(theta_get, z_axis)

        #print "total angle:", math.degrees(theta_get+phi), "cos, sin:", math.cos(theta_get+phi), math.sin(theta_get+phi)
        #print "add x:", (diagonal)*math.cos(theta_get+phi), "add y:", (diagonal)*math.sin(theta_get+phi)
        marker_c = [marker_1[0]+(diagonal)*math.cos(theta_get+phi), marker_1[1]+(diagonal)*math.sin(theta_get+phi), 0, 0]
        #print "center (x,y):", marker_c
   
        make_env_shelf(marker_c[0], marker_c[1], -theta_get)

    elif env_furniture == 'storage_env':
        w = 0.355
        d = 0.335 / 2.0
        
        #print "w,d ", w, d
        phi = math.atan2(-w, d)  # back markers
        #print "phi:", math.degrees(phi)
        diagonal = math.sqrt((w**2 + d**2))
        #print "dig:", diagonal

        marker_1, marker_2 = get_markers()
        if not marker_1 or not marker_2:
            print "Failed to get markers"
            marker_1 = [0.674191157902, 0.248624965549, 0.893736226085]
            marker_2 = [1.00831483875, 0.249981209636, 0.902455484087]
        # marker_c = [marker_1[0]+(diagonal/2.0)*math.cos(theta+phi), marker_1[1]+(diagonal/2.0)*math.sin(theta+phi), 0, 0]

        l = math.sqrt(((marker_1[0]-marker_2[0])**2)+((marker_1[1]-marker_2[1])**2))
        #print "depth:", l
        theta_get = math.asin((marker_2[1]-marker_1[1])/(2*d))

        # theta = math.radians(10)
        #print "get_theta", math.degrees(theta_get)

        z_axis = (0.0, 0.0, 1)
        Rz = ttf.rotation_matrix(theta_get, z_axis)

        #print "total angle:", math.degrees(theta_get+phi), "cos, sin:", math.cos(theta_get+phi), math.sin(theta_get+phi)
        #print "add x:", (diagonal)*math.cos(theta_get+phi), "add y:", (diagonal)*math.sin(theta_get+phi)
        marker_c = [marker_1[0]+(diagonal)*math.cos(theta_get+phi), marker_1[1]+(diagonal)*math.sin(theta_get+phi), 0, 0]
        #print "center (x,y):", marker_c
   
        make_env_shelf(marker_c[0], marker_c[1], -theta_get)



def try_path_plan(goal_pos, app_angle):
    import math
    from tf.transformations import quaternion_from_euler
    import time
    print 'Motion planning start'
    planner_name = 'RRTConnect'
    #planner_name = 'BiTRRT'
    n_attempt = 1000
    c_time = 0.5
    n_repeat = 10
    start_state = moveit_msgs.msg.RobotState()
    joint_state = sensor_msgs.msg.JointState()
    joint_state.header = std_msgs.msg.Header()
    joint_state.name =  ['RSP', 'RSR', 'RSY', 'REB', 'RWY', 'RWP', 'RWY2']
    get_r_hand_encoder()
    rospy.sleep(0.3)
    joint_state.position = encoder_joints.position[0:8] 
    start_state.joint_state = joint_state

    # Set the grasp pose: substract 17cm from the z value of the object centroid
    goal_pitches = []
    goal_pitch = np.deg2rad(app_angle)
    goal_pitches.append(goal_pitch)
    for i in range(5):
        goal_pitches.append(goal_pitch + (i + 1) * math.radians(8.0))
        goal_pitches.append(goal_pitch - (i + 1) * math.radians(8.0))
    # Get the grasp orientation (currently the front direction)
    goal_orientations = []
    for i in goal_pitches:
        goal_orientations.append(quaternion_from_euler(math.radians(-90.0), math.radians(-90.0) + i, math.radians(5.0), axes='rxyz'))
        #goal_orientations.append(quaternion_from_euler(math.radians(-90.0), math.radians(-90.0) + i, 0, axes='rxyz'))

    l = 0.21
    goal_poses = []
    for i in goal_pitches:
        
        dx = math.cos(i) * l
	dy = math.sin(i) * l
        goal_poses.append([goal_pos[0] - dx, goal_pos[1] + dy, goal_pos[2]])

    feasibility1 = 0
    i = 0
    tic_feasible = time.time()
    while not feasibility1 and i < len(goal_pitches):
        print i, "th try", goal_pitches[i], "added"
        [feasibility1, trajectory1] = CLF.move_goalpose_client('R_arm', 'R_Hand', start_state, goal_poses[i], goal_orientations[i], [], planner_name, n_attempt, c_time, n_repeat)
        idx = i
        i = i + 1
    toc_feasible = time.time()
    if feasibility1:
        retGoal = convert2action_arm(trajectory1.joint_trajectory, 'R_arm')
	if retGoal == 0:
    	    feasibility1 = 0
    print 'Done: motion planning time (sec):', toc_feasible - tic_feasible
    return feasibility1, math.degrees(goal_pitches[idx])

def test_navigation_action(gx, gy, gtheta, marker_flag, navi_flag):
    import arm_move.msg
    
    print "Navigation (x,y, theta, marker):", gx, gy, gtheta, marker_flag
    #print "                   (theta):", gtheta
    #print "             (marker flag):", marker_flag
    base_action_cli = actionlib.SimpleActionClient("hubo_navigation", arm_move.msg.naviAction)
    base_action_cli.wait_for_server()

    action_goal = arm_move.msg.naviGoal()

    action_goal.use_marker = marker_flag
    action_goal.use_navi = navi_flag #0: local, 1: global
    action_goal.pose_x = gx
    action_goal.pose_y = gy
    action_goal.pose_z = 0

    goal_quaternion = quaternion_from_euler(0.0, 0.0, math.radians(gtheta))

    action_goal.ori_x = goal_quaternion[0]
    action_goal.ori_y = goal_quaternion[1]
    action_goal.ori_z = goal_quaternion[2]
    action_goal.ori_w = goal_quaternion[3]

    #print '1'
    base_action_cli.send_goal(action_goal)
    #print '2'
    base_action_cli.wait_for_result()
    #print '3'
    
    print "Navigation done"
    return 1


#### Services
def grasp_arm_service(data, timeout=4):        
	print 'grasp_arm_service called'#, hand_name, target_name
	hand_name = 'R_hand'
	
	target_name = data.actionType[0]
	target_name = target_name.split('#')[1]
	target_name = target_name[:-1]
	
	place_name = data.object[0]
	place_name = place_name.split('#')[1]
	place_name = place_name[:-1]

	
	print hand_name, target_name, place_name
	
	#home_position = [0.1660 + 0.27, -0.2465, 1.0]#0.7629]
	home_position = [0.336, -0.3465, 1.0]
	
        print '*** Object detection and analysis ***'

	grasp_positions_rel = get_obj_info()  # grasp_positions_KM_rel(object_name)
	if grasp_positions_rel == 0:
	    print "Object detection failed. Retry"
	    grasp_positions_rel = get_obj_info()
	if grasp_positions_rel == 0:
	    print "Object detection failed. Retry"
	    grasp_positions_rel = get_obj_info()
	    response = ActionServiceResponse(result=5)
	    return response

	if not grasp_positions_rel.grsarr[0].gr:
	    print "No object detected."
	    num_objs = 0
	else:
	    num_objs = len(grasp_positions_rel.grsarr[0].gr)
	object_list = []
	all_object_list = []
	for i in range(0, num_objs):
		if grasp_positions_rel.grsarr[0].gr[i].id == 1:
			object_name = 'red_gotica'
		elif grasp_positions_rel.grsarr[0].gr[i].id == 2:
			object_name = 'bakey'
		elif grasp_positions_rel.grsarr[0].gr[i].id == 3:
			object_name = 'pringles_onion'
		elif grasp_positions_rel.grsarr[0].gr[i].id == 4:
			object_name = 'diget_box'
		elif grasp_positions_rel.grsarr[0].gr[i].id == 5:
			object_name = 'diget'
		elif grasp_positions_rel.grsarr[0].gr[i].id == 6:
			object_name = 'gotica'
		elif grasp_positions_rel.grsarr[0].gr[i].id == 8:
			object_name = 'diget_small_box'
		else:
			sys.exit('ERROR: no object name in DB')
		size_x = abs(grasp_positions_rel.objarr[0].pt[i].bb_sc.x) 
		size_y = abs(grasp_positions_rel.objarr[0].pt[i].bb_sc.y)
		size_z = abs(grasp_positions_rel.objarr[0].pt[i].bb_sc.z) + 0.02
		obj_size = [size_x, size_y, size_z]	
		obj_pos = [grasp_positions_rel.grsarr[0].gr[i].grasp_cx, grasp_positions_rel.grsarr[0].gr[i].grasp_cy - 0.0, grasp_positions_rel.grsarr[0].gr[i].grasp_cz - 0.02]
		print 'Object detected: ', object_name, obj_pos
		if object_name == 'diget_box' or object_name == 'bakey':
		    if obj_size[0] < obj_size[1]:
			obj_size[0] = 0.03
			obj_size[1] = 0.17
		    else:
			obj_size[0] = 0.17
			obj_size[1] = 0.03
                    if object_name == target_name:			
			make_obj2rviz(object_name, obj_pos, obj_size, grasp_positions_rel.objarr[0].pt[i])
			grasp_position, pitch = compute_grasp_poses_rel(grasp_positions_rel.grsarr[0].gr[i], grasp_positions_rel.objarr[0].pt[i])
			grasp_position[2] = 0.95
			pregrasp_position = compute_pregrasp_poses_direction(grasp_position, pitch)
			pregrasp_position[2] = 1.0
			approaching_angle = math.degrees(pitch)
			print approaching_angle
		    else:
			make_obj2rviz(object_name, obj_pos, obj_size, grasp_positions_rel.objarr[0].pt[i])				
			object_list.append(object_name)
		elif object_name == 'diget_small_box':	
		    if object_name == target_name:
		        obj_size[0] = 0.03
			obj_size[1] = 0.03			
			make_obj2rviz(object_name, obj_pos, obj_size, grasp_positions_rel.objarr[0].pt[i])
			grasp_position, pitch = compute_grasp_poses_rel(grasp_positions_rel.grsarr[0].gr[i], grasp_positions_rel.objarr[0].pt[i])
			grasp_position = copy.deepcopy(obj_pos)
			grasp_position[2] = 0.95
			pregrasp_position = copy.deepcopy(grasp_position)
			pregrasp_position[0] = pregrasp_position[0] - 0.07
			pregrasp_position[2] = 1.0
			approaching_angle = math.degrees(pitch)
			if approaching_angle > 0:
			    approaching_angle = approaching_angle - 90.0
			print 'approaching_angle for diget_small_box', approaching_angle
		    else:
			obj_size[0] = 0.04
			obj_size[1] = 0.04			
			make_obj2rviz(object_name, obj_pos, obj_size, grasp_positions_rel.objarr[0].pt[i])				
			object_list.append(object_name)
		else:
		    if object_name == target_name:
			obj_size[0] = 0.03
			obj_size[1] = 0.03
			approaching_angle = -10#get_approaching_angle(home_position[0:2], obj_pos[0:2])#-10.0
			print' approaching_angle (round)', approaching_angle
			CLF.add_box_client(object_name, obj_pos, [0.0, 0.0, 0.0, 0.0], obj_size, 'red')
			grasp_position = copy.deepcopy(obj_pos)        
			pregrasp_position = compute_pregrasp_poses_round(grasp_position, math.radians(approaching_angle))#copy.deepcopy(grasp_position)
			grasp_position[2] = 0.95
			pregrasp_position[2] = 1.0			
			#print grasp_position, pregrasp_position
		    else:
			obj_size[0] = 0.04
			obj_size[1] = 0.04
			CLF.add_box_client(object_name, obj_pos, [0.0, 0.0, 0.0, 0.0], obj_size, 'green')
			object_list.append(object_name)
		all_object_list.append(object_name)
        print '*** Object detection done ***', object_list
        if target_name not in all_object_list:
            print 'FAIL: target not detected'
            response = ActionServiceResponse(result=5)
	    for object_name in object_list:
		CLF.del_box_client(object_name)

	    #delete env (not the target)
	    print '*** Motion planning environment clear ***'
	    if place_name == "table_env":
		CLF.del_box_client('counter')
	    elif place_name == "display_env":
		del_shelf_env()
	    elif place_name == "storage_env":
		del_shelf_env()
	    else:
		sys.exit("No such place name")
	    return response
        else:
            print '*** Motion planning environment generation ***'
            if place_name == "table_env":
                get_env_fur(place_name) # env_furniture: 'table_env' or 'display_env' or 'storage_env'
            elif place_name == "display_env":
                get_env_fur(place_name)
            elif place_name == "storage_env":
                get_env_fur(place_name)
            else:
                sys.exit("No such place name")

        # Open hand
	print '*** Ready ***'
        try_path_plan(home_position, -45)
        print '*** Gripper open ***'
        rospy.sleep(0.01)
        gripper_open('open')
        rospy.sleep(0.01)

        # pregrasp
        print '*** Pre-grasp ***', pregrasp_position
        result1 = try_path_plan(pregrasp_position, approaching_angle)
        rospy.sleep(0.01)
        
        # grasp
        print '*** Grasp ***', grasp_position
        result2, prev_angle = try_path_plan(grasp_position, approaching_angle)
        rospy.sleep(0.01)
        #if result2 == 0:
        #    result2, prev_angle = try_path_plan(grasp_position, approaching_angle-5.0)
        #    rospy.sleep(0.01)
        #if result2 == 0:
        #    result2, prev_angle = try_path_plan(grasp_position, approaching_angle-15.0)
        #    rospy.sleep(0.01)
        
        
	result3 = 0
	if result2 == 1:
	    # close
            print '*** Gripper close ***'
            rospy.sleep(0.01)
            gripper_close('close')
            rospy.sleep(0.01)
            CLF.att_box_client('R_hand', target_name)

            # postgrasp
            print '*** Post-grasp ***'
            result3 = try_path_plan(pregrasp_position, prev_angle)
            rospy.sleep(0.01)
            #if result3 == 0:
            #    result3, prev_angle = try_path_plan(grasp_position, prev_angle-5.0)
            #    rospy.sleep(0.01)
            #if result3 == 0:
            #   result3, prev_angle = try_path_plan(grasp_position, prev_angle+5.0)
            #    rospy.sleep(0.01)
	else:
	    print '*** Gripper close ***'
            rospy.sleep(0.01)
            gripper_close('close')
            rospy.sleep(0.01)
        
        # home
        print '*** Home ***'
	#if object_name == 'diget_box' or object_name == 'diget_small_box'  or object_name == 'bakey':
        #    result4 = try_path_plan(home_position, -60)
	#else:
        result4 = try_path_plan(home_position, -45)#-20)

        rospy.sleep(0.01)

        print '*** Gripper close ***'
        rospy.sleep(0.01)
        gripper_close('close')
        rospy.sleep(0.01)

        get_r_hand_encoder()
        rospy.sleep(0.3)

        for object_name in object_list:
            CLF.del_box_client(object_name)

        # delete env (not the target)
        print '*** Motion planning environment clear ***'
        if place_name == "table_env":
            CLF.del_box_client('counter')
        elif place_name == "display_env":
            del_shelf_env()
        elif place_name == "storage_env":
            del_shelf_env()
        else:
            sys.exit("No such place name")


        response = ActionServiceResponse(result=0)#success
        if encoder_joints.position[-2] > -10 and result4 < 0.01:
            print '!!! FAIL !!! Could not grasp, could not come back home'
            response = ActionServiceResponse(result=1) #fail: can't grasp and stop moving
	    CLF.det_box_client(target_name)
	    CLF.del_box_client(target_name)
        elif encoder_joints.position[-2] > -10 and result4 > 0.99:
            print '!!! FAIL !!! Could not grasp, came home'
            response = ActionServiceResponse(result=2) #fail: can't grasp but finished moving
	    CLF.det_box_client(target_name)
	    CLF.del_box_client(target_name)
        elif encoder_joints.position[-2] <= -10 and result4 < 0.01:
            print '!!! FAIL !!! Grasped but could not come back home'
            response = ActionServiceResponse(result=3) #fail: grasped but stop moving
        if result1 < 0.01 and result2 < 0.01 and result3 < 0.01 and result4 < 0.01:
            print '!!! FAIL !!! Motion planning failed for all poses'
            response = ActionServiceResponse(result=4)  # fail: motion planning fails for all poses
	    CLF.det_box_client(target_name)
	    CLF.del_box_client(target_name)
	return response

def release_arm_service(data, timeout=4):
	print 'release_arm_service called'#, hand_name, target_name, plane_name, place_name
	hand_name = 'R_hand'
	
	target_name = data.actionType[0]
	target_name = target_name.split('#')[1]
	target_name = target_name[:-1]
	
	plane_name = data.object[0]
	plane_name = plane_name.split('#')[1]
	plane_name = plane_name[:-1]
	
	place_name = data.targetPose[0]
	place_name = place_name.split('#')[1]
	place_name = place_name[:-1]

	home_position = [0.336, -0.3465, 1.0]

        print '*** Object detection and analysis ***'
	grasp_positions_rel = get_obj_info()  

	if grasp_positions_rel == 0:
	    print "Object detection failed. Retry"
	    grasp_positions_rel = get_obj_info()
	if grasp_positions_rel == 0:
	    print "Object detection failed. Retry"
	    grasp_positions_rel = get_obj_info()
	    response = ActionServiceResponse(result=5)
	    return response

	if not grasp_positions_rel.grsarr[0].gr:
	    print "No object detected."
	    num_objs = 0
	else:
	    num_objs = len(grasp_positions_rel.grsarr[0].gr)

	object_list = []
	for i in range(0, num_objs):
		if grasp_positions_rel.grsarr[0].gr[i].id == 1:
			object_name = 'red_gotica'
		elif grasp_positions_rel.grsarr[0].gr[i].id == 2:
			object_name = 'bakey'
		elif grasp_positions_rel.grsarr[0].gr[i].id == 3:
			object_name = 'pringles_onion'
		elif grasp_positions_rel.grsarr[0].gr[i].id == 4:
			object_name = 'diget_box'
		elif grasp_positions_rel.grsarr[0].gr[i].id == 5:
			object_name = 'diget'
		elif grasp_positions_rel.grsarr[0].gr[i].id == 6:
			object_name = 'gotica'
		elif grasp_positions_rel.grsarr[0].gr[i].id == 8:
			object_name = 'diget_small_box'
		else:
			sys.exit('ERROR: no object name in DB')
		   
		if object_name != target_name:
			size_x = abs(grasp_positions_rel.objarr[0].pt[i].bb_sc.x)# - 0.03
			size_y = abs(grasp_positions_rel.objarr[0].pt[i].bb_sc.y)
			size_z = abs(grasp_positions_rel.objarr[0].pt[i].bb_sc.z)+ 0.02
			obj_size = [size_x, size_y, size_z]
			obj_pos = [grasp_positions_rel.grsarr[0].gr[i].grasp_cx, grasp_positions_rel.grsarr[0].gr[i].grasp_cy - 0.0, grasp_positions_rel.grsarr[0].gr[i].grasp_cz - 0.02]			
			object_list.append(object_name)
			if object_name == 'diget_box'  or object_name == 'bakey':
			    if obj_size[0] < obj_size[1]:
			        obj_size[0] = 0.03
	 		    else:
    			        obj_size[1] = 0.03		    
			    make_obj2rviz(object_name, obj_pos, obj_size, grasp_positions_rel.objarr[0].pt[i])	
			elif object_name == 'diget_small_box':
			    obj_size[0] = 0.04
			    obj_size[1] = 0.04			    
			    make_obj2rviz(object_name, obj_pos, obj_size, grasp_positions_rel.objarr[0].pt[i])				
			else:		
			    obj_size[0] = 0.03
			    obj_size[1] = 0.03		
			    CLF.add_box_client(object_name, obj_pos, [0.0, 0.0, 0.0, 0.0], obj_size, 'green')
		#else:
			#prerelease_position = compute_pregrasp_poses_direction(release_position, -45.0)
			#prerelease_position[2] = 1.0
        print '*** Object detection done ***', object_list

	print '*** Motion planning environment generation ***'
	if place_name == "table_env":
		get_env_fur(place_name) # env_furniture: 'table_env' or 'display_env' or 'storage_env'
	elif place_name == "display_env":
		get_env_fur(place_name)
	elif place_name == "storage_env":
		get_env_fur(place_name)
	else:
		sys.exit("No such place name") 


        print '*** Compute release location ***'
	release_positions = compute_release_positions_rel(plane_name, [0.85, 2.0], grasp_positions_rel)
	print "total number of candidate:", len(release_positions)
	#for i in range(len(release_positions)):
	#    release_position[i][2] = 0.97

	# prerelease
        #print '*** Pre-release ***'
	#result1 = try_path_plan(prerelease_position, -45)
	#rospy.sleep(0.01)

	# release
        print '*** Release ***'
	result2 = 0
	num_release_positions = len(release_positions)
	i = 0
	while i < num_release_positions and result2 == 0 and i < 2:
	    release_position = release_positions[i]
	    print "try candidate position:", release_position
	    if target_name == 'pringles_onion':
		release_position[2] = release_position[2] + 0.02
	    result2, prev_angle = try_path_plan(release_position, -20)
	    rospy.sleep(0.01)
            #if result2 == 0:
            #    result2, prev_angle = try_path_plan(release_position, -25.0)
            #    rospy.sleep(0.01)
	    i = i + 1

        # open
	if result2 == 1:
            print '*** Gripper open ***'
	    rospy.sleep(0.01)
	    gripper_open('open')
	    rospy.sleep(0.01)

	    if target_name == 'diget_box' or target_name == 'diget_small_box'  or target_name == 'bakey':
		obj_ori = quaternion_from_euler(math.radians(-90.0), math.radians(-90.0) + prev_angle, 0, axes='rxyz')
	        CLF.det_box_client(target_name)
	    else:
	        CLF.det_box_client(target_name)

	# postrelease
        #print '*** Post-release ***'
	#result3 = try_path_plan(prerelease_position, prev_angle)
	#rospy.sleep(0.01)
        ##if result3 == 0:
        #    result3, prev_angle = try_path_plan(prerelease_position, prev_angle - 5.0)
        #    rospy.sleep(0.01)
        #if result3 == 0:
        #    result3, prev_angle = try_path_plan(prerelease_position, prev_angle + 5.0)
       #     rospy.sleep(0.01)
        
	# home
        print '*** Home ***'
	result4 = try_path_plan(home_position, -45)#20)
	rospy.sleep(0.01)


        print '*** Motion planning environment clear ***'
	if place_name == "table_env":
		CLF.del_box_client('counter')
	elif place_name == "display_env":
		del_shelf_env()
	elif place_name == "storage_env":
		del_shelf_env()
	else:
		sys.exit("No such place name")   

	
	#CLF.del_box_client(target_name)
	for object_name in object_list:
		CLF.del_box_client(object_name)

        print '*** Gripper close ***'
	gripper_close('close')
        rospy.sleep(0.01)

	get_r_hand_encoder()
	rospy.sleep(0.3)

	response = ActionServiceResponse(result=0)#success
	if response.result == 0:
	    CLF.det_box_client(target_name)
	    CLF.del_box_client(target_name)
	if encoder_joints.position[-2] > -10 and result4 < 0.01:
            print '!!! FAIL !!! Released but could not come back home'
	    response = ActionServiceResponse(result=1) #fail: released but stop moving
	    CLF.det_box_client(target_name)
	    CLF.del_box_client(target_name)
	elif encoder_joints.position[-2] <= -10 and result4 > 0.99:
            print '!!! FAIL !!! Could not release but came home'
	    response = ActionServiceResponse(result=2) #fail: can't release but finished moving
	elif encoder_joints.position[-2] <= -10 and result4 < 0.01:
            print '!!! FAIL !!! Could not release, could not come back home'
	    response = ActionServiceResponse(result=3) #fail: can't release and stop moving
        if result2 < 0.01 and result4 < 0.01:
            print '!!! FAIL !!! Motion planning fails for all poses'
            response = ActionServiceResponse(result=4)  # fail: motion planning fails for all poses
	return response


def relocate_arm_service(data, timeout=4):
	print 'relocate_arm_service called'#, hand_name, target_name, plane_name, place_name
	hand_name = 'R_hand'

	obstacle_name = data.targetBody[0]
	obstacle_name = obstacle_name.split('#')[1]
	obstacle_name = obstacle_name[:-1]
	
	target_name = data.actionType[0]
	target_name = target_name.split('#')[1]
	target_name = target_name[:-1]
	
	plane_name = data.object[0]
	plane_name = plane_name.split('#')[1]
	plane_name = plane_name[:-1]
	
	place_name = data.targetPose[0]
	place_name = place_name.split('#')[1]
	place_name = place_name[:-1]

	home_position = [0.336, -0.3465, 1.0]

        print '*** Object detection and analysis ***'
	grasp_positions_rel = get_obj_info()  

	if grasp_positions_rel == 0:
	    print "Object detection failed. Retry"
	    grasp_positions_rel = get_obj_info()
	if grasp_positions_rel == 0:
	    print "Object detection failed. Retry"
	    grasp_positions_rel = get_obj_info()
	    response = ActionServiceResponse(result=5)
	    return response
	if not grasp_positions_rel.grsarr[0].gr:
	    print "No object detected."
	    num_objs = 0
	else:
	    num_objs = len(grasp_positions_rel.grsarr[0].gr)

	object_list = []
	for i in range(0, num_objs):
		if grasp_positions_rel.grsarr[0].gr[i].id == 1:
			object_name = 'red_gotica'
		elif grasp_positions_rel.grsarr[0].gr[i].id == 2:
			object_name = 'bakey'
		elif grasp_positions_rel.grsarr[0].gr[i].id == 3:
			object_name = 'pringles_onion'
		elif grasp_positions_rel.grsarr[0].gr[i].id == 4:
			object_name = 'diget_box'
		elif grasp_positions_rel.grsarr[0].gr[i].id == 5:
			object_name = 'diget'
		elif grasp_positions_rel.grsarr[0].gr[i].id == 6:
			object_name = 'gotica'
		elif grasp_positions_rel.grsarr[0].gr[i].id == 8:
			object_name = 'diget_small_box'
		else:
			sys.exit('ERROR: no object name in DB')		   

		size_x = abs(grasp_positions_rel.objarr[0].pt[i].bb_sc.x)# - 0.03
		size_y = abs(grasp_positions_rel.objarr[0].pt[i].bb_sc.y)
		size_z = abs(grasp_positions_rel.objarr[0].pt[i].bb_sc.z)+ 0.02
		obj_size = [size_x, size_y, size_z]
		obj_pos = [grasp_positions_rel.grsarr[0].gr[i].grasp_cx, grasp_positions_rel.grsarr[0].gr[i].grasp_cy - 0.0, grasp_positions_rel.grsarr[0].gr[i].grasp_cz - 0.02]			
		object_list.append(object_name)
		if object_name == 'diget_box' or object_name == 'bakey':
		    if obj_size[0] < obj_size[1]:
		        obj_size[0] = 0.03
 		    else:
		        obj_size[1] = 0.03		    
		    make_obj2rviz(object_name, obj_pos, obj_size, grasp_positions_rel.objarr[0].pt[i])	
		elif object_name == 'diget_small_box':
		    obj_size[0] = 0.04
		    obj_size[1] = 0.04			    
		    make_obj2rviz(object_name, obj_pos, obj_size, grasp_positions_rel.objarr[0].pt[i])				
		else:	
		    obj_size[0] = 0.04
		    obj_size[1] = 0.04			
		    CLF.add_box_client(object_name, obj_pos, [0.0, 0.0, 0.0, 0.0], obj_size, 'green')
		if object_name == target_name:
		    target_pos = obj_pos
        print '*** Object detection done ***', object_list

	print '*** Motion planning environment generation ***'
	if place_name == "table_env":
		get_env_fur(place_name) # env_furniture: 'table_env' or 'display_env' or 'storage_env'
	elif place_name == "display_env":
		get_env_fur(place_name)
	elif place_name == "storage_env":
		get_env_fur(place_name)
	else:
		sys.exit("No such place name") 


        print '*** Compute release location ***'
	release_positions = compute_release_positions_rel(plane_name, [0.85, 2.0], grasp_positions_rel)
	print "total number of candidate:", len(release_positions)

	# release
        print '*** Release ***'
	result2 = 0
	num_release_positions = len(release_positions)
	i = 0
	while i < num_release_positions and result2 == 0 and i < 2:
	    release_position = release_positions[i]
	    print "try candidate position:", release_position
	    if target_name == 'pringles_onion':
		release_position[2] = release_position[2] + 0.02
	    result2, prev_angle = try_path_plan(release_position, -20)
	    rospy.sleep(0.01)
            #if result2 == 0:
            #    result2, prev_angle = try_path_plan(release_position, -25.0)
            #    rospy.sleep(0.01)
	    i = i + 1

        
        # open
	if result2 == 1:
            print '*** Gripper open ***'
	    rospy.sleep(0.01)
	    gripper_open('open')
	    rospy.sleep(0.01)

	    if target_name == 'diget_box' or target_name == 'diget_small_box'  or target_name == 'bakey':
		obj_ori = quaternion_from_euler(math.radians(-90.0), math.radians(-90.0) + prev_angle, 0, axes='rxyz')
	        #CLF.det_box_client(target_name)
		CLF.det_box_client(obstacle_name)
	    else:
	        #CLF.det_box_client(target_name)
		CLF.det_box_client(obstacle_name)

	# postrelease
        #print '*** Post-release ***'
	#result3 = try_path_plan(prerelease_position, prev_angle)
	#rospy.sleep(0.01)
        ##if result3 == 0:
        #    result3, prev_angle = try_path_plan(prerelease_position, prev_angle - 5.0)
        #    rospy.sleep(0.01)
        #if result3 == 0:
        #    result3, prev_angle = try_path_plan(prerelease_position, prev_angle + 5.0)
       #     rospy.sleep(0.01)
        
	# home
        print '*** Home ***'
	result4 = try_path_plan(home_position, -45)#20)
	rospy.sleep(0.01)

	get_r_hand_encoder()
	rospy.sleep(0.3)

        print '*** Motion planning environment clear ***'
	if place_name == "table_env":
		CLF.del_box_client('counter')
	elif place_name == "display_env":
		del_shelf_env()
	elif place_name == "storage_env":
		del_shelf_env()
	else:
		sys.exit("No such place name")   

	
	#CLF.del_box_client(target_name)
	for object_name in object_list:
		CLF.det_box_client(object_name)
		CLF.del_box_client(object_name)

	CLF.del_box_client(obstacle_name)
        print '*** Gripper close ***'
	gripper_close('close')
        rospy.sleep(0.01)

	response = ActionServiceResponse(result=0)#success
	if response.result == 0:
	    CLF.det_box_client(target_name)
	    CLF.del_box_client(target_name)
	if encoder_joints.position[-2] > -10 and result4 < 0.01:
            print '!!! FAIL !!! Released but could not come back home'
	    response = ActionServiceResponse(result=1) #fail: released but stop moving
	    CLF.det_box_client(target_name)
	    CLF.del_box_client(target_name)
	elif encoder_joints.position[-2] <= -10 and result4 > 0.99:
            print '!!! FAIL !!! Could not release but came home'
	    response = ActionServiceResponse(result=2) #fail: can't release but finished moving
	elif encoder_joints.position[-2] <= -10 and result4 < 0.01:
            print '!!! FAIL !!! Could not release, could not come back home'
	    response = ActionServiceResponse(result=3) #fail: can't release and stop moving
        if result2 < 0.01 and result4 < 0.01:
            print '!!! FAIL !!! Motion planning fails for all poses'
            response = ActionServiceResponse(result=4)  # fail: motion planning fails for all poses
	return response


def retrieve_arm_service(data, timeout=4):
	print 'retrieve_arm_service called'#, hand_name, target_name, plane_name, place_name
	hand_name = 'R_hand'

	target_name = data.actionType[0]
	target_name = target_name.split('#')[1]
	target_name = target_name[:-1]
	
	place_name = data.object[0]
	place_name = place_name.split('#')[1]
	place_name = place_name[:-1]

	home_position = [0.336, -0.3465, 1.0]

        print '*** Object detection and analysis ***'

	grasp_positions_rel = get_obj_info()
	if grasp_positions_rel == 0:
	    print "Object detection failed. Retry"
	    grasp_positions_rel = get_obj_info()
	if grasp_positions_rel == 0:
	    print "Object detection failed. Retry"
	    grasp_positions_rel = get_obj_info()
	    num_objs = 0
	    #sys.exit('ERROR: Object detection does not work. Terminate')

	if not grasp_positions_rel.grsarr[0].gr:
	    print "No object detected."
	    num_objs = 0
	else:
	    num_objs = len(grasp_positions_rel.grsarr[0].gr)

	if num_objs > 0:

	    print '*** Plane size calculation *** '
	    w = 0.355
	    d = 0.335 / 2.0
	    phi = math.atan2(-w, d)# back markers
	    diagonal = math.sqrt((w**2 + d**2))
	    marker_1, marker_2 = get_markers()
	    if not marker_1 or not marker_2:
	        print "Failed to get markers. Assign hard-coded values."
	        marker_1 = [0.674191157902, 0.248624965549, 0.893736226085]
	        marker_2 = [1.00831483875, 0.249981209636, 0.902455484087]
	    l = math.sqrt(((marker_1[0]-marker_2[0])**2)+((marker_1[1]-marker_2[1])**2))
	
	    theta_get = math.asin((marker_2[1]-marker_1[1])/(2*d))
	    z_axis = (0.0, 0.0, 1)
	    Rz = ttf.rotation_matrix(theta_get, z_axis) 
	    plane_x = marker_1[0]+(diagonal)*math.cos(theta_get+phi)
	    plane_y = marker_1[1]+(diagonal)*math.sin(theta_get+phi)
	    plane_size = [0.3, 0.5]#depth, width
	    plane_center = [plane_x - 0.03, plane_y - 0.1]
	    x_min = plane_center[0] - plane_size[0]
	    x_max = plane_center[0] + plane_size[0]
	    y_min = plane_center[1] - plane_size[1]
	    y_max = plane_center[1] + plane_size[1]

	    object_list = []
	    all_object_list = []
	    X = []
	    Y = []
	    R = []
	    H = []
	    robot_height = home_position[2]
	    robot_pose = [home_position[0],  home_position[1]]
	    N = copy.deepcopy(num_objs)
	    #target_id = 8 # -9999 if target is undetected
	    for i in range(0, num_objs):
		if grasp_positions_rel.grsarr[0].gr[i].id == 1:
			object_name = 'red_gotica'
		elif grasp_positions_rel.grsarr[0].gr[i].id == 2:
			object_name = 'bakey'
		elif grasp_positions_rel.grsarr[0].gr[i].id == 3:
			object_name = 'pringles_onion'
		elif grasp_positions_rel.grsarr[0].gr[i].id == 4:
			object_name = 'diget_box'
		elif grasp_positions_rel.grsarr[0].gr[i].id == 5:
			object_name = 'diget'
		elif grasp_positions_rel.grsarr[0].gr[i].id == 6:
			object_name = 'gotica'
		elif grasp_positions_rel.grsarr[0].gr[i].id == 8:
			object_name = 'diget_small_box'
		else:
			sys.exit('ERROR: no object name in DB')
		size_x = abs(grasp_positions_rel.objarr[0].pt[i].bb_sc.x) 
		size_y = abs(grasp_positions_rel.objarr[0].pt[i].bb_sc.y)
		size_z = abs(grasp_positions_rel.objarr[0].pt[i].bb_sc.z) + 0.02
		obj_size = [size_x, size_y, size_z]	
		obj_pos = [grasp_positions_rel.grsarr[0].gr[i].grasp_cx, grasp_positions_rel.grsarr[0].gr[i].grasp_cy - 0.0, grasp_positions_rel.grsarr[0].gr[i].grasp_cz - 0.02]
		X.append(obj_pos[0])
		Y.append(obj_pos[1])
		R.append(max(size_x, size_y))
		H.append(size_z)
		if object_name == target_name:
			target_id = i
		all_object_list.append(object_name)
            print '*** Object detection done ***', all_object_list
	
            if target_name not in all_object_list:
                print 'Case 2: target not detected'    
	        target_id = -9999
	    else:
                print 'Cases 0&1: target detected'    
	    print 'Target (ID): ', target_name, target_id

	    [accessibility, relocate_id, relocate_coordinates] = rp.relocate_planner(robot_height, robot_pose, target_id, N, R, H, X, Y, x_min, x_max, y_min, y_max)
	    relocate_object = all_object_list[relocate_id]
	    print('Relocate', relocate_object)
	    #print('Target accessibility (-1=unaccessible, 0=undetected, 1=accessible): %d' % accessibility)
	    #print('Relocate Object %d at (%f, %f)' % (relocate_id, relocate_coordinates[0], relocate_coordinates[1]))
	else:
	    relocate_object = ''
	response = ActionServiceResponse(param=relocate_object) 
	return response


def home_arm_service(data, timeout=4):
    print 'home_arm_service called'
    hand_name = 'R_hand'
    place_name = data.actionType[0]
    place_name = place_name.split('#')[1]
    place_name = place_name[:-1]

    #home_position = [0.1660 + 0.27, -0.2465, 1.0]#0.7629]
    home_position = [0.336, -0.3465, 1.0]
    print '*** Motion planning environment generation ***'
    if place_name == "table_env":
		get_env_fur(place_name)
    elif place_name == "storage_env":
		get_env_fur(place_name)
    elif place_name == "display_env":
		get_env_fur(place_name)
    else:
		sys.exit("No such place name") 

    # home
    print '*** Home ***'
    result1 = try_path_plan(home_position, -45)
    rospy.sleep(0.01)
    
    print '*** Motion planning environment clear ***'
    if place_name == "table_env":
		CLF.del_box_client('counter')
    elif place_name == "display_env":
		del_shelf_env()
    elif place_name == "storage_env":
		del_shelf_env()
    else:
		sys.exit("No such place name")   

    response = ActionServiceResponse(result=0)#success
    if not result1:
        print '!!! FAIL !!! Stopped moving'
        response = ActionServiceResponse(result=1) #fail: stop moving
    return response


def init_base_service(data, timeout=4):
	print 'init_base_service called'
	test_navigation_action(0.0, 0.0, 0.0, 1, 0) # home
	rospy.sleep(0.01)
        
	response = ActionServiceResponse(result=0)      
	return response


def move_base_t2s_service(data, timeout=4):
	print 'move_base_t2s_service called'

	test_navigation_action(-0.3, 0.0, 0.0, 0, 0) # back from table
        rospy.sleep(0.01)
	test_navigation_action(0.0, 0.35, 90.0, 1, 0) # go to storage
	rospy.sleep(0.01)

	response = ActionServiceResponse(result=0)
	return response


def move_base_s2t_service(data, timeout=4):
	print 'move_base_s2t_service called'
	test_navigation_action(-0.7, 0.0, 0.0, 0, 0) # back from storage
        rospy.sleep(0.01)
	test_navigation_action(0.0, -0.2, -90.0, 1, 0) # go to table
	rospy.sleep(0.01)

	response = ActionServiceResponse(result=0)        
	return response


def move_base_t2d_service(data, timeout=4):
	print 'move_base_t2d_service called'

	#test_navigation_action(-0.3, 0.0, 1.0, 0, 0) # back from table
        #rospy.sleep(0.01)
	#test_navigation_action(-0.3, -0.1, 179.0, 1, 0) # go to display
	test_navigation_action(-0.3, 0.0, 0.0, 0, 0) # back from table
        rospy.sleep(0.01)
	test_navigation_action(0.0, 0.0, 90.0, 0, 0) # back from table
	rospy.sleep(0.01)
	test_navigation_action(-0.1, 0.3, 90.0, 1, 0) # go to display
	rospy.sleep(0.01)

	response = ActionServiceResponse(result=0)        
	return response


def move_base_d2t_service(data, timeout=4):
	print 'move_base_d2t_service called'#, base_name, object_name

	#test_navigation_action(-0.3, -0.1, -2.0, 0, 0) # back from display
        #rospy.sleep(0.01)
	#test_navigation_action(0.0, -0.05, 0.0, 0, 0) # back from display
        #rospy.sleep(0.01)
	#test_navigation_action(-0.15, 0.0, -179.0, 1, 0) # go to table

	test_navigation_action(-0.3, -0.1, 0.0, 0, 0) # back from display
	rospy.sleep(0.01)
	test_navigation_action(0.0, 0.0, -90.0, 0, 0) # back from table
	rospy.sleep(0.01)
	test_navigation_action(0.0, -0.15, -90.0, 1, 0) # go to table
	rospy.sleep(0.01)

	response = ActionServiceResponse(result=0)        
	return response


def move_base_d2s_service(data, timeout=4):
	print 'move_base_d2s_service called'

	test_navigation_action(-0.40, 0.0, 0.0, 0, 0) # back from display
	rospy.sleep(0.01)
	test_navigation_action(0.0, -0.35, -90.0, 1, 0) # go to storage
	rospy.sleep(0.01)

	response = ActionServiceResponse(result=0)        
	return response


def move_base_s2d_service(data, timeout=4):
	print 'move_base_s2d_service called'

	test_navigation_action(-0.7, 0.0, 0.0, 0, 0) # back from storage
	rospy.sleep(0.01)
	test_navigation_action(0.0, 0.35, 90.0, 1, 0) # go to display
	rospy.sleep(0.01)

	response = ActionServiceResponse(result=0)        
	return response


def move_base_left_service(data, timeout=4):
	print 'move_base_left_service called'
	test_navigation_action(0.0, 0.2, 0.0, 0, 0)
	rospy.sleep(0.01)

	response = ActionServiceResponse(result=0)
	return response


def move_base_right_service(data, timeout=4):
	print 'move_base_right_service called'
	test_navigation_action(0.0, -0.2, 0.0, 0, 0)
	rospy.sleep(0.01)

	response = ActionServiceResponse(result=0)
	return response


def move_base_back_service(data, timeout=4):
	print 'move_base_back_service called'
	test_navigation_action(-0.08, 0.0, 0.0, 0, 0)
	rospy.sleep(0.01)

	response = ActionServiceResponse(result=0)
	return response


def move_base_forward_service(data, timeout=4):
	print 'move_base_forward_service called'
	test_navigation_action(0.08, 0.0, 0.0, 0, 0)
	rospy.sleep(0.1)

	response = ActionServiceResponse(result=0)
	return response


def move_base_align_service(data, timeout=4):
	print 'move_base_align_service called'
	test_navigation_action(0.0, 0.0, 0.0, 1, 0)
	rospy.sleep(0.01)

	response = ActionServiceResponse(result=0)
	return response


def move_base_origin_service(data, timeout=4):
	print 'move_base_origin_service called'
        test_navigation_action(-0.3, -0.1, 0.0, 0, 1)
        #rospy.sleep(0.01)
        test_navigation_action(0.0, 0.0, 0.0, 1, 1)
        rospy.sleep(0.01)

	response = ActionServiceResponse(result=0)
	return response


def close_gripper_service(data, timeout=4):
	rospy.sleep(0.01)
	gripper_close('close')
	rospy.sleep(0.01)
	response = ActionServiceResponse(result=0)
	return response


def open_gripper_service(data, timeout=4):
	rospy.sleep(0.01)
	gripper_open('open')
	rospy.sleep(0.01)
	response = ActionServiceResponse(result=0)
	return response



def listener():
    
    # =================== service!! =======================
    rospy.Service('execute_arm_grasp', ActionService, grasp_arm_service)
    rospy.Service('execute_arm_release', ActionService, release_arm_service)
    rospy.Service('execute_arm_relocate', ActionService, relocate_arm_service)
    rospy.Service('execute_arm_retrieve', ActionService, retrieve_arm_service)
    rospy.Service('execute_arm_home', ActionService, home_arm_service)

    rospy.Service('close_gripper', ActionService, close_gripper_service)
    rospy.Service('open_gripper', ActionService, open_gripper_service)


    rospy.Service('init_base', ActionService, init_base_service)
    rospy.Service('execute_base_t2s', ActionService, move_base_t2s_service)
    rospy.Service('execute_base_s2t', ActionService, move_base_s2t_service)
    rospy.Service('execute_base_t2d', ActionService, move_base_t2d_service)
    rospy.Service('execute_base_d2t', ActionService, move_base_d2t_service)
    rospy.Service('execute_base_d2s', ActionService, move_base_d2s_service)
    rospy.Service('execute_base_s2d', ActionService, move_base_s2d_service)

    rospy.Service('execute_base_left', ActionService, move_base_left_service)
    rospy.Service('execute_base_right', ActionService, move_base_right_service)
    rospy.Service('execute_base_back', ActionService, move_base_back_service)
    rospy.Service('execute_base_forward', ActionService, move_base_forward_service)
    rospy.Service('execute_base_align', ActionService, move_base_align_service)
    rospy.Service('execute_base_origin', ActionService, move_base_origin_service)


    rospy.spin()


if __name__ == '__main__':
    ### Initialize variables ###
    global encoder_joints

    print "========TMI: Hello, human.========"
    rospy.init_node('tmi_node', anonymous=True)

    listener()
    print "========TMI: Good bye.========"

