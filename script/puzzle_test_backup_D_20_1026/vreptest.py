#!/usr/bin/env python
GRID_SIZE = 0.01
G2P_SIZE = 100
try:
    import vrep
except:
    print '--------------------------------------------------------------'
    print '"vrep.py" could not be imported. This means very probably that'
    print 'either "vrep.py" or the remoteApi library could not be found.'
    print 'Make sure both are in the same folder as this file,'
    print 'or appropriately adjust the file "vrep.py"'
    print '--------------------------------------------------------------'
    print ''

import numpy as np
import copy
import time
import D_0902_custom_function as CUF
import D_0902_client_function as CLF

from D_0902_envClass4altest import EnvInfo as EI
from D_0902_envClass4altest import CanInfo as CI

import timeit

class vrep_env_jaco:
    def __init__(self):
        print ('Program started')
        vrep.simxFinish(-1)  # just in case, close all opened connections
        self.clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # Connect to V-REP
        returnCode, self.robot_base_world = vrep.simxGetObjectHandle(self.clientID, 'jaco_base_world', vrep.simx_opmode_blocking)
        # returnCode, self.robot_base_world = vrep.simxGetObjectHandle(self.clientID, 'hubo_base_world', vrep.simx_opmode_blocking)
        self.open_hand(0.1)
        if self.clientID!=-1:
            print"let's start vrep environment for jaco"

    def get_object_info(self, obj_name):
        # object info = position, orientation, scale
        print"find", obj_name, "in vrep scene"
        returnCode, object = vrep.simxGetObjectHandle(self.clientID, obj_name, vrep.simx_opmode_blocking)
        returnCode, obj_pos = vrep.simxGetObjectPosition(self.clientID, object, self.robot_base_world, vrep.simx_opmode_oneshot_wait)
        returnCode, obj_ori_q = vrep.simxGetObjectQuaternion(self.clientID, object, self.robot_base_world, vrep.simx_opmode_oneshot_wait)
        returnCode, bb_min_x = vrep.simxGetObjectFloatParameter(self.clientID, object, 15, vrep.simx_opmode_oneshot_wait)
        returnCode, bb_min_y = vrep.simxGetObjectFloatParameter(self.clientID, object, 16, vrep.simx_opmode_oneshot_wait)
        returnCode, bb_min_z = vrep.simxGetObjectFloatParameter(self.clientID, object, 17, vrep.simx_opmode_oneshot_wait)
        returnCode, bb_max_x = vrep.simxGetObjectFloatParameter(self.clientID, object, 18, vrep.simx_opmode_oneshot_wait)
        returnCode, bb_max_y = vrep.simxGetObjectFloatParameter(self.clientID, object, 19, vrep.simx_opmode_oneshot_wait)
        returnCode, bb_max_z = vrep.simxGetObjectFloatParameter(self.clientID, object, 20, vrep.simx_opmode_oneshot_wait)
        obj_sx = bb_max_x - bb_min_x
        obj_sy = bb_max_y - bb_min_y
        obj_sz = bb_max_z - bb_min_z
        obj_scale = [obj_sx, obj_sy, obj_sz]
        return obj_pos, obj_ori_q, obj_scale

    def get_current_joint(self, joint_names):
        print"get current joints"
        self.joint_handle = []
        self.cur_joint_pos = []
        for i in range(len(joint_names)):
            returnCode, jh = vrep.simxGetObjectHandle(self.clientID, joint_names[i], vrep.simx_opmode_oneshot_wait)
            self.joint_handle.append(jh)
            returnCode, jp = vrep.simxGetJointPosition(self.clientID, self.joint_handle[i], vrep.simx_opmode_oneshot_wait)
            self.cur_joint_pos.append(jp)
        return self.cur_joint_pos

    def set_current_joint(self, joint_names):
        print"set current joints"
        self.joint_handle = []
        self.cur_joint_pos = []
        for i in range(len(joint_names)):
            returnCode, jh = vrep.simxGetObjectHandle(self.clientID, joint_names[i], vrep.simx_opmode_oneshot_wait)
            self.joint_handle.append(jh)
            returnCode, jp = vrep.simxGetJointPosition(self.clientID, self.joint_handle[i], vrep.simx_opmode_oneshot_wait)
            self.cur_joint_pos.append(jp)

        ret = CLF.move_joints_client_rad('arm', self.cur_joint_pos)
        return ret

    def close_hand(self, wait_time):
        print"close hand"
        returnCode = vrep.simxSetIntegerSignal(self.clientID, "hand", (1), vrep.simx_opmode_oneshot_wait)
        time.sleep(wait_time)

    def open_hand(self, wait_time):
        print"open hand"
        returnCode = vrep.simxSetIntegerSignal(self.clientID, "hand", (0), vrep.simx_opmode_oneshot_wait)
        time.sleep(wait_time)



# if __name__ == "__main__":
#     vrep_env = vrep_env_jaco()
#     for i in range(5):
#         vrep_env.close_hand(1)
#         vrep_env.open_hand(1)



if __name__ == "__main__":

    vrep_env = vrep_env_jaco()
    joint_names_jaco = ['Jaco_joint1', 'Jaco_joint2', 'Jaco_joint3', 'Jaco_joint4', 'Jaco_joint5', 'Jaco_joint6']
    vrep_env.set_current_joint(joint_names_jaco)
    CLF.move_hand_joint_client([0.35, 0.35, 0.35])

    # object_info = [[obj_pos], [obj_ori_q], [obj_scale]]
    target_name = ['target']
    target_info = []
    for i in range(len(target_name)):
        target_info.append(vrep_env.get_object_info(target_name[i]))

    obstacle_name = ['obstacle0', 'obstacle1', 'obstacle2', 'obstacle3', 'obstacle4', 'obstacle5', 'obstacle6']
    obstacle_info = []
    for i in range(len(obstacle_name)):
        obstacle_info.append(vrep_env.get_object_info(obstacle_name[i]))

    env_name = ['customizableTable_forwarder', 'Jaco_base', 'table_ls', 'table_rs', 'table_us', 'table_bs']
    env_info = []
    for i in range(len(env_name)):
        env_info.append(vrep_env.get_object_info(env_name[i]))

    for i in range(len(obstacle_info)):
        CLF.add_box_client(obstacle_name[i], obstacle_info[i][0], obstacle_info[i][1], obstacle_info[i][2], 'red')
    for i in range(len(target_info)):
        CLF.add_box_client(target_name[i], target_info[i][0], target_info[i][1], target_info[i][2], 'green')
    for i in range(len(env_info)):
        CLF.add_box_client(env_name[i], env_info[i][0], env_info[i][1], env_info[i][2], 'gray')
