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

import rospy
import numpy as np
import tf
import matplotlib.pyplot as plt
import copy
import time
import D_0902_custom_function as CUF
import D_0902_client_function as CLF

from VFHplus_change_radius import influence

from D_0902_envClass4altest import EnvInfo as EI
from D_0902_envClass4altest import CanInfo as CI

from arm_move.srv._box_info_srv import *
from arm_move.srv._arm_move_srv import *
from arm_move.srv._work_start_srv import *
from arm_move.srv._att_hand_box_srv import *
from arm_move.srv._arm_goalJoint_srv import *


class vrep_env_jaco:
    def __init__(self):
        print ('Program started')
        vrep.simxFinish(-1)  # just in case, close all opened connections
        self.clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # Connect to V-REP
        returnCode, self.robot_base_world = vrep.simxGetObjectHandle(self.clientID, 'jaco_base_world', vrep.simx_opmode_blocking)
        # returnCode, self.robot_base_world = vrep.simxGetObjectHandle(self.clientID, 'hubo_base_world', vrep.simx_opmode_blocking)

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
            # print i, joint_names[i], len(joint_names)
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
            # print i, joint_names[i], len(joint_names)
            returnCode, jh = vrep.simxGetObjectHandle(self.clientID, joint_names[i], vrep.simx_opmode_oneshot_wait)
            self.joint_handle.append(jh)
            returnCode, jp = vrep.simxGetJointPosition(self.clientID, self.joint_handle[i], vrep.simx_opmode_oneshot_wait)
            self.cur_joint_pos.append(jp)

        ret = CLF.move_joints_client_rad('arm', self.cur_joint_pos)
        return ret



if __name__ == "__main__":

    # CLF.move_joints_client_deg('arm', [180, 260, 340, -35, 100, -30])
    vrep_env = vrep_env_jaco()
    '''
    for jaco, joint names are Jaco_joint1, Jaco_joint2, ...
    '''
    joint_names_jaco = ['Jaco_joint1', 'Jaco_joint2', 'Jaco_joint3', 'Jaco_joint4', 'Jaco_joint5', 'Jaco_joint6']
    joint_names_hubo_r = ['RF1', 'Jaco_joint2', 'Jaco_joint3', 'Jaco_joint4', 'Jaco_joint5', 'Jaco_joint6']
    joint_names_hubo_l = ['LF1', 'Jaco_joint2', 'Jaco_joint3', 'Jaco_joint4', 'Jaco_joint5', 'Jaco_joint6']
    vrep_env.set_current_joint(joint_names_jaco)
    CLF.move_hand_joint_client([0.35, 0.35, 0.35])

    target_name = ['target']
    target_info = []
    for i in range(len(target_name)):
        target_info.append(vrep_env.get_object_info(target_name[i]))

    obstacle_name = ['obstacle0', 'obstacle1', 'obstacle2', 'obstacle3', 'obstacle4', 'obstacle5', 'obstacle6', 'obstacle7', 'obstacle8']
    obstacle_info = []
    # [[obj_pos.x, obj_pos.y, obj_pos.z], [obj_ori_q.x, obj_ori_q.y, obj_ori_q.z, obj_ori_q.w], [obj_scale.x, obj_scale.y, obj_scale.z]]
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

    ws = env_info[0]
    print"ws info", env_info[0]
    ws_d, ws_w = int(round(ws[2][1]*100)), int(round(ws[2][0]*100))
    print"work space width, depth", ws_w, ws_d
    # GRID_SIZE = 0.01
    ws_zero_pos = [round(ws[0][2] - ws[2][0]*0.5, 2), round(-ws[0][1] - ws[2][1]*0.5, 2)]
    print "ws cen pos", ws[0][2], ws[0][1]
    print "ws, zero pos", ws_zero_pos

    # ws_w, ws_d = 100, 100  # get table size in the v-rep
    ws_cen = [ws[0][2], ws[0][1]]
    rob_pos = [0.0, 0.0]
    OBJ_R = 0.035

    env = EI(rob_pos, ws_w, ws_d, ws_cen, grid_size=GRID_SIZE, wall_r=OBJ_R)
    env.set_env(obstacle_name, obstacle_info, target_name, target_info)

    print "rearrangement order:", env.ore_order

    CUF.draw_grid_info(env.grid_ori)

    plt.show()

    while env.order_error_flag:
        # env.get_env(obs_r, tar_r, min_ore)
        # algorithm_start = timeit.default_timer()
        env.get_max_can(env.grid_ori, bt_num=1, trial_num=1000)  # We get "grid_max_can", "can_grid"
        # env.get_env_case1()
        # env.get_max_can_case1()

        '''
        Make object info!
        Type : target, obstacle, candidate
        Info : pos, grid, A, BT, b, ORC, ORE
        '''
        can_info = []
        for i in range(len(env.can_pos)):
            can_info.append((CI('candidate', env.can_pos[i], env.can_grid[i])))

        # check env info got right
        # if 1:
        #     print "\n# of obstacles", len(env.obs_pos), "\n# of candidates", len(env.can_pos)

        '''
        GET candidates info
        '''
        t_ore_order = copy.deepcopy(env.ore_order)
        # for i in range(len(can_info)):
        #     print "can", i, ":", can_info[i].pos

        CUF.draw_grid_info(env.grid_ori)
        CUF.draw_grid_info(env.grid_del)
        CUF.draw_grid_info(env.grid_max_can)
        for c_i in range(len(can_info)):
            plt.text(can_info[c_i].grid[0], can_info[c_i].grid[1], 'Can' + str(c_i), fontsize=20, ha='center', bbox=dict(facecolor='pink', alpha=0.8))
        for o_i in range(len(env.obs_grid)):
            plt.text(env.obs_grid[o_i][0], env.obs_grid[o_i][1], 'Obs' + str(o_i), fontsize=20, ha='center', bbox=dict(facecolor='red', alpha=0.8))

        plt.show()

        end_t_cp_flag = 0
        while len(env.ore_order):  # this while loop is for the algorithm
            # algorithm will go on until it can access to target

            # Check C.A : just next step
            t_can_info = []
            in_can_info = copy.deepcopy(can_info)
            in_obs_pos = copy.deepcopy(env.obs_pos)
            in_obs_pos.remove(env.obs_pos[env.ore_order[0]])
            CLF.del_box_client(obstacle_name[env.ore_order[0]])
            t_can_info.append(env.get_can_A(in_can_info, in_obs_pos, env.tar_pos))
            CLF.add_box_client(obstacle_name[env.ore_order[0]], obstacle_info[env.ore_order[0]][0], obstacle_info[env.ore_order[0]][1], obstacle_info[env.ore_order[0]][2], 'red')

            # Check C.BT
            in_can_info = copy.deepcopy(t_can_info[0])
            for ore_i in range(len(env.ore_order)):     # after rearrange all ORE
                CLF.del_box_client(obstacle_name[env.ore_order[ore_i]])
            t_can_info[0] = env.get_can_BT(in_can_info, env.tar_pos)
            for ore_i in range(len(env.ore_order)):
                CLF.add_box_client(obstacle_name[env.ore_order[ore_i]], obstacle_info[env.ore_order[ore_i]][0], obstacle_info[env.ore_order[ore_i]][1], obstacle_info[env.ore_order[ore_i]][2], 'red')


            # Check C.BO : BO : other ORE, just before target
            in_can_info = copy.deepcopy(t_can_info[0])
            for ore_i in range(len(env.ore_order)):     # after rearrange all ORE
                CLF.del_box_client(obstacle_name[env.ore_order[ore_i]])
            for j in range(len(env.ore_order)):     # check other ORE just before target
                if j > i:
                    t_can_info[i] = env.get_can_BT(in_can_info, env.obs_pos[env.ore_order[j]])
            for ore_i in range(len(env.ore_order)):
                CLF.add_box_client(obstacle_name[env.ore_order[ore_i]], obstacle_info[env.ore_order[ore_i]][0], obstacle_info[env.ore_order[ore_i]][1], obstacle_info[env.ore_order[ore_i]][2], 'red')

            t_cf = []
            t_cf_index = []
            for i in range(1):
                in_can_info = copy.deepcopy(t_can_info[i])
                ret_can, ret_index = env.get_cf(in_can_info)
                t_cf.append(ret_can)
                t_cf_index.append(ret_index)
                # print "\n step", i, " has # of cf pos:", len(t_cf[i]), "index", t_cf_index[i]

            # See the feasibile candidate
            for i in range(len(t_cf[0])):
                print "\n Our Cf pos:", i, t_cf[0][i].pos
            # See if this case if case0 or case1
            print "t_cf:", t_cf, "order", env.ore_order
