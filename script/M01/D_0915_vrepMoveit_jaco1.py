#!/usr/bin/env python
GRID_SIZE = 0.01
G2P_SIZE = 100

import rospy
import numpy as np
import tf
import matplotlib.pyplot as plt
import copy
import time
import D_0902_custom_function as CUF
import D_0902_client_function as CLF

from D_0909_envClass4altest import EnvInfo as EI
from D_0909_envClass4altest import CanInfo as CI

import timeit

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

    def close_hand(self, wait_time):
        print"close hand"
        returnCode = vrep.simxSetIntegerSignal(self.clientID, "hand", (1), vrep.simx_opmode_oneshot_wait)
        time.sleep(wait_time)

    def open_hand(self, wait_time):
        print"open hand"
        returnCode = vrep.simxSetIntegerSignal(self.clientID, "hand", (0), vrep.simx_opmode_oneshot_wait)
        time.sleep(wait_time)

    def pick_and_place(self, pick_pose, pick_object_name, place_pose):
        env.pre_pick(env.obs_pos, pick_pose, place_pose)
        ret_pick_pose = env.pick(env.obs_pos, pick_pose, place_pose)
        env.move_to([[ret_pick_pose[0][0] - 0.03, ret_pick_pose[0][1], ret_pick_pose[0][2]], ret_pick_pose[1]])
        time.sleep(0.5)
        vrep_env.close_hand(0.5)
        CLF.att_box_client('gripper', pick_object_name)

        env.move_to([[ret_pick_pose[0][0] + 0.05, ret_pick_pose[0][1], ret_pick_pose[0][2]], ret_pick_pose[1]])
        env.back_home(vrep_env.get_current_joint(joint_names_jaco))
        # CLF.car(0.05)

        env.pre_place(env.obs_pos, place_pose, vrep_env.get_current_joint(joint_names_jaco))
        ret_place_pose = env.place(env.obs_pos, place_pose, vrep_env.get_current_joint(joint_names_jaco))
        env.move_to([[ret_place_pose[0][0] - 0.1, ret_place_pose[0][1], ret_place_pose[0][2]], ret_place_pose[1]])
        time.sleep(0.5)
        vrep_env.open_hand(0.5)
        CLF.det_box_client(pick_object_name, [env.object_z, -sel_can_pos[1], sel_can_pos[0]], obstacle_info[env.ore_order[0]][1], obstacle_info[env.ore_order[0]][2], 'blue')

        env.move_to([[ret_place_pose[0][0] + 0.1, ret_place_pose[0][1], ret_place_pose[0][2]], ret_place_pose[1]])
        env.back_home(vrep_env.get_current_joint(joint_names_jaco))
        # CLF.add_box_client(obstacle_name[env.ore_order[0]], [env.object_z, -sel_can_pos[1], sel_can_pos[0]], obstacle_info[env.ore_order[0]][1], obstacle_info[env.ore_order[0]][2], 'blue')


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
    vrep_env.open_hand(0.5)
    CLF.move_hand_joint_client([0.4, 0.4, 0.4])

    target_name = ['target']
    target_info = []
    for i in range(len(target_name)):
        target_info.append(vrep_env.get_object_info(target_name[i]))
        target_info[i][0][2] = target_info[i][0][2] + 0.04
        target_info[i][2][2] = target_info[i][2][2] + 0.08

    obstacle_name = ['obstacle0', 'obstacle1', 'obstacle2', 'obstacle3', 'obstacle4', 'obstacle5', 'obstacle6', 'obstacle7', 'obstacle8',
                     'obstacle9', 'obstacle10', 'obstacle11', 'obstacle12', 'obstacle13', 'obstacle14',]
    # obstacle_name = ['obstacle0', 'obstacle1', 'obstacle2', 'obstacle3', 'obstacle4', 'obstacle5', 'obstacle6', 'obstacle7', 'obstacle8']
    obstacle_info = []
    # [[obj_pos.x, obj_pos.y, obj_pos.z], [obj_ori_q.x, obj_ori_q.y, obj_ori_q.z, obj_ori_q.w], [obj_scale.x, obj_scale.y, obj_scale.z]]
    for i in range(len(obstacle_name)):
        obstacle_info.append(vrep_env.get_object_info(obstacle_name[i]))
        obstacle_info[i][0][2] = obstacle_info[i][0][2] + 0.04
        obstacle_info[i][2][2] = obstacle_info[i][2][2] + 0.08

    env_name = ['customizableTable_forwarder', 'Jaco_base', 'table_ls', 'table_rs', 'table_us', 'table_bs']
    env_info = []
    for i in range(len(env_name)):
        env_info.append(vrep_env.get_object_info(env_name[i]))
        if i > 1:
            env_info[i][2][0] = env_info[i][2][0]+0.01
            env_info[i][2][1] = env_info[i][2][1]+0.01
            env_info[i][2][2] = env_info[i][2][2]+0.01

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

    env.update_env(env.obs_pos, env.obs_grid)
    print "rearrangement order:", env.ore_order
    if len(env.ore_order) == 0:
        print "end rearrangement"
        vrep_env.pick_and_place(env.tar_pos, 'target', env.tar_pos)
        time.sleep(1)
    # CUF.draw_grid_info(env.grid_ori)
    #
    # plt.show()

    space_err = 0
    rearr_cnt = 0

    # env.get_env(obs_r, tar_r, min_ore)
    algorithm_start = timeit.default_timer()
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
    method = 'mine'
    method = 'far'
    method = 'deep'

    while len(env.ore_order):  # this while loop is for the algorithm
        # algorithm will go on until it can access to target
        #
        env.ore_order = []
        CUF.draw_grid_info(env.grid_max_can)
        plt.show()
        print "OR:", env.ore_order
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
        in_can_info = env.init_BT(in_can_info)  # init the BT value of candidates to '0'
        in_obs_pos = copy.deepcopy(env.obs_pos)
        for ore_i in range(len(env.ore_order)):     # after rearrange all ORE
            in_obs_pos.remove(env.obs_pos[env.ore_order[ore_i]])
            CLF.del_box_client(obstacle_name[env.ore_order[ore_i]])
        t_can_info[0] = env.get_can_BT(in_can_info, in_obs_pos, env.tar_pos)
        for ore_i in range(len(env.ore_order)):
            CLF.add_box_client(obstacle_name[env.ore_order[ore_i]], obstacle_info[env.ore_order[ore_i]][0], obstacle_info[env.ore_order[ore_i]][1], obstacle_info[env.ore_order[ore_i]][2], 'red')

        # Check C.BO : BO : other ORE, just before target

        in_can_info = copy.deepcopy(t_can_info[0])
        in_obs_pos = copy.deepcopy(env.obs_pos)
        for ore_i in range(len(env.ore_order)):     # after rearrange all ORE
            in_obs_pos.remove(env.obs_pos[env.ore_order[ore_i]])
            CLF.del_box_client(obstacle_name[env.ore_order[ore_i]])
        for j in range(len(env.ore_order)):     # check other ORE just before target
            if j > i:
                t_can_info[0] = env.get_can_BT(in_can_info, in_obs_pos,env.obs_pos[env.ore_order[j]])
        for ore_i in range(len(env.ore_order)):
            CLF.add_box_client(obstacle_name[env.ore_order[ore_i]], obstacle_info[env.ore_order[ore_i]][0], obstacle_info[env.ore_order[ore_i]][1], obstacle_info[env.ore_order[ore_i]][2], 'red')

        s_v = []
        s_v_index = []
        for i in range(1):
            in_can_info = copy.deepcopy(t_can_info[i])
            ret_can, ret_index = env.get_cf(in_can_info)
            s_v.append(ret_can)
            s_v_index.append(ret_index)
            # print "\n step", i, " has # of cf pos:", len(t_cf[i]), "index", t_cf_index[i]
        print "s_v:", len(s_v[0]), "\ns_v_index:", len(s_v_index[0])
        for i in range(len(s_v[0])):
            print "s_v index:", [i], s_v_index[0][i]
        # See the feasibile candidate
        # for i in range(len(t_cf[0])):
        #     print "\n Our Cf pos:", i, t_cf[0][i].pos
        # See if this case if case0 or case1
        # print "t_cf:", t_cf, "order", env.ore_order

        if len(s_v[0]) >= len(env.ore_order):
            print "enough candidate spots"
            t_b = []
            for i in range(1):
                in_obs_pos = copy.deepcopy(env.obs_pos)
                for ore_i in range(i + 1):
                    in_obs_pos.remove(env.obs_pos[env.ore_order[ore_i]])
                t_b.append(env.get_cf_b(s_v[i], in_obs_pos))
                # print "\n step", i, " has cf b:", t_b[i]

                # draw_figs = 1
                # if draw_figs == 1:
                #     for c_i in range(len(can_info)):
                #         plt.text(can_info[c_i].grid[0], can_info[c_i].grid[1], 'Can' + str(c_i), fontsize=20, ha='center', bbox=dict(facecolor='pink', alpha=0.8))
                #     for o_i in range(len(env.obs_grid)):
                #         plt.text(env.obs_grid[o_i][0], env.obs_grid[o_i][1], 'Obs' + str(o_i), fontsize=20, ha='center', bbox=dict(facecolor='red', alpha=0.8))
                #
                #     for step_i in range(1):
                #         step_grid = copy.deepcopy(env.grid_act)
                #         step_obs_grid = copy.deepcopy(env.obs_grid)
                #         for ore_i in range(step_i + 1):
                #             step_obs_grid.remove(env.obs_grid[env.ore_order[ore_i]])
                #         for i in range(len(step_obs_grid)):
                #             step_grid = CUF.obstacle_circle(step_grid, [round(step_obs_grid[i][0], 2), round(step_obs_grid[i][1], 2), env.obs_r[i]], 2)
                #         for ci in range(len(can_info)):
                #             xi, yi = can_info[ci].grid
                #             step_grid = CUF.obstacle_circle(step_grid, [xi, yi, 0.04], 30)
                #
                #         step_grid = CUF.obstacle_circle(step_grid, [env.tar_grid[0], env.tar_grid[1], tar_r], 4)  # target
                #
                #         for cf_i in range(len(t_b[step_i])):
                #             xi = (t_cf[step_i][cf_i].pos[0] - env.ws_zero[0]) * G2P_SIZE
                #             yi = (t_cf[step_i][cf_i].pos[1] - env.ws_zero[1]) * G2P_SIZE
                #             step_grid = CUF.obstacle_circle(step_grid, [xi, yi, 0.04], 3)
                #
                #         CUF.draw_grid_info(step_grid)
                #
                #         for cf_i in range(len(t_b[step_i])):
                #             xi = (t_cf[step_i][cf_i].pos[0] - env.ws_zero[0]) * G2P_SIZE
                #             yi = (t_cf[step_i][cf_i].pos[1] - env.ws_zero[1]) * G2P_SIZE
                #             plt.text(xi, yi, 'b=' + str(t_b[step_i][cf_i]), fontsize=20, ha='center', bbox=dict(facecolor='pink', alpha=0.8))
                #         for ci in range(len(t_can_info[step_i])):
                #             plt.text(t_can_info[step_i][ci].grid[0], t_can_info[step_i][ci].grid[1] - 2.0, '[A, BT] :' + str([t_can_info[step_i][ci].A, t_can_info[step_i][ci].BT]), fontsize=10, ha='center', bbox=dict(facecolor='pink', alpha=0.8))
                #         for o_i in range(len(env.obs_grid)):
                #             plt.text(env.obs_grid[o_i][0], env.obs_grid[o_i][1], 'Obs' + str(o_i), fontsize=20, ha='center', bbox=dict(facecolor='red', alpha=0.8))
                #         plt.title('step' + str(step_i) + " obs: " + str(env.ore_order[step_i]) + " rearranged")
        elif len(s_v[0]) < len(env.ore_order):
            print "not enough candidate spots"
            # print "Since we meet condition: N(CF) < N(ORE) by", len(t_cf[0]), "<", len(env.ore_order), ",\nwe have to remove additional obstacles."
            ## step1 : "get t_cp", check candidates which have A = 0 and BT = 0
            ## This means that a candidate is not reachable and it does not block the target object

            # Check A for this environment state
            in_can_info = copy.deepcopy(can_info)
            in_obs_pos = copy.deepcopy(env.obs_pos)
            t_can_add = copy.deepcopy(env.get_can_A(in_can_info, in_obs_pos, env.tar_pos))

            s_e = []  # s_e: extra candidate spots

            in_can_info = copy.deepcopy(t_can_add)

            ret_can, ret_index = env.get_cp(in_can_info)
            print "# of OR'", len(ret_can)

            t_s_e = ret_can
            t_s_e_index = ret_index
            # print "t_cp:", len(t_cp), "index", t_cp_index
            # for i in range(len(t_cp)):
            #     print "\n Our Cp:", i, t_cp[i].pos

            if len(t_s_e) == 0:
                print "no possible extra candidate exist"
                space_err = 1
                break
            # step2 : check c_ore for each cp and pick min of it
            t_s_r = []  # s_r: candidate spot relocate plan
            in_can_info = copy.deepcopy(t_s_e)
            # tmp_order_time_start = timeit.default_timer()
            # tmp_order_time_start2 = time.clock()
            t_s_r = env.get_c_ore(in_can_info)
            # tmp_order_time_end = timeit.default_timer()
            # tmp_order_time_end2 = time.clock()
            # order_time = order_time + tmp_order_time_end - tmp_order_time_start
            # order_time2 = order_time2 + tmp_order_time_end2 - tmp_order_time_start2
            # order_cnt = order_cnt + 100 * len(t_s_e)
            # print "\n"
            # for i in range(len(t_cp)):
            #     print "cp", t_cp[i].pos, "\nc_ore", c_ore[i]
            s_r = []
            s_e_index = []
            print "\n"
            for i in range(len(t_s_e)):
                print "can", t_s_e_index[i], "grid:", t_s_e[i].grid, ", s_r:", t_s_r[i]

            for i in range(len(t_s_e)):
                if t_s_r[i] != []:
                    s_e.append(t_s_e[i])
                    s_r.append(t_s_r[i])
                    s_e_index.append(t_s_e_index[i])

            # tmp_se = copy.deepcopy(s_e)
            # tmp_sr = copy.deepcopy(s_r)
            # emp_sr = []
            # for i in range(len(s_e)):
            #     if s_r[i] == []:
            #         print "remove empty s_e", i
            #         emp_sr.append(i)
            #
            # print "tmp se:", tmp_se, "\ntmp sr", tmp_sr
            # for i in range(len(emp_sr)):
            #
            #     print "tmp_se[emp_sr[i]]", tmp_se[emp_sr[i]].pos
            #     print "tmp_sr[emp_sr[i]]", tmp_sr[emp_sr[i]]
            #     s_e.remove(tmp_se[emp_sr[i]])
            #     s_r.remove(tmp_sr[emp_sr[i]])

            while len(s_e):
                print "# of s_e:", len(s_e), s_r
                print "\n"
                for i in range(len(s_e)):
                    print "can", s_e_index[i], "pos:", s_e[i].pos, ", s_r:", s_r[i]
                min_s_r = CUF.min_len_list(s_r)

                print "\nmin sr:", min_s_r
                #
                # print "picked ci index:", t_cp.index(t_cp[c_ore.index(min_c_ore)])
                # print "picked ci address:", copy.deepcopy(t_cp[c_ore.index(min_c_ore)]).pos
                cp = copy.deepcopy(s_e[s_r.index(min_s_r)])
                # print "selected cp pos", cp.pos

                ## step3 : "get t_cf", check candidates which have A = 1 and BT' = 0
                ## Check A for this environment state T' is t_cp_i
                in_can_info = copy.deepcopy(can_info)
                in_obs_pos = copy.deepcopy(env.obs_pos)
                in_tar_pos = copy.deepcopy(cp.pos)
                t_can_add = copy.deepcopy(env.get_can_A(in_can_info, in_obs_pos, env.tar_pos))

                # Check C.BT for this environment state
                in_can_info = copy.deepcopy(t_can_add)
                in_can_info = env.init_BT(in_can_info)  # init the BT value of candidates to '0'
                in_obs_pos = copy.deepcopy(env.obs_pos)

                sorted_min_s_r = copy.deepcopy(min_s_r)
                sorted_min_s_r.sort(reverse=True)
                print "sorted min_s_r:", sorted_min_s_r

                if sorted_min_s_r[0] == len(env.obs_pos):  # if OR' has o_t ! remove s_e
                    print "o_t is in OR'"
                    s_e.remove(s_e[s_r.index(min_s_r)])
                    s_e_index.remove(s_e_index[s_r.index(min_s_r)])
                    s_r.remove(s_r[s_r.index(min_s_r)])
                else:
                    for ore_i in range(len(min_s_r)):  # after rearrange all OR'
                        in_obs_pos.remove(in_obs_pos[sorted_min_s_r[ore_i]])
                        CLF.del_box_client(obstacle_name[sorted_min_s_r[ore_i]])
                    in_tar_pos = copy.deepcopy(cp.pos)
                    t_can_add = env.get_can_BT(in_can_info, in_obs_pos, in_tar_pos)

                    for ore_i in range(len(min_s_r)):  # after rearrange all OR'
                        CLF.add_box_client(obstacle_name[sorted_min_s_r[ore_i]], obstacle_info[sorted_min_s_r[ore_i]][0], obstacle_info[sorted_min_s_r[ore_i]][1], obstacle_info[sorted_min_s_r[ore_i]][2], 'red')
                    # for i in range(len(t_can_add)):
                    #     print "can", i, "A:", t_can_add[i].A, "B:", t_can_add[i].BT

                    s_e_v = []
                    s_v_index = []

                    in_can_info = copy.deepcopy(t_can_add)
                    ret_can, ret_index = env.get_cf(in_can_info)
                    s_e_v.append(ret_can)
                    s_v_index.append(ret_index)

                    print "s_e_v: ", s_e_v
                    for i in range(len(s_e_v[0])):
                        print s_e_v[0][i].grid

                    if len(s_e_v[0]) >= len(min_s_r) - 1:
                        print "this se is possible"
                        if len(min_s_r) == 1:
                            print "only one move needed"
                            # t_can_info = []
                            # for i in range(len(env.ore_order)):
                            #     in_can_info = copy.deepcopy(can_info)
                            #     in_obs_pos = copy.deepcopy(env.obs_pos)
                            #     for ore_i in range(i + 1):
                            #         if min_s_r[0] != env.ore_order[ore_i]:
                            #             in_obs_pos.remove(env.obs_pos[env.ore_order[ore_i]])
                            #     in_obs_pos.remove(env.obs_pos[min_s_r[0]])
                            #     t_can_info.append(env.get_can_A(in_can_info, in_obs_pos, env.tar_pos))

                            s_v = [[s_e[s_r.index(min_s_r)]]]
                            s_v_index = [[s_e_index[s_r.index(min_s_r)]]]
                            # print "se v:", s_v, s_v[0], s_v[0][0], s_v[0][0].pos
                            # for i in range(len(env.ore_order)):
                            #     add_can_info = copy.deepcopy(t_can_info[i])
                            #     ret_can, ret_index = env.get_cf(add_can_info)
                            #     s_v.append(ret_can)
                            #     s_v_index.append(ret_index)

                            t_b = [[0]]
                            # for i in range(1):
                            #     in_obs_pos = copy.deepcopy(env.obs_pos)
                            #     for ore_i in range(i+1):
                            #         in_obs_pos.remove(env.obs_pos[env.ore_order[ore_i]])
                            #     t_b.append(env.get_cf_b(s_v[i], in_obs_pos))
                            #     # print "\n step", i, " has cf b:", t_b[i]
                            break  # for out s_e loop
                        else:
                            t_b = []
                            in_obs_pos = copy.deepcopy(env.obs_pos)
                            for ore_i in range(1):
                                in_obs_pos.remove(env.obs_pos[min_s_r[ore_i]])
                            t_b.append(env.get_cf_b(s_e_v[0], in_obs_pos))

                            s_v[0] = s_e_v[0]

                            break  # for out s_e loop
                    else:  # s_e[s_r.index(min_s_r)]
                        print "\nremove",
                        print "s_e:", s_e
                        print "s_r:", s_r
                        print "s_e_index:", s_e_index
                        s_e.remove(s_e[s_r.index(min_s_r)])
                        s_e_index.remove(s_e_index[s_r.index(min_s_r)])
                        s_r.remove(s_r[s_r.index(min_s_r)])

            if len(s_e) == 0:
                # print "no possible extra candidate exist"
                break

            env.ore_order = min_s_r
            # draw_figs = 1
            # if draw_figs == 1:
            #     for c_i in range(len(can_info)):
            #         plt.text(can_info[c_i].grid[0], can_info[c_i].grid[1], 'Can' + str(c_i), fontsize=20, ha='center', bbox=dict(facecolor='pink', alpha=0.8))
            #     for o_i in range(len(env.obs_grid)):
            #         plt.text(env.obs_grid[o_i][0], env.obs_grid[o_i][1], 'Obs' + str(o_i), fontsize=20, ha='center', bbox=dict(facecolor='red', alpha=0.8))
            #
            #     step_i = 0
            #     step_grid = copy.deepcopy(env.grid_act)
            #     step_obs_grid = copy.deepcopy(env.obs_grid)
            #     step_obs_grid.remove(env.obs_grid[env.ore_order[0]])
            #     for i in range(len(step_obs_grid)):
            #         # print "i:", i, "step_obs_grid [i]:", step_obs_grid[i]
            #         step_grid = CUF.obstacle_circle(step_grid, [round(step_obs_grid[i][0], 2), round(step_obs_grid[i][1], 2), env.obs_r[i]], 2)
            #     for ci in range(len(can_info)):
            #         xi, yi = can_info[ci].grid
            #         step_grid = CUF.obstacle_circle(step_grid, [xi, yi, 0.04], 30)
            #
            #     step_grid = CUF.obstacle_circle(step_grid, [env.tar_grid[0], env.tar_grid[1], tar_r], 4)  # target
            #
            #     for cf_i in range(len(t_b[step_i])):
            #         xi = (t_cf[step_i][cf_i].pos[0] - env.ws_zero[0]) * G2P_SIZE
            #         yi = (t_cf[step_i][cf_i].pos[1] - env.ws_zero[1]) * G2P_SIZE
            #         step_grid = CUF.obstacle_circle(step_grid, [xi, yi, 0.04], 3)
            #
            #     CUF.draw_grid_info(step_grid)
            #
            #     for cf_i in range(len(t_b[step_i])):
            #         xi = (t_cf[step_i][cf_i].pos[0] - env.ws_zero[0]) * G2P_SIZE
            #         yi = (t_cf[step_i][cf_i].pos[1] - env.ws_zero[1]) * G2P_SIZE
            #         plt.text(xi, yi, 'b=' + str(t_b[step_i][cf_i]), fontsize=20, ha='center', bbox=dict(facecolor='pink', alpha=0.8))
            #     for ci in range(len(t_can_info[step_i])):
            #         plt.text(t_can_info[step_i][ci].grid[0], t_can_info[step_i][ci].grid[1] - 2.0, '[A, BT] :' + str([t_can_info[step_i][ci].A, t_can_info[step_i][ci].BT]), fontsize=10, ha='center', bbox=dict(facecolor='pink', alpha=0.8))
            #     for o_i in range(len(env.obs_grid)):
            #         plt.text(env.obs_grid[o_i][0], env.obs_grid[o_i][1], 'Obs' + str(o_i), fontsize=20, ha='center', bbox=dict(facecolor='red', alpha=0.8))
            #     plt.title('step' + str(step_i) + " obs: " + str(env.ore_order[step_i]) + " rearranged")

        if space_err:
            print "no possible extra candidate exist"
            break

        # move obstacle to can(min(b))
        # print "s_v", s_v
        # print "s_v[0]", s_v[0]
        # print "s_v[0][0]", s_v[0][0]
        # print "s_v[0][0].pos", s_v[0][0].pos
        print "t_b[0]", t_b[0]

        find_b = copy.deepcopy(t_b[0])
        # print "move to c_", find_b.index(min(find_b))
        if method == 'far':
            t_sel_can_index = [i for i in range(len(find_b))]
        elif method == 'deep':
            t_sel_can_index = [i for i in range(len(find_b))]
        elif method == 'mine':
            t_sel_can_index = [i for i in range(len(find_b)) if find_b[i] == min(find_b)]
        t_sel_can_dist = []
        # print "\ntar grid: ", env.tar_grid
        # print "\ntar pos: ", env.tar_pos
        print "t sel can index", t_sel_can_index
        for i in range(len(t_sel_can_index)):
            # print "t_cf grid x,y:", t_sel_can_index[i], t_cf[0][t_sel_can_index[i]].grid[0], t_cf[0][t_sel_can_index[i]].grid[1]
            # print "t_cf pos x,y:", t_sel_can_index[i], s_v[0][t_sel_can_index[i]].pos[0], s_v[0][t_sel_can_index[i]].pos[1]
            if method == 'deep':
                t_sel_can_dist.append(np.sqrt((env.rob_pos[0] - s_v[0][t_sel_can_index[i]].pos[0]) ** 2 + (env.rob_pos[1] - s_v[0][t_sel_can_index[i]].pos[1]) ** 2))
            else :
                t_sel_can_dist.append(np.sqrt((env.tar_pos[0] - s_v[0][t_sel_can_index[i]].pos[0]) ** 2 + (env.tar_pos[1] - s_v[0][t_sel_can_index[i]].pos[1]) ** 2))
        # print "t sel can dist", t_sel_can_dist
        sel_can_index = t_sel_can_index[t_sel_can_dist.index(max(t_sel_can_dist))]
        # print "sel can index", sel_can_index

        sel_can_pos = can_info[s_v_index[0][sel_can_index]].pos
        sel_can_grid = can_info[s_v_index[0][sel_can_index]].grid

        sel_obs_pos = env.obs_pos[env.ore_order[0]]
        sel_obs_grid = env.obs_grid[env.ore_order[0]]

        env.obs_pos[env.ore_order[0]] = sel_can_pos
        env.obs_grid[env.ore_order[0]] = sel_can_grid

        can_info[s_v_index[0][sel_can_index]].pos = sel_obs_pos
        can_info[s_v_index[0][sel_can_index]].grid = sel_obs_grid

        # tmp_order_time_start = timeit.default_timer()
        # tmp_order_time_start2 = time.clock()
        # env.pick_n_place()
        # CLF.add_box_client(obstacle_name[env.ore_order[0]], [env.object_z, -sel_can_pos[1], sel_can_pos[0]], obstacle_info[env.ore_order[0]][1], obstacle_info[env.ore_order[0]][2], 'blue')
        vrep_env.pick_and_place(sel_obs_pos, obstacle_name[env.ore_order[0]], env.obs_pos[env.ore_order[0]])
        time.sleep(1)
        obstacle_info = []
        for i in range(len(obstacle_name)):
            obstacle_info.append(vrep_env.get_object_info(obstacle_name[i]))
            obstacle_info[i][0][2] = obstacle_info[i][0][2] + 0.04
            obstacle_info[i][2][2] = obstacle_info[i][2][2] + 0.08
        for i in range(len(obstacle_info)):
            CLF.add_box_client(obstacle_name[i], obstacle_info[i][0], obstacle_info[i][1], obstacle_info[i][2], 'red')

        env.set_env(obstacle_name, obstacle_info, target_name, target_info)
        home_joint = [3.1415927410125732, 4.537856101989746, 5.93411922454834, -0.6108652353286743, 1.7453292608261108, -0.5235987901687622]

        CLF.move_joints_client_rad('arm', home_joint)
        env.update_env(env.obs_pos, env.obs_grid)
        # tmp_order_time_end = timeit.default_timer()
        # order_time = order_time + tmp_order_time_end - tmp_order_time_start
        # order_time2 = order_time2 + tmp_order_time_end2 - tmp_order_time_start2
        # order_cnt = order_cnt + 1

        rearr_cnt = rearr_cnt + 1
        if env.order_error_flag == 0:
            print "\nretry for another environment"
            space_err = 1
            break

        print "after move order is:", env.ore_order

        # CUF.draw_grid_info(env.grid_ori)
        # for c_i in range(len(can_info)):
        #     plt.text(can_info[c_i].grid[0], can_info[c_i].grid[1], 'Can' + str(c_i), fontsize=20, ha='center', bbox=dict(facecolor='pink', alpha=0.8))
        # for o_i in range(len(env.obs_grid)):
        #     plt.text(env.obs_grid[o_i][0], env.obs_grid[o_i][1], 'Obs' + str(o_i), fontsize=20, ha='center', bbox=dict(facecolor='red', alpha=0.8))

        if len(env.ore_order) == 0:
            print "end rearrangement"
            vrep_env.pick_and_place(env.tar_pos, 'target', env.tar_pos)
            time.sleep(1)

            #     plt.title('rearrangement finished')
            break
            # else:
            #     plt.title('after rearrngement')

            # plt.show()

    vrep_env.pick_and_place(env.tar_pos, 'target', env.tar_pos)
    time.sleep(1)
    algorithm_end = timeit.default_timer()
    tot_time = algorithm_end - algorithm_start
    print "tot time:", tot_time