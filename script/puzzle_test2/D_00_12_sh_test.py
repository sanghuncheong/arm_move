#!/usr/bin/env python
GRID_SIZE = 0.01
G2P_SIZE = 100

import rospy
import numpy as np
import tf
import matplotlib.pyplot as plt
import copy
import time
import D_00_custom_function as CUF
import D_00_client_function as CLF

from D_20_1020_VFHplus_change_radius import influence

from D_00_envClass4altest import EnvInfo as EI
from D_00_envClass4altest import CanInfo as CI

from arm_move.srv._box_info_srv import *
from arm_move.srv._arm_move_srv import *
from arm_move.srv._work_start_srv import *
from arm_move.srv._att_hand_box_srv import *
from arm_move.srv._arm_goalJoint_srv import *
import timeit

#using Jeeho modules
import timer_class
import parse_testdata


def go_home():
    # 2020.08.05 SH
    move_group_name = 'panda_arm'
    home_joint = [-0.7912285295667355, -1.7449968666946676, 1.6255344777637362, -2.9980328554805484, 1.552371742049853, 1.345932931635115, 0.8050298552807971]
    CLF.move_joints_client_rad(move_group_name, home_joint)


def go_ready():
    # 2020.08.05 SH
    move_group_name = 'panda_arm'
    home_joint = [-1.6238, -1.6078, -0.2229, -2.6057, 1.4646, 1.4325, -0.2159]
    CLF.move_joints_client_rad(move_group_name, home_joint)


def hand_open():
    # 2020.08.05 SH
    CLF.panda_gripper_open()


def pick_and_place(env, pick_pose, pick_object_name, place_pose):
    # print"\tPICK AND PLACE ACTION => rearrange", pick_object_name

    pick_time = env.pick(env.obs_pos, pick_pose, place_pose)
    CLF.att_box_client('hand', pick_object_name)
    ready_time1 = env.go_ready()
    place_time = env.place(env.obs_pos, place_pose)#, vrep_env.get_current_joint(joint_names_jaco))
    CLF.det_box_client(pick_object_name, [0, 0, 0], [0, 0, 0, 0], [0, 0, 0], 'red')
    CLF.add_mesh_client(pick_object_name, [place_pose[0], place_pose[1], 0.605], [0.0, 0.0, 0.0, 0.0], [0.001, 0.001, 0.001])
    ready_time2 = env.go_ready()
    # print"exe time:", pick_time + ready_time1 + place_time + ready_time2
    return pick_time+ready_time1+place_time+ready_time2
    # print"\tEND PICK AND PLACE ACTION"
    #
    # # ret_pick_pose = env.pick(env.obs_pos, pick_pose, place_pose)
    # env.move_to([[ret_pick_pose[0][0] - 0.03, ret_pick_pose[0][1], ret_pick_pose[0][2]], ret_pick_pose[1]])
    #
    # env.move_to([[ret_pick_pose[0][0] + 0.05, ret_pick_pose[0][1], ret_pick_pose[0][2]], ret_pick_pose[1]])
    #
    #
    # env.pre_place(env.obs_pos, place_pose, vrep_env.get_current_joint(joint_names_jaco))
    # ret_place_pose = env.place(env.obs_pos, place_pose, vrep_env.get_current_joint(joint_names_jaco))
    # env.move_to([[ret_place_pose[0][0] - 0.1, ret_place_pose[0][1], ret_place_pose[0][2]], ret_place_pose[1]])
    #
    # CLF.det_box_client(pick_object_name, [env.object_z, -sel_can_pos[1], sel_can_pos[0]], obstacle_info[env.ore_order[0]][1], obstacle_info[env.ore_order[0]][2], 'blue')
    #
    # env.move_to([[ret_place_pose[0][0] + 0.1, ret_place_pose[0][1], ret_place_pose[0][2]], ret_place_pose[1]])
    # # CLF.add_box_client(obstacle_name[env.ore_order[0]], [env.object_z, -sel_can_pos[1], sel_can_pos[0]], obstacle_info[env.ore_order[0]][1], obstacle_info[env.ore_order[0]][2], 'blue')


def test_algorithm(method_in, data_in):
    # method
    # "where" : icra2020 "where to relocate?"
    # "far" : farthest method
    go_ready()
    hand_open()
    # print "start with method:", method_in
    # print "\n***STEP 1*** : env setting"

    obj_h = -0.0
    obj_z = 0.605 + obj_h#+ obj_h/2.0
    target_name = ['target']
    target_info = []
    if -data_in[0][0]>0:
        target_info.append([[data_in[0][1] - 0.03, -data_in[0][0]-0.02, obj_z], [0, 0, 0, 0], [0.001, 0.001, 0.001]])  # for the add_mesh
    else:
        target_info.append([[data_in[0][1] - 0.03, -data_in[0][0]+0.02, obj_z], [0, 0, 0, 0], [0.001, 0.001, 0.001]]) # for the add_mesh
    # target_info.append([[data_in[0][0], data_in[0][1], obj_z], [0, 0, 0, 0], [0.06, 0.06, 0.12]]) # for the add_box
    # target_info[i][0][2] = target_info[i][0][2] + 0.04
    # target_info[i][2][2] = target_info[i][2][2] + 0.08

    # obstacle_name = []
    # for i in range(len(data_in[1])):
    obstacle_name = [str(i).zfill(2) for i in range(len(data_in[1]))]
        # obstacle_name.append('obstacle'+str(i))
    # print obstacle_name
    # obstacle_name = ['obstacle0', 'obstacle1', 'obstacle2', 'obstacle3', 'obstacle4', 'obstacle5', 'obstacle6', 'obstacle7', 'obstacle8']
    obstacle_info = []
    # [[obj_pos.x, obj_pos.y, obj_pos.z], [obj_ori_q.x, obj_ori_q.y, obj_ori_q.z, obj_ori_q.w], [obj_scale.x, obj_scale.y, obj_scale.z]]
    for i in range(len(obstacle_name)):
        obstacle_info.append([[data_in[1][i][1]-0.02, -data_in[1][i][0], obj_z], [0, 0, 0, 0], [0.001, 0.001, 0.001]]) # for the add_mesh
        # obstacle_info.append([[data_in[1][i][0], data_in[1][i][1], obj_z], [0, 0, 0, 0], [0.06, 0.06, 0.12]]) # for the add_box
        # obstacle_info[i][0][2] = obstacle_info[i][0][2] + 0.04
        # obstacle_info[i][2][2] = obstacle_info[i][2][2] + 0.08
    can_r = 0.02
    if len(obstacle_name)>17:
        can_r = 0.035
        # print "can r", len(obstacle_name), can_r
    elif len(obstacle_name)>13:
        can_r = 0.045
        # print "can r", len(obstacle_name), can_r
    elif len(obstacle_name)>10:
        can_r = 0.045
    # print "can r", len(obstacle_name), can_r
    # print "\tNo. of obstacles:", len(obstacle_name)

    env_name = ['shelf_gazebo']#2020.10.21: puzzle test, 'Jaco_base', 'table_ls', 'table_rs', 'table_us', 'table_bs']
    env_info = []

    base_position = [0.8637-0.02, 0, 0.0 + obj_h]
    base_quaternion = [0, 0, 0, 1]
    base_scale = [0.001, 0.001, 0.001]
    CLF.add_mesh_client('shelf_gazebo', base_position, base_quaternion, base_scale)

    ws_pos = [0.8637+0.5*0.45+0.03-0.02, 0.0, 0.0 + obj_h]
    ws_rot = [0.0, 0.0, 0.0, 0.0]
    ws_scale = [0.45, 0.91, 0.0]
    env_info.append([ws_pos, ws_rot, ws_scale])

    # for i in range(len(env_name)):
    #     env_info.append(vrep_env.get_object_info(env_name[i]))
    #     if i > 1:
    #         env_info[i][2][0] = env_info[i][2][0]+0.01
    #         env_info[i][2][1] = env_info[i][2][1]+0.01
    #         env_info[i][2][2] = env_info[i][2][2]+0.01

    for i in range(len(obstacle_info)):
        CLF.add_mesh_client(obstacle_name[i], obstacle_info[i][0], obstacle_info[i][1], obstacle_info[i][2])
        # CLF.add_box_client(obstacle_name[i], obstacle_info[i][0], obstacle_info[i][1], obstacle_info[i][2], 'red')
    for i in range(len(target_info)):
        CLF.add_mesh_client(target_name[i], target_info[i][0], target_info[i][1], target_info[i][2])
        # CLF.add_box_client(target_name[i], target_info[i][0], target_info[i][1], target_info[i][2], 'green')
    # for i in range(len(env_info)):
    #     # CLF.add_mesh_client(env_name[i], env_info[i][0], env_info[i][1], env_info[i][2])
    #     CLF.add_box_client(env_name[i], env_info[i][0], env_info[i][1], env_info[i][2], 'gray')

    ws = env_info[0]
    # print"ws info", env_info[0]
    ws_w = int(round(ws[2][0]*100))  # x-axes in Rviz
    ws_d = int(round(ws[2][1]*100))  # y-axes in Rviz
    # print "\tRviz ws width, depth:", ws_w, ws_d
    # GRID_SIZE = 0.01
    ws_zero_pos = [round(ws[0][0] - ws[2][0]*0.5, 2), round(ws[0][1] - ws[2][1]*0.5, 2)]
    # print "\tRviz ws   cen pos:", ws[0]
    # print "\tRviz ws, zero pos:", ws_zero_pos

    # ws_w, ws_d = 100, 100  # get table size in the v-rep
    ws_cen = [-ws[0][1], ws[0][0]]
    rob_pos = [0.0, 0.0]
    OBJ_R = 0.035

    env = EI(rob_pos, ws_w, ws_d, ws_cen, ws_zero_pos, grid_size=GRID_SIZE, wall_r=OBJ_R)
    env.set_env(obstacle_name, obstacle_info, target_name, target_info)

    env.update_env(env.obs_pos, env.obs_grid)
    print "env.ore_order:", env.ore_order, env.ore_order[0]
    # print "\trearrangement order:", env.ore_order
    # if len(env.ore_order) == 0:
    #     print "end rearrangement"
    #     pick_and_place(env, env.tar_pos, 'target', env.tar_pos)
    #     time.sleep(1)

    # CUF.draw_grid_info(env.grid_ori)
    # plt.show()
    ret = 0
    space_err = 0
    rearr_cnt = 0
    reloc_cnt = len(env.ore_order)
    # print"first reloc cnt:", reloc_cnt

    # env.get_env(obs_r, tar_r, min_ore)
    algorithm_start = time.time()
    tp_time = 0
    mp_time = 0
    ex_time = 0

    method = method_in
    # method = 'mine'
    # method = 'far'
    # method = 'deep'

    while len(env.ore_order):  # this while loop is for the algorithm
        # print"\n***STEP 2*** REARRANGE ORDER => :", env.ore_order
        env.get_max_can(env.grid_ori, bt_num=2, trial_num=2500, can_r=can_r)  # We get "grid_max_can", "can_grid"
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

        # CUF.draw_grid_info(env.grid_ori)
        # CUF.draw_grid_info(env.grid_del)
        # CUF.draw_grid_info(env.grid_max_can)
        # for c_i in range(len(can_info)):
        #     plt.text(can_info[c_i].grid[0], can_info[c_i].grid[1], 'Can' + str(c_i), fontsize=20, ha='center', bbox=dict(facecolor='pink', alpha=0.8))
        # for o_i in range(len(env.obs_grid)):
        #     plt.text(env.obs_grid[o_i][0], env.obs_grid[o_i][1], 'Obs' + str(o_i), fontsize=20, ha='center', bbox=dict(facecolor='red', alpha=0.8))
        # plt.show()


        # print"\tCheck C.A"
        # Check C.A : just next step
        t_can_info = []

        in_can_info = copy.deepcopy(can_info)
        in_obs_pos = copy.deepcopy(env.obs_pos)
        in_obs_pos.remove(env.obs_pos[env.ore_order[0]])
        CLF.del_box_client(obstacle_name[env.ore_order[0]])
        tmp_mp_start = time.time()
        t_can_info.append(env.get_can_A(in_can_info, in_obs_pos, env.tar_pos))
        tmp_mp_end = time.time()
        mp_time = mp_time + tmp_mp_end - tmp_mp_start

        print "in loop env.ore_order:", env.ore_order, env.ore_order[0]
        CLF.add_mesh_client(obstacle_name[env.ore_order[0]], obstacle_info[env.ore_order[0]][0], obstacle_info[env.ore_order[0]][1], obstacle_info[env.ore_order[0]][2])
        # CLF.add_box_client(obstacle_name[env.ore_order[0]], obstacle_info[env.ore_order[0]][0], obstacle_info[env.ore_order[0]][1], obstacle_info[env.ore_order[0]][2], 'red')

        # Check C.BT
        in_can_info = copy.deepcopy(t_can_info[0])
        in_can_info = env.init_BT(in_can_info)  # init the BT value of candidates to '0'
        in_obs_pos = copy.deepcopy(env.obs_pos)
        for ore_i in range(len(env.ore_order)):     # after rearrange all ORE
            in_obs_pos.remove(env.obs_pos[env.ore_order[ore_i]])
            CLF.del_box_client(obstacle_name[env.ore_order[ore_i]])

        tmp_mp_start = time.time()
        t_can_info[0] = env.get_can_BT(in_can_info, in_obs_pos, env.tar_pos)
        tmp_mp_end = time.time()
        mp_time = mp_time + tmp_mp_end - tmp_mp_start
        for ore_i in range(len(env.ore_order)):
            CLF.add_mesh_client(obstacle_name[env.ore_order[ore_i]], obstacle_info[env.ore_order[ore_i]][0], obstacle_info[env.ore_order[ore_i]][1], obstacle_info[env.ore_order[ore_i]][2])
            # CLF.add_box_client(obstacle_name[env.ore_order[ore_i]], obstacle_info[env.ore_order[ore_i]][0], obstacle_info[env.ore_order[ore_i]][1], obstacle_info[env.ore_order[ore_i]][2], 'red')

        # Check C.BO : BO : other ORE, just before target

        in_can_info = copy.deepcopy(t_can_info[0])
        in_obs_pos = copy.deepcopy(env.obs_pos)
        for ore_i in range(len(env.ore_order)):     # after rearrange all ORE
            in_obs_pos.remove(env.obs_pos[env.ore_order[ore_i]])
            CLF.del_box_client(obstacle_name[env.ore_order[ore_i]])
        for j in range(len(env.ore_order)):     # check other ORE just before target
            if j > i:
                tmp_mp_start = time.time()
                t_can_info[0] = env.get_can_BT(in_can_info, in_obs_pos, env.obs_pos[env.ore_order[j]])
                tmp_mp_end = time.time()
                mp_time = mp_time + tmp_mp_end - tmp_mp_start
        for ore_i in range(len(env.ore_order)):
            CLF.add_mesh_client(obstacle_name[env.ore_order[ore_i]], obstacle_info[env.ore_order[ore_i]][0], obstacle_info[env.ore_order[ore_i]][1], obstacle_info[env.ore_order[ore_i]][2])
            # CLF.add_box_client(obstacle_name[env.ore_order[ore_i]], obstacle_info[env.ore_order[ore_i]][0], obstacle_info[env.ore_order[ore_i]][1], obstacle_info[env.ore_order[ore_i]][2], 'red')

        s_v = []
        s_v_index = []
        for i in range(1):
            in_can_info = copy.deepcopy(t_can_info[i])
            ret_can, ret_index = env.get_cf(in_can_info)
            s_v.append(ret_can)
            s_v_index.append(ret_index)
            # print "\n step", i, " has # of cf pos:", len(t_cf[i]), "index", t_cf_index[i]

        # print"\n***STEP 3*** : find valid candidates"
        # print "\ts_v:", len(s_v[0]), "\n\ts_v_index:", len(s_v_index[0])
        # for i in range(len(s_v[0])):
        #     print "s_v index:", [i], s_v_index[0][i]
        # See the feasibile candidate
        # for i in range(len(t_cf[0])):
        #     print "\n Our Cf pos:", i, t_cf[0][i].pos
        # See if this case if case0 or case1
        # print "t_cf:", t_cf, "order", env.ore_order

        # if len(s_v[0]) >= len(env.ore_order):
        if len(s_v[0]) >= 1:
            # print "\n\tenough candidate spots"
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
            # print "\n\tnot enough candidate spots"
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
            # print "\t# of OR'", len(ret_can)

            t_s_e = ret_can
            t_s_e_index = ret_index
            # print "t_cp:", len(t_cp), "index", t_cp_index
            # for i in range(len(t_cp)):
            #     print "\n Our Cp:", i, t_cp[i].pos

            if len(t_s_e) == 0:
                # print "\tno possible extra candidate exist"
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
            # print "\n"
            # for i in range(len(t_s_e)):
            #     print "can", t_s_e_index[i], "grid:", t_s_e[i].grid, ", s_r:", t_s_r[i]

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
                # print "# of s_e:", len(s_e), s_r
                # print "\n"
                # for i in range(len(s_e)):
                #     print "can", s_e_index[i], "pos:", s_e[i].pos, ", s_r:", s_r[i]
                min_s_r = CUF.min_len_list(s_r)

                # print "\nmin sr:", min_s_r
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
                # t_can_add = copy.deepcopy(env.get_can_A(in_can_info, in_obs_pos, in_tar_pos))
                t_can_add = copy.deepcopy(env.get_can_A(in_can_info, in_obs_pos, env.tar_pos))

                # Check C.BT for this environment state
                in_can_info = copy.deepcopy(t_can_add)
                in_can_info = env.init_BT(in_can_info)  # init the BT value of candidates to '0'
                in_obs_pos = copy.deepcopy(env.obs_pos)

                sorted_min_s_r = copy.deepcopy(min_s_r)
                sorted_min_s_r.sort(reverse=True)
                # print "sorted min_s_r:", sorted_min_s_r

                if sorted_min_s_r[0] == len(env.obs_pos):  # if OR' has o_t ! remove s_e
                    # print "o_t is in OR'"
                    s_e.remove(s_e[s_r.index(min_s_r)])
                    s_e_index.remove(s_e_index[s_r.index(min_s_r)])
                    s_r.remove(s_r[s_r.index(min_s_r)])
                else:
                    for ore_i in range(len(min_s_r)):  # after rearrange all OR'
                        in_obs_pos.remove(in_obs_pos[sorted_min_s_r[ore_i]])
                        CLF.del_box_client(obstacle_name[sorted_min_s_r[ore_i]])
                    in_tar_pos = copy.deepcopy([-cp.pos[1], cp.pos[0]])
                    t_can_add = env.get_can_BT(in_can_info, in_obs_pos, in_tar_pos)

                    for ore_i in range(len(min_s_r)):  # after rearrange all OR'
                        CLF.add_mesh_client(obstacle_name[sorted_min_s_r[ore_i]], obstacle_info[sorted_min_s_r[ore_i]][0], obstacle_info[sorted_min_s_r[ore_i]][1], obstacle_info[sorted_min_s_r[ore_i]][2])
                        # CLF.add_box_client(obstacle_name[sorted_min_s_r[ore_i]], obstacle_info[sorted_min_s_r[ore_i]][0], obstacle_info[sorted_min_s_r[ore_i]][1], obstacle_info[sorted_min_s_r[ore_i]][2], 'red')
                    # for i in range(len(t_can_add)):
                    #     print "can", i, "A:", t_can_add[i].A, "B:", t_can_add[i].BT

                    s_e_v = []
                    s_v_index = []

                    in_can_info = copy.deepcopy(t_can_add)
                    ret_can, ret_index = env.get_cf(in_can_info)
                    s_e_v.append(ret_can)
                    s_v_index.append(ret_index)

                    # print "s_e_v: ", s_e_v
                    # for i in range(len(s_e_v[0])):
                    #     print s_e_v[0][i].grid

                    if len(s_e_v[0]) >= len(min_s_r) - 1:
                        # print "this se is possible"
                        if len(min_s_r) == 1:
                            # print "only one move needed"
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
                        # print "\nremove",
                        # print "s_e:", s_e
                        # print "s_r:", s_r
                        # print "s_e_index:", s_e_index
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
            # print "no possible extra candidate exist"
            break

        # move obstacle to can(min(b))
        # print "s_v", s_v
        # print "s_v[0]", s_v[0]
        # print "s_v[0][0]", s_v[0][0]
        # print "s_v[0][0].pos", s_v[0][0].pos
        # print "\tt_b[0]", t_b[0]

        find_b = copy.deepcopy(t_b[0])
        # print "move to c_", find_b.index(min(find_b))
        if method == 'far':
            t_sel_can_index = [i for i in range(len(find_b))]
        elif method == 'deep':
            t_sel_can_index = [i for i in range(len(find_b))]
        elif method == 'SH_re':
            t_sel_can_index = [i for i in range(len(find_b)) if find_b[i] == min(find_b)]
        t_sel_can_dist = []
        # print "\ntar grid: ", env.tar_grid
        # print "\ntar pos: ", env.tar_pos
        # print "\tt sel can index", t_sel_can_index
        for i in range(len(t_sel_can_index)):
            # print "t_cf grid x,y:", t_sel_can_index[i], t_cf[0][t_sel_can_index[i]].grid[0], t_cf[0][t_sel_can_index[i]].grid[1]
            # print "t_cf pos x,y:", t_sel_can_index[i], s_v[0][t_sel_can_index[i]].pos[0], s_v[0][t_sel_can_index[i]].pos[1]
            if method == 'deep':
                t_sel_can_dist.append(np.sqrt((env.rob_pos[1] - s_v[0][t_sel_can_index[i]].pos[0]) ** 2 + (-env.rob_pos[0] - s_v[0][t_sel_can_index[i]].pos[1]) ** 2))
            else:
                t_sel_can_dist.append(np.sqrt((env.tar_pos[1] - s_v[0][t_sel_can_index[i]].pos[0]) ** 2 + (-env.tar_pos[0] - s_v[0][t_sel_can_index[i]].pos[1]) ** 2))

        # print "obs pos:", env.obs_pos
        # print "t sel can dist", t_sel_can_dist
        sel_can_index = t_sel_can_index[t_sel_can_dist.index(max(t_sel_can_dist))]

        # print "sel can index", sel_can_index

        sel_can_pos = can_info[s_v_index[0][sel_can_index]].pos
        # sel_can_pos = [can_info[s_v_index[0][sel_can_index]].pos[1], -can_info[s_v_index[0][sel_can_index]].pos[0]]
        sel_can_grid = can_info[s_v_index[0][sel_can_index]].grid

        # print"\npick obj in RVIZ:", env.obs_pos[env.ore_order[0]][1], -env.obs_pos[env.ore_order[0]][0]
        # print"pick obj in ALG :", env.obs_pos[env.ore_order[0]]

        sel_obs_pos = env.obs_pos[env.ore_order[0]]
        sel_obs_grid = env.obs_grid[env.ore_order[0]]

        # print"obstacle pos change:", [-can_info[s_v_index[0][sel_can_index]].pos[1], -can_info[s_v_index[0][sel_can_index]].pos[0]]
        env.obs_pos[env.ore_order[0]] = [-can_info[s_v_index[0][sel_can_index]].pos[1], can_info[s_v_index[0][sel_can_index]].pos[0]]
        env.obs_grid[env.ore_order[0]] = sel_can_grid

        # print"place obj in RVIZ:", sel_can_pos
        # print"place obj in ALG :", env.obs_pos[env.ore_order[0]]

        # can_info[s_v_index[0][sel_can_index]].pos = sel_obs_pos
        # can_info[s_v_index[0][sel_can_index]].grid = sel_obs_grid

        # tmp_order_time_start = timeit.default_timer()
        # tmp_order_time_start2 = time.clock()
        # env.pick_n_place()
        # CLF.add_box_client(obstacle_name[env.ore_order[0]], [env.object_z, -sel_can_pos[1], sel_can_pos[0]], obstacle_info[env.ore_order[0]][1], obstacle_info[env.ore_order[0]][2], 'blue')

        # pick_and_place(env, sel_obs_pos, obstacle_name[env.ore_order[0]], env.obs_pos[env.ore_order[0]])
        # print "obs pos:", env.obs_pos
        tmp_ex_time = pick_and_place(env, sel_obs_pos, obstacle_name[env.ore_order[0]], sel_can_pos)
        ex_time = ex_time + tmp_ex_time
        obstacle_info[env.ore_order[0]][0] = [can_info[s_v_index[0][sel_can_index]].pos[0], can_info[s_v_index[0][sel_can_index]].pos[1], 0.605]
        env.obs_pos[env.ore_order[0]] = copy.deepcopy([-can_info[s_v_index[0][sel_can_index]].pos[1], can_info[s_v_index[0][sel_can_index]].pos[0]])
        env.obs_grid[env.ore_order[0]] = sel_can_grid
        # def pick_and_place(env, pick_pose, pick_object_name, place_pose):
        #     print"\tPICK AND PLACE ACTION => rearrange", pick_object_name
        #
        #     env.pick(env.obs_pos, pick_pose, place_pose)
        #     CLF.att_box_client('hand', pick_object_name)
        #     env.go_ready()
        #     env.place(env.obs_pos, place_pose)  # , vrep_env.get_current_joint(joint_names_jaco))
        #     CLF.det_box_client(pick_object_name, [0, 0, 0], [0, 0, 0, 0], [0, 0, 0], 'red')
        #     CLF.add_mesh_client(pick_object_name, [place_pose[1], -place_pose[0], 0.605], [0.0, 0.0, 0.0, 0.0], [0.001, 0.001, 0.001])
        #     env.go_ready()
        #     print"\tEND PICK AND PLACE ACTION"
        #

        # time.sleep(1)
        # obstacle_info = []
        # for i in range(len(obstacle_name)):
        #     obstacle_info.append(vrep_env.get_object_info(obstacle_name[i]))
        #     # obstacle_info[i][0][2] = obstacle_info[i][0][2] + 0.04
        #     # obstacle_info[i][2][2] = obstacle_info[i][2][2] + 0.08
        # for i in range(len(obstacle_info)):
        #     CLF.add_box_client(obstacle_name[i], obstacle_info[i][0], obstacle_info[i][1], obstacle_info[i][2], 'red')

        # env.set_env(obstacle_name, obstacle_info, target_name, target_info)
        # home_joint = [3.1415927410125732, 4.537856101989746, 5.93411922454834, -0.6108652353286743, 1.7453292608261108, -0.5235987901687622]
        #
        # CLF.move_joints_client_rad('arm', home_joint)
        if len(env.ore_order) == 1:
            rearr_cnt = rearr_cnt + 1
            # print "end rearrangement"
            # pick_and_place(env, env.tar_pos, 'target', env.tar_pos)
            # time.sleep(1)

            #     plt.title('rearrangement finished')
            break
        ret = env.update_env(env.obs_pos, env.obs_grid)
        print "after update env -> env.ore_order:", env.ore_order#, env.ore_order[0]

        # print "obs pos:", env.obs_pos
        if len(env.ore_order) == 0:
            rearr_cnt = rearr_cnt + 1
            # print "end rearrangement"
            # pick_and_place(env, env.tar_pos, 'target', env.tar_pos)
            # time.sleep(1)

            #     plt.title('rearrangement finished')
            break
        if env.ore_order[0] == -1:
            ret = -99
            # print "no path"
            break


        # print "obs pos:", env.obs_pos
        # tmp_order_time_end = timeit.default_timer()
        # order_time = order_time + tmp_order_time_end - tmp_order_time_start
        # order_time2 = order_time2 + tmp_order_time_end2 - tmp_order_time_start2
        # order_cnt = order_cnt + 1

        rearr_cnt = rearr_cnt + 1
        if env.order_error_flag == 0:
            # print "\nretry for another environment"
            space_err = 1
            break

        # print "after move order is:", env.ore_order
        # CUF.draw_grid_info(env.grid_ori)
        # for c_i in range(len(can_info)):
        #     plt.text(can_info[c_i].grid[0], can_info[c_i].grid[1], 'Can' + str(c_i), fontsize=20, ha='center', bbox=dict(facecolor='pink', alpha=0.8))
        # for o_i in range(len(env.obs_grid)):
        #     plt.text(env.obs_grid[o_i][0], env.obs_grid[o_i][1], 'Obs' + str(o_i), fontsize=20, ha='center', bbox=dict(facecolor='red', alpha=0.8))
        if len(env.ore_order) == 0:
            # print "end rearrangement"
            # pick_and_place(env, env.tar_pos, 'target', env.tar_pos)
            # time.sleep(1)

            #     plt.title('rearrangement finished')
            break
            # else:
            #     plt.title('after rearrngement')

            # plt.show()

    # pick_and_place(env, env.tar_pos, 'target', env.tar_pos)
    # time.sleep(1)
    if ret == -99:
        return -99, -99, -99, -99, -99, -99

    algorithm_end = time.time()#it.default_timer()
    tot_time = algorithm_end - algorithm_start
    tp_time = tot_time - mp_time - ex_time

    print "n-rearrangement", reloc_cnt, rearr_cnt
    print "tot, tp, mp, ex time:", tot_time, tp_time, mp_time, ex_time, "\n"
    return reloc_cnt, rearr_cnt, tot_time, tp_time, mp_time, ex_time

if __name__ == "__main__":

    # X = [0.013,-0.324,-0.347,0.161,0.327,0.169,0.005,-0.307,-0.022,0.127,0.294,0.195,-0.193,-0.135,-0.316,-0.166]
    # Y = [1.10545,1.15995,0.96495,1.18595,0.94395,0.93395,1.21495,1.07945,1.2677,1.06945,1.15995,1.2677,0.95595,1.2677,1.2677,1.07545]
    # data_in = []
    # obs_list = []
    # tar_n = 13

    # X = [0.331,-0.18,-0.331,0.284,-0.028,0.314,0.006,0.178,0.36,-0.133,0.165,0.025,0.146,-0.195,0.16,-0.299]
    # Y = [1.17095,1.2677,1.04645,1.07245,1.2677,1.2677,1.22895,1.2677,0.96295,0.95695,0.96595,1.11545,1.17395,1.05345,1.06245,1.17695]
    # data_in = []
    # obs_list = []
    # tar_n = 5
    # X.pop(tar_n)
    # Y.pop(tar_n)
    #
    # X = [0.018,0.326,-0.179,0.344,-0.318,-0.314,0.138,-0.12,0.138,-0.185,-0.289,0.005,0.015,0.303,-0.291,0.284]
    # Y = [1.10445,0.94095,1.07545,1.07345,0.94595,1.05645,0.95895,0.96095,1.04945,1.18395,1.15795,1.22795,1.2677,1.18095,1.2677,1.2677]
    # data_in = []
    # obs_list = []
    # tar_n = 12
    # X.pop(tar_n)
    # Y.pop(tar_n)

    # X = [-0.32,0.02,-0.148,-0.129,-0.007,0.357,-0.017,0.159,-0.16,0.299,0.309,0.154,-0.17,-0.323,0.149,-0.325]
    # Y = [0.94595,1.21395,1.05645,0.96595,1.10945,1.05245,1.2677,0.94095,1.18395,1.2677,0.96095,1.2677,1.2677,1.16095,1.07745,1.2677]
    # data_in = []
    # obs_list = []
    # tar_n = 11
    # X.pop(tar_n)
    # Y.pop(tar_n)
    #
    # X = [-0.001,-0.17,-0.354,-0.346,-0.145,0.036,0.315,0.024,-0.287,0.334,0.147,0.184,-0.135,-0.306,0.155,-0.178]
    # Y = [1.10645,1.04645,0.95195,1.2677,1.2677,1.2677,1.17995,1.22895,1.06045,1.05745,1.07745,0.93795,0.94395,1.18095,1.2677,1.18695]
    # data_in = []
    # obs_list = []
    # tar_n = 13
    # X.pop(tar_n)
    # Y.pop(tar_n)
    #
    # X = [-0.326,-0.142,0.184,0.329,0.0,0.151,0.348,-0.326,0.147,-0.331,-0.182,0.316,-0.284,0.027,0.135,0.034]
    # Y = [0.93295,1.04845,1.2677,1.05645,1.11145,0.95595,0.93595,1.18395,1.05845,1.07745,1.15995,1.2677,1.2677,1.2677,1.16195,1.19895]
    # data_in = []
    # obs_list = []
    # tar_n = 14
    # X.pop(tar_n)
    # Y.pop(tar_n)
    #

    is_reading_from_file = False
    # for debug only
    # sys.argv = ["0", "0", "2"]
    #read testdata from file
    default_N = 16
    testdata_path = ""
    # dummy data
    testdata_set = parse_testdata.testdata("", 0)

    from os.path import expanduser
    home = expanduser("~")
    # N from input arg
    N = 0
    if (len(sys.argv) > 1):
        N = int(sys.argv[1])
        # print("Starting with N = " + sys.argv[1])
        # read_testdata = False

    # if there was no input argument
    if (N == 0):
        N = default_N
        # read_testdata = False

    # read from testdata
    if (len(sys.argv) > 2):
        data_ind = int(sys.argv[2])
        is_reading_from_file = True
        # print("\nReading Test Data from File")
        if (len(sys.argv) > 3):
            testdata_path = sys.argv[3]
        else:
            testdata_path = home + "/test_data.txt"
        # print("File path: " + testdata_path)

    if (is_reading_from_file == True):
        testdata_set = parse_testdata.testdata(testdata_path, data_ind)
        N = testdata_set.N

    # print("Starting with N = " + str(N))

    data_in = []
    obs_list = []
    R = []
    H = []
    tar_n = 0
    if(is_reading_from_file == False):
        X = [0.34,0.343,-0.3,0.29,-0.295,0.197,0.123,-0.025,-0.359,0.191,-0.016,0.0,0.281,-0.12,-0.16,0.124]
        Y = [1.2677,1.18995,1.2677,1.06845,1.15995,1.06745,1.2677,1.2677,1.05245,1.16795,1.20895,1.10945,0.95095,1.2677,1.16695,0.93895]
        tar_n = 6

    else:
        X = testdata_set.X
        Y = testdata_set.Y
        R = testdata_set.R
        H = testdata_set.H
        tar_n = testdata_set.target
        # print"input tar_n:", tar_n

    # tar_x = X.pop(tar_n)
    # tar_y = Y.pop(tar_n)
    data_in.append([X[tar_n], Y[tar_n]])
    for i in range(len(X)):
        if i != tar_n:
            obs_list.append([X[i], Y[i]])
    data_in.append(obs_list)
    # print data_in
    method = "SH_re"
    n_reloc_obj, n_rearr_obj, tot_time, tp_time, mp_time, ex_time = test_algorithm(method, data_in)

    # Record Time
    t = time.localtime()
    current_time = time.strftime("%H:%M:%S", t)

    #time_box.time_list_graph.add_to_file(home + "/test_log.csv", current_time)
    sim_record = open(home + "/simulation_records_sh_12.csv", 'a')
    sim_record.write("Simulation Results Recorded at: " + current_time + "\n")
    sim_record.write("N: " + str(N) + "\n")
    sim_record.write("Total Time Taken: " + str(tot_time) + " sec" + "\n")
    sim_record.write("Objects Relocated: " + str(n_rearr_obj) + "\n")
    sim_record.write("Actions Taken: " + str(n_rearr_obj) + "\n")
    sim_record.write("Time on Graph: " + str(tp_time) + "\n")
    sim_record.write("Time on Motion Planning: " + str(mp_time) + " sec" + "\n")
    sim_record.write("Time on Motion Execution: " + str(ex_time) + " sec" + "\n")
    sim_record.write("testdata set line: " + testdata_set.raw + "\n\n")
    sim_record.close()


    # before
    '''
    X = ['0.03', '-0.01', '0.36', '0.30', '-0.19', '-0.05', '-0.29', '0.22', '0.19', '0.14', '-0.12']
    Y = ['1.22', '1.11', '1.04', '1.17', '1.06', '1.31', '1.17', '1.31', '1.06', '1.19', '1.13']

    data_in = []
    data_in.append([-0.17, 1.22])
    obs_list = []

    for i in range(len(X)):
            obs_list.append([float(X[i]), float(Y[i])])
    data_in.append(obs_list)
    print "data:", data_in
    method = "where"
    test_algorithm(method, data_in)
    '''