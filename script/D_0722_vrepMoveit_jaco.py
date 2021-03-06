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
import S_custom_function as CUF
import S_client_function as CLF

from VFHplus_change_radius import influence
import tree_making_plot as TM_P
# import tree_making_no_plot as TM

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
    env = vrep_env_jaco()
    '''
    for jaco, joint names are Jaco_joint1, Jaco_joint2, ...
    '''
    joint_names_jaco = ['Jaco_joint1', 'Jaco_joint2', 'Jaco_joint3', 'Jaco_joint4', 'Jaco_joint5', 'Jaco_joint6']
    joint_names_hubo_r = ['RF1', 'Jaco_joint2', 'Jaco_joint3', 'Jaco_joint4', 'Jaco_joint5', 'Jaco_joint6']
    joint_names_hubo_l = ['LF1', 'Jaco_joint2', 'Jaco_joint3', 'Jaco_joint4', 'Jaco_joint5', 'Jaco_joint6']
    env.set_current_joint(joint_names_jaco)

    target_name = ['target']
    target_info = []
    for i in range(len(target_name)):
        target_info.append(env.get_object_info(target_name[i]))

    obstacle_name = ['obstacle1', 'obstacle2', 'obstacle3', 'obstacle4', 'obstacle5', 'obstacle6', 'obstacle7', 'obstacle8', 'obstacle9']
    obstacle_info = []  # [[obj_pos], [obj_ori_q], [obj_scale]]
    for i in range(len(obstacle_name)):
        obstacle_info.append(env.get_object_info(obstacle_name[i]))

    env_name = ['customizableTable_forwarder', 'Jaco_base', 'table_ls', 'table_rs', 'table_us', 'table_bs']
    env_info = []
    for i in range(len(env_name)):
        env_info.append(env.get_object_info(env_name[i]))

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
    print "ws, zero pos", ws_zero_pos

    grid_acc = np.zeros([ws_w, ws_d])
    grid_acc = CUF.mark_edge_grid(grid_acc)
    rob_pos = [0.0, 0.0]
    obs_r = []
    obs_pos = []
    obs_ori = []
    obs_grid = []
    tar_pos = [round(target_info[0][0][2], 2), round(-target_info[0][0][1], 2)]
    tar_grid = [int(round((target_info[0][0][2] - ws_zero_pos[0]) * 100)), int(round((-target_info[0][0][1] - ws_zero_pos[1]) * 100))]
    tar_r = [round(target_info[0][2][1] * 0.5, 2)]
    print "number of obstacles", len(obstacle_info)
    for i in range(len(obstacle_info)):
        obs_pos.append([round(obstacle_info[i][0][2], 2), round(-obstacle_info[i][0][1], 2)])
        obs_grid.append([int(round((obstacle_info[i][0][2] - ws_zero_pos[0])*100)), int(round((-obstacle_info[i][0][1] - ws_zero_pos[1])*100))])
        obs_r.append(round(obstacle_info[i][2][1] * 0.5, 2))
        obs_ori.append([0.0, 0.0, 0.0])

    ob = len(obs_pos)
    print "obstacles", obs_pos
    print "target_vfh", tar_pos
    print "target_rviz", target_info[0]
    print "robot pos", rob_pos
    d_max = 1.0
    tm_tar_pos = copy.deepcopy(tar_pos)
    tm_tar_ori = [0.0, 0.0, 0.0]
    tm_obs_pos = copy.deepcopy(obs_pos)
    ob = len(tm_obs_pos)
    tm_obs_ori = []
    for i in range(ob):
        tm_obs_ori.append([0.0, 0.0, 0.0])

    re_order = [7, 6, 3, 'T']
    print "rearrangement of obstacles", re_order
    if len(re_order) == 1:
        print "no need to rearrange => pick", re_order[0]
        # f_check = influence(ob, tar_pos, obs_pos, rob_pos, d_max, eta=45)
        # print "\n\ntarget pos ori", target_info[0][0], target_info[0][1]
        # print "point!", f_check
        f_check_pos = target_info[0][0]
        f_check_pos[2] = f_check_pos[2] - 0.17
        f_check_ori = [-0.00145713772037, -0.998970756926, 0.0364956710831,  0.0268955302573]
        print f_check_pos, f_check_ori
        CLF.move_goalpose_client('arm', f_check_pos, f_check_ori)
        CLF.att_box_client('hand', 'target')
        CLF.move_goalpose_client('arm', [0.3, 0.0, 0.5], f_check_ori)
        CLF.det_box_client('target', [0.3, 0.0, 0.5+0.17], target_info[0][1], target_info[0][2], 'green')

    rearrange_order = copy.deepcopy(re_order)
    ore_grid = []
    ore_pos = []
    r_can = []
    obs_re_grid = copy.deepcopy(obs_grid)  # obs_re_grid: obstacles grid info after rearranged.
    obs_re_pos = copy.deepcopy(obs_pos)  # obs_re_pos: obstacles pos info after rearranged.
    obs_re_r = copy.deepcopy(obs_r)  # obs_re_r: obstacles radius info after rearranged
    for i in range(len(rearrange_order)-1):
        ore_grid.append(obs_grid[rearrange_order[i]])
        ore_pos.append(obs_pos[rearrange_order[i]])
        CLF.del_box_client(obstacle_name[rearrange_order[i]])
        r_can.append(obstacle_info[rearrange_order[i]][2][1] * 0.5)

    for i in range(len(rearrange_order) - 1):
        obs_re_grid.remove(obs_grid[rearrange_order[i]])
        obs_re_pos.remove(obs_pos[rearrange_order[i]])
        obs_re_r.remove(obs_r[rearrange_order[i]])

    print "\ntarget grid", tar_grid
    print "obstacle grid list", obs_grid
    print "target, obstacle radius list", tar_r, obs_r
    print "rearrange obstacle grid", ore_grid
    print "rearrange obstacle pos", ore_pos
    print "obstacle grid after rearranged", obs_re_grid
    print "obstacle pos after rearranged", obs_re_pos

    grid_ori = copy.deepcopy(grid_acc)
    for i in range(len(obs_grid)):
        grid_ori = CUF.obstacle_circle(grid_ori, [int(obs_grid[i][0]), int(obs_grid[i][1]), obs_r[i]], 2)
    grid_ori = CUF.obstacle_circle(grid_ori, [tar_grid[0], tar_grid[1], tar_r[0]], 4)  # target

    grid_del = copy.deepcopy(grid_acc)
    for i in range(len(obs_re_grid)):
        grid_del = CUF.obstacle_circle(grid_del, [int(obs_re_grid[i][0]), int(obs_re_grid[i][1]), obs_re_r[i]], 2)
    grid_del = CUF.obstacle_circle(grid_del, [tar_grid[0], tar_grid[1], tar_r[0]], 4)  # target

    # '''
    #     ===============================
    #     ====== grid setting ends!======
    #     ===============================
    # '''
    start_al = time.time()

    bt_num = 10
    trial_num = 2500
    print "max r of the rearrange needed obstacles:", r_can, max(r_can)
    circle_r = max(r_can)+0.015
    print "candidates radius", circle_r

    # this part is for getting candidates
    for bt in range(bt_num):
        bt_circle = []
        can_cen_grid = []
        grid_can = copy.deepcopy(grid_del)  # get original scene from the grid_set
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
                can_cen_grid.append(empt_grid[pick_cen])
                for em in range(len(empt_grid)):
                    d_w = empt_grid[pick_cen][0] - empt_grid[em][0]
                    d_d = empt_grid[pick_cen][1] - empt_grid[em][1]
                    d_c = (d_w * d_w + d_d * d_d) ** 0.5 * GRID_SIZE
                    if d_c <= circle_r:
                        grid_can[empt_grid[em][0]][empt_grid[em][1]] = 3
                        grid_can[empt_grid[pick_cen][0]][empt_grid[pick_cen][1]] = 3
                        occu_grid.append([empt_grid[em][0], empt_grid[em][1]])
        bt_circle.append([can_cen_grid, grid_can])

    max_cir_num = []
    for i in range(len(bt_circle)):
        print(i, "bt c", len(bt_circle[i][0]))
        max_cir_num.append([len(bt_circle[i][0])])

    print(max_cir_num.index(max(max_cir_num)))
    max_trial = max_cir_num.index(max(max_cir_num))

    grid_max_can = copy.deepcopy(bt_circle[max_trial][1])
    can_cen_grid = bt_circle[max_trial][0]

    if len(can_cen_grid) > 0:
        print"\n========================\n"

    print "\ncheck if candidate occlude the target"
    can_pan = []
    for i in can_cen_grid:
        xi, yi = ws_zero_pos[0] + i[0] * GRID_SIZE, ws_zero_pos[1] + i[1] * GRID_SIZE
        vfh_t_pos = copy.deepcopy(tar_pos)
        vfh_o_pos = copy.deepcopy(obs_re_pos)
        vfh_o_pos.append([xi, yi])
        vfh_ob = len(vfh_o_pos)
        vfh_o_ori = []
        for ori_i in range(vfh_ob):
            vfh_o_ori.append([0.0, 0.0, 0.0])
        vfh_t_ori = [0.0, 0.0, 0.0]
        vfh_ret = influence(vfh_ob, vfh_t_pos, vfh_o_pos, rob_pos, d_max, eta=45)
        # print("vfh", vfh)
        if vfh_ret[3] == 0:
            print "\ncandidate", i, "occludes the target", vfh_t_pos
            print "candidate penalty"
            can_pan.append(i)
    for i in can_pan:
        can_cen_grid.remove(i)

    print "\ncheck if the candidate occlude the obstacle to remove"
    can_pan = []
    for i in can_cen_grid:
        for ore_posi in ore_pos:
            # print "working", i, ore_posi
            vfh_t_pos = copy.deepcopy(ore_posi)
            vfh_o_pos = copy.deepcopy(obs_re_pos)
            vfh_o_pos.append(copy.deepcopy(tar_pos))
            vfh_o_pos.append([ws_zero_pos[0] + i[0] * GRID_SIZE, ws_zero_pos[1] + i[1] * GRID_SIZE])
            vfh_ob = len(vfh_o_pos)
            vfh_o_ori = []
            for ori_i in range(vfh_ob):
                vfh_o_ori.append([0.0, 0.0, 0.0])
            vfh_t_ori = [0.0, 0.0, 0.0]
            vfh_ret = influence(vfh_ob, vfh_t_pos, vfh_o_pos, rob_pos, d_max, eta=45)
            # print("vfh", vfh_ret)
            if vfh_ret[3] == 0:
                print "\ncandidate", i, "occludes the obstacle to remove", ore_posi
                print "candidate penalty"
                if (i in can_pan) == 0:
                    can_pan.append(i)
    for i in can_pan:
        can_cen_grid.remove(i)

    print "\ncheck if the candidate cannot be rearranged"
    can_pan = []
    for i in can_cen_grid:
        xi, yi = i
        vfh_t_pos = [ws_zero_pos[0] + i[0] * GRID_SIZE, ws_zero_pos[1] + i[1] * GRID_SIZE]
        vfh_o_pos = copy.deepcopy(obs_re_pos)
        vfh_o_pos.append(copy.deepcopy(tar_pos))
        vfh_ob = len(vfh_o_pos)
        vfh_ret = influence(vfh_ob, vfh_t_pos, vfh_o_pos, rob_pos, d_max, eta=45)
        # print("vfh", vfh_ret)
        if vfh_ret[3] == 0:
            print "\ncandidate", i, "can not be rearranged"
            print "candidate penalty"
            if (i in can_pan) == 0:
                can_pan.append(i)
    for i in can_pan:
        can_cen_grid.remove(i)

    grid_val_can_vfh = copy.deepcopy(grid_del)
    for i in can_cen_grid:
        xi, yi = i
        grid_val_can_vfh = CUF.obstacle_circle(grid_val_can_vfh, [xi, yi, 0.03], 3)  # target

    print "\ncurrent candidates", can_cen_grid, "\n"

    print "\n==============================="
    print "  check with rviz for the traj"
    print "==============================="
    print "\nbefore checking OCC, check the trajectory for candidates"
    can_pan = []
    for ci in can_cen_grid:

        xi, yi = ws_zero_pos[0] + ci[0] * GRID_SIZE, ws_zero_pos[1] + ci[1] * GRID_SIZE

        f_check_pos = [obstacle_info[0][0][0], -yi, xi - 0.17]
        f_check_ori = [-0.00145713772037, -0.998970756926, 0.0364956710831,  0.0268955302573]

        x1, y1 = ws_zero_pos[0] + ore_grid[0][0] * GRID_SIZE, ws_zero_pos[1] + ore_grid[0][1] * GRID_SIZE
        x2, y2 = ws_zero_pos[0] + ore_grid[1][0] * GRID_SIZE, ws_zero_pos[1] + ore_grid[1][1] * GRID_SIZE
        CLF.add_box_client('ore1', [obstacle_info[0][0][0], -y1, x1], [-0.707, 0.0, -0.707, 0.0], [0.06, 0.06, 0.2], 'red')
        CLF.add_box_client('ore2', [obstacle_info[0][0][0], -y2, x2], [-0.707, 0.0, -0.707, 0.0], [0.06, 0.06, 0.2], 'red')

        CLF.add_box_client('can_check', [obstacle_info[0][0][0], -yi, xi], [-0.707, 0.0, -0.707, 0.0], [0.06, 0.06, 0.2], 'pink')
        fes_ret = CLF.feasible_check_client('arm', f_check_pos, f_check_ori)
        if fes_ret == 0:
            print "candidate", ci, "can not be rearranged\n"
            if (ci in can_pan) == 0:
                can_pan.append(ci)
        else:
            print "candidate", ci, "OK\n"
            time.sleep(0.0001)
    CLF.del_box_client('can_check')
    CLF.del_box_client('ore1')
    CLF.del_box_client('ore2')
    for i in can_pan:
        can_cen_grid.remove(i)

    print "\nbefore checking OCC, check if the candidate occludes the target"
    can_pan = []
    for ci in can_cen_grid:

        xi, yi = ws_zero_pos[0] + ci[0] * GRID_SIZE, ws_zero_pos[1] + ci[1] * GRID_SIZE
        # add box from candidate grid info
        CLF.add_box_client('can_check', [obstacle_info[0][0][0], -yi, xi], [-0.707, 0.0, -0.707, 0.0], [0.06, 0.06, 0.2], 'pink')

        f_check_pos = [target_info[0][0][0], target_info[0][0][1], target_info[0][0][2] - 0.17]
        f_check_ori = [-0.00145713772037, -0.998970756926, 0.0364956710831,  0.0268955302573]
        fes_ret = CLF.feasible_check_client('arm', f_check_pos, f_check_ori)
        if fes_ret == 0:
            print "candidate", ci, "occludes the target\n"
            if (ci in can_pan) == 0:
                can_pan.append(ci)
        else:
            print "candidate", ci, "OK\n"
            time.sleep(0.0001)
    CLF.del_box_client('can_check')
    for i in can_pan:
        can_cen_grid.remove(i)

    grid_val_can_rviz = copy.deepcopy(grid_del)
    for i in can_cen_grid:
        xi, yi = i
        grid_val_can_rviz = CUF.obstacle_circle(grid_val_can_rviz, [xi, yi, 0.03], 3)  # target

    print "\n==============================="
    print "  check OCC for the candidates "
    print "==============================="
    if __name__ == '__main__':
        if len(can_cen_grid) >= len(ore_grid):
            print "\n Number of candidates are enough for rearrangement"
            can_occ_zero = []
            for cc in can_cen_grid:
                occ = 0
                xi, yi = ws_zero_pos[0] + cc[0] * GRID_SIZE, ws_zero_pos[1] + cc[1] * GRID_SIZE
                print "\ncheck", cc, "occludes"
                CLF.add_box_client('can_check', [obstacle_info[0][0][0], -yi, xi], [-0.707, 0.0, -0.707, 0.0], [0.06, 0.06, 0.2], 'pink')
                for ci in can_cen_grid:
                    if cc != ci:
                        print ci
                        xi, yi = ws_zero_pos[0] + ci[0] * GRID_SIZE, ws_zero_pos[1] + ci[1] * GRID_SIZE

                        f_check_pos = [obstacle_info[0][0][0], -yi, xi - 0.17]
                        f_check_ori = [-0.00145713772037, -0.998970756926, 0.0364956710831,  0.0268955302573]
                        fes_ret = CLF.feasible_check_client('arm', f_check_pos, f_check_ori)
                        if fes_ret == 0:
                            print cc, "occludes", ci
                            print "increase occ"
                            if (ci in can_pan) == 0:
                                occ = occ + 1
                                break

                print cc, "has occ", occ
                if occ == 0:
                    print cc, "has occ == 0"
                    can_occ_zero.append(cc)

            CLF.del_box_client('can_check')

            print can_occ_zero, "theses are occ = 0 points"
            grid_can_occ_zero = copy.deepcopy(grid_del)
            for i in can_occ_zero:
                xi, yi = i
                grid_can_occ_zero = CUF.obstacle_circle(grid_can_occ_zero, [xi, yi, 0.03], 3)  # target
        else: #len(can_cen_grid) < len(ore_grid):
            print "\n==============================="
            print "we need to remove more obstacles"
            print "==============================="

            # step 1. find accessible obstacles
            obs_re_acc_grid = []
            for oi in obs_re_grid:
                xi, yi = ws_zero_pos[0] + oi[0] * GRID_SIZE, ws_zero_pos[1] + oi[1] * GRID_SIZE

                f_check_pos = [obstacle_info[0][0][0], -yi, xi - 0.17]
                f_check_ori = [-0.00145713772037, -0.998970756926, 0.0364956710831, 0.0268955302573]
                fes_ret = CLF.feasible_check_client('arm', f_check_pos, f_check_ori)
                if fes_ret == 1:
                    print "can acc to", oi
                    obs_re_acc_grid.append(oi)
            print "can rearrange", obs_re_acc_grid

            # step 2. Calculate the possible occ zero candidates that we can get,
            #         if we remove more => can_occ_zero_gain
            can_occ_zero_gain = 0
            for i in range(obs_re_acc_grid):
                print i


    CUF.draw_grid_info(grid_ori)  # grid map with all obstacls
    CUF.draw_grid_info(grid_del)  # grid map with deleted obstacles after rearranged
    CUF.draw_grid_info(grid_max_can)  # grid map with maximum number of candidates
    CUF.draw_grid_info(grid_val_can_vfh)  # grid map after considering acc-possibility with vfh only
    CUF.draw_grid_info(grid_val_can_rviz)  # grid map after considering acc-possibility with rviz
    CUF.draw_grid_info(grid_can_occ_zero)  # grid map after considering acc-possibility with rviz
    plt.show()
