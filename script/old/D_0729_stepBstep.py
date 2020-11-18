import numpy as np
import matplotlib.pyplot as plt
import copy
import time

from candidate_info import CandidateInfo as CI
from VFHplus_change_radius import influence
from tree_making_no_plot import tree_making as TM_noplot
from tree_making_plot import tree_making as TM_plot
import S_custom_function as CUF

GRID_SIZE = 0.01
G2P_SIZE = 100
'''
type of grid
grid_acc: actual grid
grid_can: actual grid + candidate center

value of grid
0: empty grid
1: occupied grid that you cannot place an object
2: center of obstacles on the robot's work space
3: area that obstacles occludes the work space
4: center of candidates on the robot's work space
5: area that candidates occludes the work space
6: center of the target object on the robot's work space
7: area that the target object occludes the work space
'''


def getEmpOcc(grid_list):
    emp_g = []
    occ_g = []
    for wi in range(np.shape(grid_list)[0]):
        for di in range(np.shape(grid_list)[1]):
            if grid_list[wi][di] == 0:
                emp_g.append([wi, di])
            else:
                occ_g.append([wi, di])
    return emp_g, occ_g


def place_circle_object_ig(grid_list, obj_r, obj_type):
    emp_g, occ_g = getEmpOcc(grid_list)
    while 1:
        # This part is for checking the occlusion
        ran_c = np.random.randint(0, len(emp_g))
        empty_check = 0
        # print(ran_c)
        for oc in range(len(occ_g)):
            d_w = emp_g[ran_c][0] - occ_g[oc][0]
            d_d = emp_g[ran_c][1] - occ_g[oc][1]
            d_c = (d_w * d_w + d_d * d_d) ** 0.5 * GRID_SIZE
            if d_c <= obj_r + 0.01:
                empty_check = 1
        # This part is to occlude the empty grid to given grid type
        if empty_check == 0:
            for em in range(len(emp_g)):
                d_w = emp_g[ran_c][0] - emp_g[em][0]
                d_d = emp_g[ran_c][1] - emp_g[em][1]
                d_c = (d_w * d_w + d_d * d_d) ** 0.5 * GRID_SIZE
                if d_c <= obj_r:
                    grid_list[emp_g[em][0]][emp_g[em][1]] = obj_type
                    occ_g.append([emp_g[em][0], emp_g[em][1]])
            set_grid = emp_g[ran_c]
            return grid_list, set_grid


def mark_edge_grid(grid_list):
    w, d = np.shape(grid_list)
    grid_list[0], grid_list[w - 1] = 1, 1
    for i in range(w):
        grid_list[i][0] = 1
        grid_list[i][d - 1] = 1
    return grid_list


def draw_grid_info(input_grid_info):
    new_fig = plt.figure()
    for w in range(np.shape(input_grid_info)[0]):
        for d in range(np.shape(input_grid_info)[1]):
            # if input_grid_info[w][d] == 0:
            #     plt.scatter(w, d, c='gray', alpha=0.2)
            if input_grid_info[w][d] == 1:
                plt.scatter(w, d, c='black', alpha=0.2)
            elif input_grid_info[w][d] == 2:
                plt.scatter(w, d, c='red', alpha=0.2)
            elif input_grid_info[w][d] == 3:
                plt.scatter(w, d, c='pink', alpha=0.2)
            elif input_grid_info[w][d] == 4:
                plt.scatter(w, d, c='limegreen', alpha=0.2)
    plt.axis('equal')
    new_fig.show()


def draw_pos_info(obs_pos_in, tar_pos_in, rob_pos_in):
    new_fig = plt.figure()
    for i in obs_pos_in:
        plt.scatter(i[0], i[1], c='red', alpha=0.8)

    plt.scatter(tar_pos_in[0], tar_pos_in[1], c='limegreen', alpha=0.8)
    plt.scatter(rob_pos_in[0], rob_pos_in[1], c='gray', alpha=0.8)
    plt.axis('equal')
    new_fig.show()


def obstacle_circle(input_grid_info, circle_xyr, grid_num):
    for w in range(np.shape(input_grid_info)[0]):
        for d in range(np.shape(input_grid_info)[1]):
            if input_grid_info[w][d] == 0:
                d_w = w - circle_xyr[0]
                d_d = d - circle_xyr[1]
                d_c = (d_w * d_w + d_d * d_d) ** 0.5 * GRID_SIZE
                if d_c <= circle_xyr[2]:
                    input_grid_info[w][d] = grid_num
    out_grid_info = input_grid_info
    return out_grid_info


def get_obstacle_re(ob, target_ori, obs_pos_in, Body_position, d_max, eta):
    obstacle_rearr = []
    obs_pos = copy.deepcopy(obs_pos_in)
    vfh, km = influence(ob, target_ori, obs_pos, Body_position, d_max, eta)
    influence()

    if vfh == 1:
        # print("no need to rearrange")
        return 0
    while 1:
        ob = len(obs_pos)
        vfh, km = influence(ob, target_ori, obs_pos, Body_position, d_max)
        print("km", km, "to remove", obs_pos[km])
        print("ob :", ob)
        print("target_ori :", target_ori)
        print("obs_pos :", obs_pos)
        print("Body_position :", Body_position, "\n")
        if vfh == 1:
            # obstacle_rearr.append(obs_pos[km[0][1]])
            # obstacle_rearr.append(target_ori)
            # print("find way to rearrange \n list is:", obstacle_rearr)
            return obstacle_rearr
        elif vfh == 0:
            # print("we have to rearrange:", obs_pos[km[0][1]])
            obstacle_rearr.append([obs_pos[km], km])
            target_ori = obs_pos[km]
            obs_pos.remove(obs_pos[km])


if __name__ == '__main__':

    ws_w, ws_d = 50, 80  # get table size in the v-rep
    ws_cen = [0.75, 0.00]
    rob_pos = [0.0, 0.0]
    OBJ_R = 0.035

    ws_zero = [round(ws_cen[0] - ws_w * GRID_SIZE * 0.5, 2), round(ws_cen[1] - ws_d * GRID_SIZE * 0.5, 2)]

    ws_side = []
    ws_side.append(
        [ws_cen[0] - ws_w * GRID_SIZE * 0.5, ws_cen[1] - ws_d * GRID_SIZE * 0.5 - OBJ_R])  # left low point
    ws_side.append([ws_cen[0] + ws_w * GRID_SIZE * 0.5 + OBJ_R,
                    ws_cen[1] - ws_d * GRID_SIZE * 0.5 - OBJ_R])  # right low point
    ws_side.append([ws_cen[0] + ws_w * GRID_SIZE * 0.5 + OBJ_R,
                    ws_cen[1] + ws_d * GRID_SIZE * 0.5 + OBJ_R])  # right high point
    ws_side.append(
        [ws_cen[0] - ws_w * GRID_SIZE * 0.5, ws_cen[1] + ws_d * GRID_SIZE * 0.5 + OBJ_R])  # left high point

    obs_wall = []
    obs_wall.extend(CUF.linspace2D(ws_side[0], ws_side[1], round(ws_w * GRID_SIZE / OBJ_R)))
    obs_wall.extend(CUF.linspace2D(ws_side[1], ws_side[2], round(ws_d * GRID_SIZE / OBJ_R)))
    obs_wall.extend(CUF.linspace2D(ws_side[2], ws_side[3], round(ws_w * GRID_SIZE / OBJ_R)))

    ws_zero = [round(ws_cen[0]-ws_w*GRID_SIZE*0.5, 2), round(ws_cen[1]-ws_d*GRID_SIZE*0.5, 2)]
    print("ws zero point", ws_zero)
    # GRID_SIZE = 0.01

    grid_acc = np.zeros([ws_w, ws_d])
    grid_acc = mark_edge_grid(grid_acc)
    obs_r = [0.035, 0.035, 0.035, 0.035, 0.035, 0.035, 0.035, 0.035, 0.035, 0.035]
    tar_r = 0.035
    obs_n = len(obs_r)
    ore_grid = []

    while 1:
        obs_grid = []
        tar_grid = []
        grid_tmp = copy.deepcopy(grid_acc)

        for ri in obs_r:
            grid_tmp, obs_center_tmp = CUF.place_circle_object_ig(grid_tmp, ri, 2)
            obs_grid.append(obs_center_tmp)
        grid_tmp, tar_tmp = CUF.place_circle_object_ig(grid_tmp, tar_r, 4)
        tar_grid = copy.deepcopy(tar_tmp)

        obs_pos = []
        for i in obs_grid:
            xi, yi = i
            obs_pos.append([round(xi * GRID_SIZE+ws_zero[0], 2), round(yi * GRID_SIZE+ws_zero[1], 2)])
        tar_pos = [round(tar_grid[0] * GRID_SIZE+ws_zero[0], 2), round(tar_grid[1] * GRID_SIZE+ws_zero[1], 2)]   # target object!
        tar_pos_tmp = copy.deepcopy(tar_pos)
        '''
            This part is to delete the obstacles and get the candidates!
        '''
        d_max = 2.0
        eta = 45
        tm_tar_pos = copy.deepcopy(tar_pos)
        tm_tar_ori = [0.0, 0.0, 0.0]
        tm_obs_pos = copy.deepcopy(obs_pos)
        tm_obs_pos.extend(obs_wall)
        ob = len(tm_obs_pos)
        tm_obs_ori = []
        for i in range(ob):
            tm_obs_ori.append([0.0, 0.0, 0.0])

        ore_order = TM_noplot(ob, tm_tar_pos, tm_tar_ori, tm_obs_pos, tm_obs_ori, rob_pos, rob_pos, d_max)

        if len(ore_order) > 1:
            print("environment setting OK")
            print("target", tar_pos)
            print("obstacles", obs_pos)
            print("remove", ore_order, "th obstacles")
            tm_tar_pos = copy.deepcopy(tar_pos)
            tm_obs_pos = copy.deepcopy(obs_pos)
            tm_obs_pos.extend(obs_wall)
            ob = len(tm_obs_pos)
            ore_order = TM_plot(ob, tm_tar_pos, tm_tar_ori, tm_obs_pos, tm_obs_ori, rob_pos, rob_pos, d_max)

            # check if the obstacles are rearranged then target is reachable
            tm_tar_pos = copy.deepcopy(tar_pos)
            tm_obs_pos = copy.deepcopy(obs_pos)
            tm_ore_pos = []
            for i in ore_order:
                if i != 'T':
                    tm_ore_pos.append(obs_pos[i])
            for i in ore_order:
                if i != 'T':
                    tm_obs_pos.remove(obs_pos[i])
            tm_obs_pos.extend(obs_wall)
            ob = len(tm_obs_pos)
            tmp_order = TM_noplot(ob, tm_tar_pos, tm_tar_ori, tm_obs_pos, tm_obs_ori, rob_pos, rob_pos, d_max)
            if len(tmp_order) == 1:
                break
    '''
    with out additional rearrangement
    '''
    if len(ore_order) > 1:
        ore_grid = []
        ore_pos = []
        ore_r = []

        obs_re_grid = copy.deepcopy(obs_grid)
        obs_re_pos = copy.deepcopy(obs_pos)
        obs_re_r = copy.deepcopy(obs_r)

        for i in ore_order:
            if i != 'T':
                ore_grid.append(obs_re_grid[i])
                ore_pos.append(obs_re_pos[i])
                ore_r.append(obs_re_r[i])
        for i in ore_order:
            if i != 'T':
                obs_re_grid.remove(obs_grid[i])
                obs_re_pos.remove(obs_pos[i])
                obs_re_r.remove(obs_r[i])

        grid_ori = copy.deepcopy(grid_acc)
        for i in range(len(obs_r)):
            grid_ori = CUF.obstacle_circle(grid_ori, [round(obs_grid[i][0], 2), round(obs_grid[i][1], 2), obs_r[i]], 2)
        grid_ori = CUF.obstacle_circle(grid_ori, [tar_grid[0], tar_grid[1], tar_r], 4)  # target

        grid_del = copy.deepcopy(grid_acc)
        for i in range(len(obs_re_r)):
            grid_del = CUF.obstacle_circle(grid_del, [round(obs_re_grid[i][0], 2), round(obs_re_grid[i][1], 2), obs_re_r[i]], 2)
        grid_del = CUF.obstacle_circle(grid_del, [tar_grid[0], tar_grid[1], tar_r], 4)  # target

        # get maximum number of candidates!
        start_al = time.time()
        bt_num = 1
        trial_num = 1000
        bt_circle = []
        can_grid = []
        circle_r = max(ore_r)+0.005
        for bt in range(bt_num):
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

        print(max_cir_num.index(max(max_cir_num)))
        max_trial = max_cir_num.index(max(max_cir_num))

        grid_max_can = copy.deepcopy(bt_circle[max_trial][1])
        can_grid = bt_circle[max_trial][0]
        can_info = []
        for i in range(len(can_grid)):
            can_info.append(CI())
            can_info[i].pos = can_grid[i]

            # print "candidate position :", can_info[i].pos

        print("\nCheck if candidate blocks the target")
        for i in range(len(can_info)):
            xi, yi = can_info[i].pos
            vfh_obs_pos = copy.deepcopy(obs_re_pos)
            vfh_obs_pos.append([ws_zero[0] + xi * GRID_SIZE, ws_zero[1] + yi * GRID_SIZE])
            vfh_obs_pos.extend(obs_wall)
            vfh_tar_pos = copy.deepcopy(tar_pos)
            ob = len(vfh_obs_pos)
            vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, rob_pos, d_max, eta)
            if vfh[3] == 0:
                can_info[i].BT = 1  # BT == 1 : The candidate blocks the target.
            else:                   # BT == 0 : The candidate does not block the target.
                can_info[i].BT = 0

        #
        print("\nCheck if the candidate is accessible.")
        for i in range(len(can_info)):
            xi, yi = can_info[i].pos
            vfh_tar_pos = [ws_zero[0] + xi * GRID_SIZE, ws_zero[1] + yi * GRID_SIZE]
            vfh_obs_pos = copy.deepcopy(obs_pos)
            vfh_obs_pos.append(tar_pos)
            vfh_obs_pos.extend(obs_wall)
            ob = len(vfh_obs_pos)
            vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, rob_pos, d_max, eta)
            if vfh[3] == 0:         # A == 1 : The candidate is accessible.
                can_info[i].A = 0   # A == 0 : The candidate is not accessible.
                tm_tar_pos = copy.deepcopy(vfh_tar_pos)
                tm_tar_ori = [0.0, 0.0, 0.0]
                tm_obs_pos = copy.deepcopy(obs_pos)
                tm_obs_pos.extend(obs_wall)
                ob = len(tm_obs_pos)
                tm_obs_ori = []
                for obs_ori_i in range(ob):
                    tm_obs_ori.append([0.0, 0.0, 0.0])
                ore_order = TM_noplot(ob, tm_tar_pos, tm_tar_ori, tm_obs_pos, tm_obs_ori, rob_pos, rob_pos, d_max)
                ore_order.pop()  # The last order is always the target so we need to pop the last element.
                can_info[i].ORC = ore_order
            else:                   #
                can_info[i].A = 1
        for i in range(20):
            can_info[i].show()
        #
        # '''
        # START STEP BY STEP
        # '''
        # can_step = []
        # for step_i in range(len(ore_grid)):  # We removed candidates that occludes the target so '-1' is fine.
        #     #  step1 : get available candidates in this step
        #     print "\nstep : 1-", step_i, "starts!"
        #     tmp_can_pan = []
        #     tmp_can_grid = copy.deepcopy(can_grid)
        #     for c_i in tmp_can_grid:
        #         if step_i == len(ore_grid)-1:  # We need to set vfh_tar_pos to tar_pos
        #             vfh_tar_pos = copy.deepcopy(tar_pos)
        #         else:
        #             vfh_tar_pos = [ws_zero[0] + ore_grid[step_i + 1][0] * GRID_SIZE, ws_zero[1] + ore_grid[step_i + 1][1] * GRID_SIZE]
        #         vfh_obs_pos = copy.deepcopy(obs_re_pos)
        #         vfh_obs_pos.append([ws_zero[0] + c_i[0] * GRID_SIZE, ws_zero[1] + c_i[1] * GRID_SIZE])
        #         vfh_obs_pos.extend(obs_wall)
        #         ob = len(vfh_obs_pos)
        #         vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, rob_pos, d_max, eta)
        #         if vfh[3] == 0:
        #             print("candidate", c_i, "occludes", ore_grid[step_i + 1])
        #             print("candidate penalty")
        #             if c_i not in tmp_can_pan:
        #                 tmp_can_pan.append(c_i)
        #     for i in tmp_can_pan:
        #         tmp_can_grid.remove(c_i)
        #     can_step.append(tmp_can_grid)
        #     if 1:  # 1: show the fig, 0: pass the fig
        #         grid_step_can = copy.deepcopy(grid_del)
        #         for i in tmp_can_grid:
        #             xi, yi = i
        #             grid_step_can = CUF.obstacle_circle(grid_step_can, [xi, yi, 0.04], 3)  # target
        #         CUF.draw_grid_info(grid_step_can)
        #         for i in tmp_can_pan:
        #             plt.scatter(i[0], i[1], c='gray')
        #             plt.text(i[0], i[1], 'occludes'+str(ore_grid[step_i + 1]), ha='center')
        #         plt.title('step'+str(step_i))
        #     print "Before we have,", len(can_grid), "\nAfter we have", len(tmp_can_grid)
        #
        # # #  step2 : get occ of the candidates
        # #     for step_i in range(len(ore_grid) - 1):  # We removed candidates that occludes the target so '-1' is fine.
        # #         print "\nstep : 2-", step_i, "starts!"
        # #         tmp_can_pan = []
        # #         tmp_can_grid = copy.deepcopy(can_grid)
        # #         for c_i in tmp_can_grid:
        # #             vfh_tar_pos = [ws_zero[0] + ore_grid[step_i + 1][0] * GRID_SIZE,
        # #                            ws_zero[1] + ore_grid[step_i + 1][1] * GRID_SIZE]
        # #             vfh_obs_pos = copy.deepcopy(obs_re_pos)
        # #             vfh_obs_pos.append([ws_zero[0] + c_i[0] * GRID_SIZE, ws_zero[1] + c_i[1] * GRID_SIZE])
        # #             vfh_obs_pos.extend(obs_wall)
        # #             ob = len(vfh_obs_pos)
        # #             vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, rob_pos, d_max, eta)
        # #             if vfh[3] == 0:
        # #                 print("candidate", c_i, "occludes", ore_grid[step_i + 1])
        # #                 print("candidate penalty")
        # #                 if c_i not in tmp_can_pan:
        # #                     tmp_can_pan.append(c_i)
        # #         for i in tmp_can_pan:
        # #             tmp_can_grid.remove(c_i)
        # #         can_step.append(tmp_can_grid)
        # #         if 1:  # 1: show the fig, 0: pass the fig
        # #             grid_step_can = copy.deepcopy(grid_del)
        # #             for i in tmp_can_grid:
        # #                 xi, yi = i
        # #                 grid_step_can = CUF.obstacle_circle(grid_step_can, [xi, yi, 0.04], 3)  # target
        # #             CUF.draw_grid_info(grid_step_can)
        # #             for i in tmp_can_pan:
        # #                 plt.scatter(i[0], i[1], c='gray')
        # #                 plt.text(i[0], i[1], 'occludes' + str(ore_grid[step_i + 1]), ha='center')
        # #             plt.title('step' + str(step_i))
        # #         print "Before we have,", len(can_grid), "\nAfter we have", len(tmp_can_grid)
        #
        # # print"\nCheck if the candidate occlude the obstacle to remove or target."
        # can_pan = []
        # for i in can_grid:
        #     for oi in ore_grid:
        #         xi, yi = i
        #         vfh_tar_pos = [ws_zero[0] + oi[0] * GRID_SIZE, ws_zero[1] + oi[1] * GRID_SIZE]
        #         vfh_obs_pos = copy.deepcopy(obs_re_pos)
        #         vfh_obs_pos.append([ws_zero[0] + xi * GRID_SIZE, ws_zero[1] + yi * GRID_SIZE])
        #         vfh_obs_pos.extend(obs_wall)
        #         ob = len(vfh_obs_pos)
        #         vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, rob_pos, d_max, eta)
        #         if vfh[3] == 0:
        #             if i not in can_pan:
        #                 print("candidate", i, "occludes the obstacle to remove", oi)
        #                 print("candidate penalty")
        #                 can_pan.append(i)
        # for i in can_pan:
        #     can_grid.remove(i)
        # print "\nAfter we have", len(can_grid)
        #
        # grid_val_can = copy.deepcopy(grid_del)
        # for i in can_grid:
        #     xi, yi = i
        #     grid_val_can = CUF.obstacle_circle(grid_val_can, [xi, yi, 0.04], 3)  # target
        # # print("\n Check the occ for the candidates")
        # can_occ_zero = []
        # for co in can_grid:  # assume co as an obstacle
        #     occ = 0
        #     for ci in can_grid:  # assume ci as the target position
        #         if co != ci:
        #             vfh_tar_pos = [ws_zero[0] + ci[0] * GRID_SIZE, ws_zero[1] + ci[1] * GRID_SIZE]
        #             vfh_obs_pos = copy.deepcopy(obs_re_pos)
        #             vfh_obs_pos.append([ws_zero[0] + co[0] * GRID_SIZE, ws_zero[1] + co[1] * GRID_SIZE])
        #             vfh_obs_pos.append(tar_pos)
        #             vfh_obs_pos.extend(obs_wall)
        #             ob = len(vfh_obs_pos)
        #             vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, rob_pos, d_max, eta)
        #             if vfh[3] == 0:
        #                 occ = occ + 1
        #     if occ == 0:
        #         can_occ_zero.append(co)
        #
        # print("occ zero candidates are", can_occ_zero)
        # grid_occ_zero = copy.deepcopy(grid_del)
        # for i in can_occ_zero:
        #     xi, yi = i
        #     grid_occ_zero = CUF.obstacle_circle(grid_occ_zero, [xi, yi, 0.04], 3)  # target
        #
        # #  Draw stuffs!!
        # draw_grid_info(grid_ori)
        # draw_grid_info(grid_del)
        # draw_grid_info(grid_max_can)
        # if len(can_grid) == 0:
        #     print("There are no candidates!")
        # else:
        #     draw_grid_info(grid_val_can)
        #     draw_grid_info(grid_occ_zero)
        # print("with rearranging", len(ore_pos), "we have", len(can_grid), "candidates with", len(can_occ_zero), "occ = zero points")
        # plt.show()

    # with additional rearrangement
    # step 1. find accessible obstacles
    # for i in range(len(obs_re_grid)):
    #     obs_re_acc_index = []
    #     tmp_index = obs_grid.index(obs_re_grid[i])
    #     obs_re_acc_index.append(tmp_index)
    #
    #     for obs_add_rei in obs_re_acc_index:
    #         ore_order.append(obs_add_rei)
    #         if len(ore_order) > 3:
    #             ore_grid = []
    #             ore_pos = []
    #             ore_r = []
    #
    #             obs_re_grid = copy.deepcopy(obs_grid)
    #             obs_re_pos = copy.deepcopy(obs_pos)
    #             obs_re_r = copy.deepcopy(obs_r)
    #
    #             for i in ore_order:
    #                 if i != 'T':
    #                     ore_grid.append(obs_re_grid[i])
    #                     ore_pos.append(obs_re_pos[i])
    #                     ore_r.append(obs_re_r[i])
    #             for i in ore_order:
    #                 if i != 'T':
    #                     obs_re_grid.remove(obs_grid[i])
    #                     obs_re_pos.remove(obs_pos[i])
    #                     obs_re_r.remove(obs_r[i])
    #
    #             grid_ori = copy.deepcopy(grid_acc)
    #             for i in range(len(obs_r)):
    #                 grid_ori = CUF.obstacle_circle(grid_ori, [round(obs_grid[i][0], 2), round(obs_grid[i][1], 2), obs_r[i]], 2)
    #             grid_ori = CUF.obstacle_circle(grid_ori, [tar_grid[0], tar_grid[1], tar_r], 4)  # target
    #
    #             grid_del = copy.deepcopy(grid_acc)
    #             for i in range(len(obs_re_r)):
    #                 grid_del = CUF.obstacle_circle(grid_del, [round(obs_re_grid[i][0], 2), round(obs_re_grid[i][1], 2), obs_re_r[i]], 2)
    #             grid_del = CUF.obstacle_circle(grid_del, [tar_grid[0], tar_grid[1], tar_r], 4)  # target
    #
    #             bt_num = 1
    #             trial_num = 1000
    #             bt_circle = []
    #             can_grid = []
    #             circle_r = max(ore_r) + 0.005
    #             for bt in range(bt_num):
    #                 grid_can = copy.deepcopy(grid_del)  # get original scene from the grid_set
    #                 empt_grid, occu_grid = CUF.getEmpOcc(grid_can)
    #                 for i in range(trial_num):
    #                     pick_cen = np.random.randint(0, len(empt_grid))
    #                     check_sum = 0
    #                     for oc in range(len(occu_grid)):
    #                         d_w = empt_grid[pick_cen][0] - occu_grid[oc][0]
    #                         d_d = empt_grid[pick_cen][1] - occu_grid[oc][1]
    #                         d_c = (d_w * d_w + d_d * d_d) ** 0.5 * GRID_SIZE
    #                         if d_c <= circle_r:
    #                             check_sum = 1
    #
    #                     if check_sum == 0:
    #                         can_grid.append(empt_grid[pick_cen])
    #                         for em in range(len(empt_grid)):
    #                             d_w = empt_grid[pick_cen][0] - empt_grid[em][0]
    #                             d_d = empt_grid[pick_cen][1] - empt_grid[em][1]
    #                             d_c = (d_w * d_w + d_d * d_d) ** 0.5 * GRID_SIZE
    #                             if d_c <= circle_r:
    #                                 grid_can[empt_grid[em][0]][empt_grid[em][1]] = 3
    #                                 grid_can[empt_grid[pick_cen][0]][empt_grid[pick_cen][1]] = 3
    #                                 occu_grid.append([empt_grid[em][0], empt_grid[em][1]])
    #                 bt_circle.append([can_grid, grid_can])
    #
    #             max_cir_num = []
    #             for i in range(len(bt_circle)):
    #                 # print(i, "bt c", len(bt_circle[i][0]))
    #                 max_cir_num.append([len(bt_circle[i][0])])
    #
    #             max_trial = max_cir_num.index(max(max_cir_num))
    #
    #             grid_max_can = copy.deepcopy(bt_circle[max_trial][1])
    #             can_grid = bt_circle[max_trial][0]
    #
    #             # print("\ncheck if the candidate cannot be rearranged")
    #             can_pan = []
    #             for i in can_grid:
    #                 xi, yi = i
    #                 vfh_tar_pos = [ws_zero[0] + xi * GRID_SIZE, ws_zero[1] + yi * GRID_SIZE]
    #                 vfh_obs_pos = copy.deepcopy(obs_re_pos)
    #                 vfh_obs_pos.append(tar_pos)
    #                 vfh_obs_pos.extend(obs_wall)
    #                 ob = len(vfh_obs_pos)
    #                 vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, rob_pos, d_max, eta)
    #                 if vfh[3] == 0:
    #                     # print("candidate", xi, yi, "can not be rearranged")
    #                     # print("candidate penalty")
    #                     can_pan.append(i)
    #             for i in can_pan:
    #                 can_grid.remove(i)
    #
    #             # print("\ncheck if candidate occludes the target.")
    #             can_pan = []
    #             for i in can_grid:
    #                 xi, yi = i
    #                 vfh_obs_pos = copy.deepcopy(obs_re_pos)
    #                 vfh_obs_pos.append([ws_zero[0] + xi * GRID_SIZE, ws_zero[1] + yi * GRID_SIZE])
    #                 vfh_obs_pos.extend(obs_wall)
    #                 vfh_tar_pos = copy.deepcopy(tar_pos)
    #                 ob = len(vfh_obs_pos)
    #                 vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, rob_pos, d_max, eta)
    #                 # print("vfh", vfh)
    #                 if vfh[3] == 0:
    #                     # print("candidate", i, "occludes the target", vfh_tar_pos)
    #                     # print("candidate penalty")
    #                     can_pan.append(i)
    #             for i in can_pan:
    #                 can_grid.remove(i)
    #
    #             # print("\ncheck if the candidate occludes the obstacle.")
    #             can_pan = []
    #             for i in can_grid:
    #                 for oi in ore_grid:
    #                     xi, yi = i
    #                     vfh_tar_pos = [ws_zero[0] + oi[0] * GRID_SIZE, ws_zero[1] + oi[1] * GRID_SIZE]
    #                     vfh_obs_pos = copy.deepcopy(obs_re_pos)
    #                     vfh_obs_pos.append([ws_zero[0] + xi * GRID_SIZE, ws_zero[1] + yi * GRID_SIZE])
    #                     vfh_obs_pos.extend(obs_wall)
    #                     ob = len(vfh_obs_pos)
    #                     vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, rob_pos, d_max, eta)
    #                     if vfh[3] == 0:
    #                         if i in can_pan == 0:
    #                             # print("candidate", i, "occludes the obstacle to remove", oi)
    #                             # print("candidate penalty")
    #                             can_pan.append(i)
    #             for i in can_pan:
    #                 can_grid.remove(i)
    #
    #             grid_val_can = copy.deepcopy(grid_del)
    #             for i in can_grid:
    #                 xi, yi = i
    #                 grid_val_can = obstacle_circle(grid_val_can, [xi, yi, 0.04], 3)  # target
    #
    #             # print("\ncheck the occ for the candidates")
    #             can_occ_zero = []
    #             for co in can_grid:  # assume co as an obstacle
    #                 occ = 0
    #                 for ci in can_grid:  # assume ci as the target position
    #                     if co != ci:
    #                         vfh_tar_pos = [ws_zero[0] + ci[0] * GRID_SIZE, ws_zero[1] + ci[1] * GRID_SIZE]
    #                         vfh_obs_pos = copy.deepcopy(obs_re_pos)
    #                         vfh_obs_pos.append([ws_zero[0] + co[0] * GRID_SIZE, ws_zero[1] + co[1] * GRID_SIZE])
    #                         vfh_obs_pos.append(tar_pos)
    #                         vfh_obs_pos.extend(obs_wall)
    #                         ob = len(vfh_obs_pos)
    #                         vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, rob_pos, d_max, eta)
    #                         if vfh[3] == 0:
    #                             occ = occ + 1
    #                 if occ == 0:
    #                     can_occ_zero.append(co)
    #
    #             print("\nocc zero candidates are", can_occ_zero)
    #             grid_occ_zero = copy.deepcopy(grid_del)
    #             for i in can_occ_zero:
    #                 xi, yi = i
    #                 grid_occ_zero = obstacle_circle(grid_occ_zero, [xi, yi, 0.04], 3)  # target
    #
    #             # draw_grid_info(grid_ori)
    #             # draw_grid_info(grid_del)
    #             # draw_grid_info(grid_max_can)
    #             # if len(can_grid) == 0:
    #             #     print("There are no candidates!")
    #             # else:
    #             #     draw_grid_info(grid_val_can)
    #             #     draw_grid_info(grid_occ_zero)
    #
    #             print("with rearranging", len(ore_pos), "we have", len(can_grid), "candidates with", len(can_occ_zero), "occ = zero points")

    # plt.show()