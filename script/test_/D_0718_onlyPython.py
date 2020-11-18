import numpy as np
import matplotlib.pyplot as plt
import copy
import time
from JULY_06.M06.vfh_jw.VFHplus_change_radius import influence
from JULY_06.M06.vfh_jw.tree_making_noplot import tree_making as TM_noplot
from JULY_06.M06.vfh_jw.tree_making_plot import tree_making as TM_plot

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

    ws_zero = [round(ws_cen[0]-ws_w*GRID_SIZE*0.5, 2), round(ws_cen[1]-ws_d*GRID_SIZE*0.5, 2)]
    print("ws zero point", ws_zero)
    # GRID_SIZE = 0.01

    grid_acc = np.zeros([ws_w, ws_d])
    grid_acc = mark_edge_grid(grid_acc)
    obs_r = [0.035, 0.035, 0.035, 0.035, 0.035, 0.035, 0.035, 0.035, 0.035]
    tar_r = 0.035
    obs_n = len(obs_r)
    ore_grid = []

    while 1:
        obs_grid = []
        tar_grid = []
        grid_tmp = copy.deepcopy(grid_acc)

        for ri in obs_r:
            grid_tmp, obs_center_tmp = place_circle_object_ig(grid_tmp, ri, 2)
            obs_grid.append(obs_center_tmp)
        grid_tmp, tar_tmp = place_circle_object_ig(grid_tmp, tar_r, 4)
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
        ob = len(obs_grid)
        tm_tar_pos = copy.deepcopy(tar_pos)
        tm_tar_ori = [0.0, 0.0, 0.0]
        tm_obs_pos = copy.deepcopy(obs_pos)
        tm_obs_ori = []
        for i in range(ob):
            tm_obs_ori.append([0.0, 0.0, 0.0])

        ore_order = TM_noplot(ob, tm_tar_pos, tm_tar_ori, tm_obs_pos, tm_obs_ori, rob_pos, rob_pos, d_max)

        if len(ore_order) > 2:
            print("environment setting OK")
            print("target", tar_pos)
            print("obstacles", obs_pos)
            print("remove", ore_order, "th obstacles")
            TM_plot(ob, tm_tar_pos, tm_tar_ori, tm_obs_pos, tm_obs_ori, rob_pos, rob_pos, d_max)
            break
    # with out additional rearrangement
    if len(ore_order) > 2:
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
            grid_ori = obstacle_circle(grid_ori, [round(obs_grid[i][0], 2), round(obs_grid[i][1], 2), obs_r[i]], 2)
        grid_ori = obstacle_circle(grid_ori, [tar_grid[0], tar_grid[1], tar_r], 4)  # target

        grid_del = copy.deepcopy(grid_acc)
        for i in range(len(obs_re_r)):
            grid_del = obstacle_circle(grid_del, [round(obs_re_grid[i][0], 2), round(obs_re_grid[i][1], 2), obs_re_r[i]], 2)
        grid_del = obstacle_circle(grid_del, [tar_grid[0], tar_grid[1], tar_r], 4)  # target

        # draw_grid_info(grid_ori)
        # draw_grid_info(grid_del)
        # plt.show()
        ''' 
            ===============================
            ====== grid setting ends!======
            ===============================
        '''
        start_al = time.time()
        bt_num = 1
        trial_num = 1000
        bt_circle = []
        can_grid = []
        # print("max r of the rearrange needed obstacles:", ore_r, max(ore_r))
        circle_r = max(ore_r)+0.005
        # print("candidates radius", circle_r)
        for bt in range(bt_num):
            grid_can = copy.deepcopy(grid_del)  # get original scene from the grid_set
            empt_grid, occu_grid = getEmpOcc(grid_can)
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
            # print(i, "bt c", len(bt_circle[i][0]))
            max_cir_num.append([len(bt_circle[i][0])])

        print(max_cir_num.index(max(max_cir_num)))
        max_trial = max_cir_num.index(max(max_cir_num))

        grid_max_can = copy.deepcopy(bt_circle[max_trial][1])
        can_grid = bt_circle[max_trial][0]

        # print("\ncheck if the candidate cannot be rearranged")
        can_pan = []
        for i in can_grid:
            xi, yi = i
            vfh_tar_pos = [ws_zero[0] + xi * GRID_SIZE, ws_zero[1] + yi * GRID_SIZE]
            vfh_obs_pos = copy.deepcopy(obs_re_pos)
            vfh_obs_pos.append(tar_pos)
            ob = len(vfh_obs_pos)
            vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, rob_pos, d_max, eta)
            if vfh[3] == 0:
                # print("candidate", xi, yi, "can not be rearranged")
                # print("candidate penalty")
                can_pan.append(i)
        for i in can_pan:
            can_grid.remove(i)

        # print("\ncheck if candidate occlude the target")
        can_pan = []
        for i in can_grid:
            xi, yi = i
            vfh_obs_pos = copy.deepcopy(obs_re_pos)
            vfh_obs_pos.append([ws_zero[0] + xi * GRID_SIZE, ws_zero[1] + yi * GRID_SIZE])
            vfh_tar_pos = copy.deepcopy(tar_pos)
            ob = len(vfh_obs_pos)
            vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, rob_pos, d_max, eta)
            # print("vfh", vfh)
            if vfh[3] == 0:
                # print("candidate", i, "occludes the target", vfh_tar_pos)
                # print("candidate penalty")
                can_pan.append(i)
        for i in can_pan:
            can_grid.remove(i)

        # print("\ncheck if the candidate occlude the obstacle to remove")
        can_pan = []
        for i in can_grid:
            for oi in ore_grid:
                xi, yi = i
                vfh_tar_pos = [ws_zero[0] + oi[0] * GRID_SIZE, ws_zero[1] + oi[1] * GRID_SIZE]
                vfh_obs_pos = copy.deepcopy(obs_re_pos)
                vfh_obs_pos.append([ws_zero[0] + xi * GRID_SIZE, ws_zero[1] + yi * GRID_SIZE])
                ob = len(vfh_obs_pos)
                vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, rob_pos, d_max, eta)
                if vfh[3] == 0:
                    if i in can_pan == 0:
                        # print("candidate", i, "occludes the obstacle to remove", oi)
                        # print("candidate penalty")
                        can_pan.append(i)
        for i in can_pan:
            can_grid.remove(i)

        grid_val_can = copy.deepcopy(grid_del)
        for i in can_grid:
            xi, yi = i
            grid_val_can = obstacle_circle(grid_val_can, [xi, yi, 0.04], 3)  # target

        # print("\ncheck the occ for the candidates")
        can_occ_zero = []
        for co in can_grid: # assume co as an obstacle
            occ = 0
            for ci in can_grid: # assume ci as the target position
                if co != ci:
                    vfh_tar_pos = [ws_zero[0] + ci[0] * GRID_SIZE, ws_zero[1] + ci[1] * GRID_SIZE]
                    vfh_obs_pos = copy.deepcopy(obs_re_pos)
                    vfh_obs_pos.append([ws_zero[0] + co[0] * GRID_SIZE, ws_zero[1] + co[1] * GRID_SIZE])
                    vfh_obs_pos.append(tar_pos)
                    ob = len(vfh_obs_pos)
                    vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, rob_pos, d_max, eta)
                    if vfh[3] == 0:
                        occ = occ + 1
            if occ == 0:
                can_occ_zero.append(co)

        print("occ zero candidates are", can_occ_zero)
        grid_occ_zero = copy.deepcopy(grid_del)
        for i in can_occ_zero:
            xi, yi = i
            grid_occ_zero = obstacle_circle(grid_occ_zero, [xi, yi, 0.04], 3)  # target

        # draw_grid_info(grid_ori)
        # draw_grid_info(grid_del)
        # draw_grid_info(grid_max_can)
        # if len(can_grid) == 0:
            # print("There are no candidates!")
        # else:
        #     draw_grid_info(grid_val_can)
        #     draw_grid_info(grid_occ_zero)

        print("with rearranging", len(ore_pos), "we have", len(can_grid), "candidates with", len(can_occ_zero), "occ = zero points")

    # with additional rearrangement
    # step 1. find accessible obstacles
    for i in range(len(obs_re_grid)):
        obs_re_acc_index = []
        tmp_index = obs_grid.index(obs_re_grid[i])
        obs_re_acc_index.append(tmp_index)

        for obs_add_rei in obs_re_acc_index:
            ore_order.append(obs_add_rei)
            if len(ore_order) > 3:
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
                    grid_ori = obstacle_circle(grid_ori, [round(obs_grid[i][0], 2), round(obs_grid[i][1], 2), obs_r[i]], 2)
                grid_ori = obstacle_circle(grid_ori, [tar_grid[0], tar_grid[1], tar_r], 4)  # target

                grid_del = copy.deepcopy(grid_acc)
                for i in range(len(obs_re_r)):
                    grid_del = obstacle_circle(grid_del, [round(obs_re_grid[i][0], 2), round(obs_re_grid[i][1], 2), obs_re_r[i]], 2)
                grid_del = obstacle_circle(grid_del, [tar_grid[0], tar_grid[1], tar_r], 4)  # target

                bt_num = 1
                trial_num = 1000
                bt_circle = []
                can_grid = []
                circle_r = max(ore_r) + 0.005
                for bt in range(bt_num):
                    grid_can = copy.deepcopy(grid_del)  # get original scene from the grid_set
                    empt_grid, occu_grid = getEmpOcc(grid_can)
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
                    # print(i, "bt c", len(bt_circle[i][0]))
                    max_cir_num.append([len(bt_circle[i][0])])

                max_trial = max_cir_num.index(max(max_cir_num))

                grid_max_can = copy.deepcopy(bt_circle[max_trial][1])
                can_grid = bt_circle[max_trial][0]

                # print("\ncheck if the candidate cannot be rearranged")
                can_pan = []
                for i in can_grid:
                    xi, yi = i
                    vfh_tar_pos = [ws_zero[0] + xi * GRID_SIZE, ws_zero[1] + yi * GRID_SIZE]
                    vfh_obs_pos = copy.deepcopy(obs_re_pos)
                    vfh_obs_pos.append(tar_pos)
                    ob = len(vfh_obs_pos)
                    vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, rob_pos, d_max, eta)
                    if vfh[3] == 0:
                        # print("candidate", xi, yi, "can not be rearranged")
                        # print("candidate penalty")
                        can_pan.append(i)
                for i in can_pan:
                    can_grid.remove(i)

                # print("\ncheck if candidate occludes the target.")
                can_pan = []
                for i in can_grid:
                    xi, yi = i
                    vfh_obs_pos = copy.deepcopy(obs_re_pos)
                    vfh_obs_pos.append([ws_zero[0] + xi * GRID_SIZE, ws_zero[1] + yi * GRID_SIZE])
                    vfh_tar_pos = copy.deepcopy(tar_pos)
                    ob = len(vfh_obs_pos)
                    vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, rob_pos, d_max, eta)
                    # print("vfh", vfh)
                    if vfh[3] == 0:
                        # print("candidate", i, "occludes the target", vfh_tar_pos)
                        # print("candidate penalty")
                        can_pan.append(i)
                for i in can_pan:
                    can_grid.remove(i)

                # print("\ncheck if the candidate occludes the obstacle.")
                can_pan = []
                for i in can_grid:
                    for oi in ore_grid:
                        xi, yi = i
                        vfh_tar_pos = [ws_zero[0] + oi[0] * GRID_SIZE, ws_zero[1] + oi[1] * GRID_SIZE]
                        vfh_obs_pos = copy.deepcopy(obs_re_pos)
                        vfh_obs_pos.append([ws_zero[0] + xi * GRID_SIZE, ws_zero[1] + yi * GRID_SIZE])
                        ob = len(vfh_obs_pos)
                        vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, rob_pos, d_max, eta)
                        if vfh[3] == 0:
                            if i in can_pan == 0:
                                # print("candidate", i, "occludes the obstacle to remove", oi)
                                # print("candidate penalty")
                                can_pan.append(i)
                for i in can_pan:
                    can_grid.remove(i)

                grid_val_can = copy.deepcopy(grid_del)
                for i in can_grid:
                    xi, yi = i
                    grid_val_can = obstacle_circle(grid_val_can, [xi, yi, 0.04], 3)  # target

                # print("\ncheck the occ for the candidates")
                can_occ_zero = []
                for co in can_grid:  # assume co as an obstacle
                    occ = 0
                    for ci in can_grid:  # assume ci as the target position
                        if co != ci:
                            vfh_tar_pos = [ws_zero[0] + ci[0] * GRID_SIZE, ws_zero[1] + ci[1] * GRID_SIZE]
                            vfh_obs_pos = copy.deepcopy(obs_re_pos)
                            vfh_obs_pos.append([ws_zero[0] + co[0] * GRID_SIZE, ws_zero[1] + co[1] * GRID_SIZE])
                            vfh_obs_pos.append(tar_pos)
                            ob = len(vfh_obs_pos)
                            vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, rob_pos, d_max, eta)
                            if vfh[3] == 0:
                                occ = occ + 1
                    if occ == 0:
                        can_occ_zero.append(co)

                print("\nocc zero candidates are", can_occ_zero)
                grid_occ_zero = copy.deepcopy(grid_del)
                for i in can_occ_zero:
                    xi, yi = i
                    grid_occ_zero = obstacle_circle(grid_occ_zero, [xi, yi, 0.04], 3)  # target

                # draw_grid_info(grid_ori)
                # draw_grid_info(grid_del)
                # draw_grid_info(grid_max_can)
                # if len(can_grid) == 0:
                #     print("There are no candidates!")
                # else:
                #     draw_grid_info(grid_val_can)
                #     draw_grid_info(grid_occ_zero)

                print("with rearranging", len(ore_pos), "we have", len(can_grid), "candidates with", len(can_occ_zero), "occ = zero points")

    # plt.show()