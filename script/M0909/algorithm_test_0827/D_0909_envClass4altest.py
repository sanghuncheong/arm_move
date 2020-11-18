import S_custom_function as CUF
from VFHplus_change_radius import influence
# from VFHplus_mobile import influence
# from tree_making_no_plot import tree_making as TM_noplot
# from tree_making_no_plot2 import tree_making as TM_noplot
# from tree_making_plot import tree_making as TM_plot
from nam_vfh_algorithm_0828 import g_ore as NG_ore

import numpy as np
import copy
import matplotlib.pyplot as plt

class EnvInfo:

    def __init__(self, rob_pos, ws_width, ws_depth, ws_cen, grid_size, wall_r):
        self.rob_pos = rob_pos
        self.GRID_SIZE = grid_size
        self.ws_w = ws_width
        self.ws_d = ws_depth
        self.ws_cen = ws_cen
        self.ws_zero = [round(self.ws_cen[0] - ws_width * self.GRID_SIZE * 0.5, 2), round(self.ws_cen[1] - ws_depth * self.GRID_SIZE * 0.5, 2)]
        self.obs_wall = self.get_obs_wall(OBJ_R=wall_r)

        self.order_error_flag = 1
        self.d_max = 2.0
        self.eta = 30

        grid_act = np.zeros([ws_width, ws_depth])
        self.grid_act = CUF.mark_edge_grid(grid_act)

    def get_env(self, obs_r, tar_r, min_ore):
        while 1:
            self.obs_grid = []
            grid_tmp = copy.deepcopy(self.grid_act)
            self.obs_r = obs_r
            self.tar_r = tar_r
            for ri in self.obs_r:
                grid_tmp, obs_center_tmp = CUF.place_circle_object_ig(grid_tmp, ri, 2)
                self.obs_grid.append(obs_center_tmp)
            grid_tmp, tar_tmp = CUF.place_circle_object_ig(grid_tmp, self.tar_r, 4)
            self.tar_grid = copy.deepcopy(tar_tmp)

            self.obs_pos = []
            for i in self.obs_grid:
                xi, yi = i
                self.obs_pos.append([round(xi * self.GRID_SIZE + self.ws_zero[0], 2), round(yi * self.GRID_SIZE + self.ws_zero[1], 2)])
            self.tar_pos = [round(self.tar_grid[0] * self.GRID_SIZE + self.ws_zero[0], 2), round(self.tar_grid[1] * self.GRID_SIZE + self.ws_zero[1], 2)]  # target object!

            self.d_max = 2.0
            tm_tar_pos = copy.deepcopy(self.tar_pos)
            tm_tar_ori = [0.0, 0.0, 0.0]
            tm_obs_pos = copy.deepcopy(self.obs_pos)
            # tm_obs_pos.extend(self.obs_wall)
            ob = len(tm_obs_pos)
            tm_obs_ori = []
            for i in range(ob):
                tm_obs_ori.append([0.0, 0.0, 0.0])

            ore_order = NG_ore(tm_tar_pos, tm_obs_pos, self.tar_r, self.obs_r, self.rob_pos, self.ws_zero, [self.ws_w * self.GRID_SIZE, self.ws_d * self.GRID_SIZE])
            # ore_order = TM_noplot(ob, tm_tar_pos, tm_tar_ori, tm_obs_pos, tm_obs_ori, self.rob_pos, self.rob_pos, self.d_max)
            # ore_order = TM_noplot(ob, tm_tar_pos, tm_tar_ori, tm_obs_pos, tm_obs_ori, self.obs_wall, self.rob_pos, self.rob_pos, self.d_max)
            # TM_noplot()
            if len(ore_order) > min_ore:
                # print"before rearrangemet: ", ore_order
                tm_tar_pos = copy.deepcopy(self.tar_pos)
                tm_obs_pos = copy.deepcopy(self.obs_pos)
                # tm_obs_pos.extend(self.obs_wall)
                # ob = len(tm_obs_pos)
                self.ore_order = ore_order
                # self.ore_order = TM_plot(ob, tm_tar_pos, tm_tar_ori, tm_obs_pos, tm_obs_ori, self.rob_pos, self.rob_pos, self.d_max)

                # check if the obstacles are rearranged then target is reachable
                tm_tar_pos = copy.deepcopy(self.tar_pos)
                tm_obs_pos = copy.deepcopy(self.obs_pos)
                tm_ore_pos = []
                for i in self.ore_order:
                    if i != 'T':
                        tm_obs_pos.remove(self.obs_pos[i])
                # tm_obs_pos.extend(self.obs_wall)
                ob = len(tm_obs_pos)
                tmp_order = NG_ore(tm_tar_pos, tm_obs_pos, self.tar_r, self.obs_r, self.rob_pos, self.ws_zero, [self.ws_w * self.GRID_SIZE, self.ws_d * self.GRID_SIZE])
                # tmp_order = TM_noplot(ob, tm_tar_pos, tm_tar_ori, tm_obs_pos, tm_obs_ori, self.obs_wall,self.rob_pos, self.rob_pos, self.d_max)
                # print "after removing:", tmp_order
                if tmp_order[0] == 'T':
                    # print"environment setting OK"
                    break
            # else:
                # print "SHORT...min:", min_ore, "ours:", len(ore_order)
        '''
        with out additional rearrangement
        '''
        if len(self.ore_order) > min_ore:
            self.ore_grid = []
            self.ore_pos = []
            self.ore_r = []

            self.obs_re_grid = copy.deepcopy(self.obs_grid)
            self.obs_re_pos = copy.deepcopy(self.obs_pos)
            self.obs_re_r = copy.deepcopy(self.obs_r)

            for i in self.ore_order:
                if i != 'T':
                    self.ore_grid.append(self.obs_re_grid[i])
                    self.ore_pos.append(self.obs_re_pos[i])
                    self.ore_r.append(self.obs_re_r[i])
            for i in self.ore_order:
                if i != 'T':
                    self.obs_re_grid.remove(self.obs_grid[i])
                    self.obs_re_pos.remove(self.obs_pos[i])
                    self.obs_re_r.remove(self.obs_r[i])

            self.grid_ori = copy.deepcopy(self.grid_act)
            for i in range(len(self.obs_r)):
                self.grid_ori = CUF.obstacle_circle(self.grid_ori, [round(self.obs_grid[i][0], 2), round(self.obs_grid[i][1], 2), self.obs_r[i]], 2)
            self.grid_ori = CUF.obstacle_circle(self.grid_ori, [self.tar_grid[0], self.tar_grid[1], self.tar_r], 4)  # target

            self.grid_del = copy.deepcopy(self.grid_act)
            for i in range(len(self.obs_re_r)):
                self.grid_del = CUF.obstacle_circle(self.grid_del, [round(self.obs_re_grid[i][0], 2), round(self.obs_re_grid[i][1], 2), self.obs_re_r[i]], 2)
            self.grid_del = CUF.obstacle_circle(self.grid_del, [self.tar_grid[0], self.tar_grid[1], self.tar_r], 4)  # target
            self.ore_order.pop()

    def get_env_case1(self):
        while 1:
            obs_r = [0.035, 0.035, 0.035, 0.035, 0.035, 0.035, 0.035, 0.035]
            tar_r = 0.035
            min_ore = 2
            self.obs_grid = []
            grid_tmp = copy.deepcopy(self.grid_act)
            self.obs_r = obs_r
            self.tar_r = tar_r

            self.obs_grid = [[10, 70], [10, 50], [10, 10],
                             [20, 70], [20, 50], [19, 31], [24, 25], [20, 10]]
            self.tar_grid = [30, 30]

            # self.grid_ori = copy.deepcopy(self.grid_act)
            # for i in range(len(self.obs_r)):
            #     print "obs", i, round(self.obs_grid[i][0], 2), round(self.obs_grid[i][1], 2)
            #     self.grid_ori = CUF.obstacle_circle(self.grid_ori, [round(self.obs_grid[i][0], 2), round(self.obs_grid[i][1], 2), self.obs_r[i]], 2)
            # self.grid_ori = CUF.obstacle_circle(self.grid_ori, [self.tar_grid[0], self.tar_grid[1], self.tar_r], 4)  # target
            # break
            # for ri in self.obs_r:
            #     grid_tmp, obs_center_tmp = CUF.place_circle_object_ig(grid_tmp, ri, 2)
            #     self.obs_grid.append(obs_center_tmp)
            # grid_tmp, tar_tmp = CUF.place_circle_object_ig(grid_tmp, self.tar_r, 4)
            # self.tar_grid = copy.deepcopy(tar_tmp)

            self.obs_pos = []
            for i in self.obs_grid:
                xi, yi = i
                self.obs_pos.append([round(xi * self.GRID_SIZE + self.ws_zero[0], 2), round(yi * self.GRID_SIZE + self.ws_zero[1], 2)])
            self.tar_pos = [round(self.tar_grid[0] * self.GRID_SIZE + self.ws_zero[0], 2), round(self.tar_grid[1] * self.GRID_SIZE + self.ws_zero[1], 2)]  # target object!

            self.d_max = 2.0
            tm_tar_pos = copy.deepcopy(self.tar_pos)
            tm_tar_ori = [0.0, 0.0, 0.0]
            tm_obs_pos = copy.deepcopy(self.obs_pos)
            # tm_obs_pos.extend(self.obs_wall)
            ob = len(tm_obs_pos)
            tm_obs_ori = []
            for i in range(ob):
                tm_obs_ori.append([0.0, 0.0, 0.0])

            # ore_order = TM_noplot(ob, tm_tar_pos, tm_tar_ori, tm_obs_pos, tm_obs_ori, self.obs_wall, self.rob_pos, self.rob_pos, self.d_max)
            ore_order = NG_ore(tm_tar_pos, tm_obs_pos, self.tar_r, self.obs_r, self.rob_pos, self.ws_zero, [self.ws_w * self.GRID_SIZE, self.ws_d * self.GRID_SIZE])
            if len(ore_order) > min_ore:
                # print"environment setting OK"
                # print"target", self.tar_pos
                # print"obstacles", self.obs_pos
                # print"remove", ore_order, "th obstacles"
                tm_tar_pos = copy.deepcopy(self.tar_pos)
                tm_obs_pos = copy.deepcopy(self.obs_pos)
                # tm_obs_pos.extend(self.obs_wall)
                ob = len(tm_obs_pos)
                # self.ore_order = ore_order
                self.ore_order = TM_noplot(ob, tm_tar_pos, tm_tar_ori, tm_obs_pos, tm_obs_ori, self.obs_wall, self.rob_pos, self.rob_pos, self.d_max)

                # check if the obstacles are rearranged then target is reachable
                tm_tar_pos = copy.deepcopy(self.tar_pos)
                tm_obs_pos = copy.deepcopy(self.obs_pos)
                tm_ore_pos = []
                for i in self.ore_order:
                    if i != 'T':
                        tm_ore_pos.append(self.obs_pos[i])
                for i in self.ore_order:
                    if i != 'T':
                        tm_obs_pos.remove(self.obs_pos[i])
                # tm_obs_pos.extend(self.obs_wall)
                ob = len(tm_obs_pos)

                # tmp_order = NG_ore(tm_tar_pos, tm_obs_pos, self.tar_r, self.obs_r, self.rob_pos, self.ws_zero, [self.ws_w * self.GRID_SIZE, self.ws_d * self.GRID_SIZE])
                tmp_order = TM_noplot(ob, tm_tar_pos, tm_tar_ori, tm_obs_pos, tm_obs_ori, self.obs_wall, self.rob_pos, self.rob_pos, self.d_max)
                if len(tmp_order) == 1:
                    print"environment setting OK"
                    break
        '''
        # with out additional rearrangement
        '''
        if len(self.ore_order) > min_ore:
            self.ore_grid = []
            self.ore_pos = []
            self.ore_r = []

            self.obs_re_grid = copy.deepcopy(self.obs_grid)
            self.obs_re_pos = copy.deepcopy(self.obs_pos)
            self.obs_re_r = copy.deepcopy(self.obs_r)

            for i in self.ore_order:
                if i != 'T':
                    self.ore_grid.append(self.obs_re_grid[i])
                    self.ore_pos.append(self.obs_re_pos[i])
                    self.ore_r.append(self.obs_re_r[i])
            for i in self.ore_order:
                if i != 'T':
                    self.obs_re_grid.remove(self.obs_grid[i])
                    self.obs_re_pos.remove(self.obs_pos[i])
                    self.obs_re_r.remove(self.obs_r[i])

            self.grid_ori = copy.deepcopy(self.grid_act)
            for i in range(len(self.obs_r)):
                self.grid_ori = CUF.obstacle_circle(self.grid_ori, [round(self.obs_grid[i][0], 2), round(self.obs_grid[i][1], 2), self.obs_r[i]], 2)
            self.grid_ori = CUF.obstacle_circle(self.grid_ori, [self.tar_grid[0], self.tar_grid[1], self.tar_r], 4)  # target

            self.grid_del = copy.deepcopy(self.grid_act)
            for i in range(len(self.obs_re_r)):
                self.grid_del = CUF.obstacle_circle(self.grid_del, [round(self.obs_re_grid[i][0], 2), round(self.obs_re_grid[i][1], 2), self.obs_re_r[i]], 2)
            self.grid_del = CUF.obstacle_circle(self.grid_del, [self.tar_grid[0], self.tar_grid[1], self.tar_r], 4)  # target
            self.ore_order.pop()

    def get_max_can_case1(self):
        # [19, 35], [24, 25]
        self.can_grid = [[30, 70], [30, 50], [14, 25], [30, 10]]
        self.can_pos = []
        circle_r = max(self.ore_r)+0.005
        for i in self.can_grid:
            xi, yi = i
            self.can_pos.append([self.ws_zero[0] + xi * self.GRID_SIZE, self.ws_zero[1] + yi * self.GRID_SIZE])

        self.grid_max_can = copy.deepcopy(self.grid_act)
        for i in range(len(self.obs_r)):
            self.grid_max_can = CUF.obstacle_circle(self.grid_max_can, [round(self.obs_grid[i][0], 2), round(self.obs_grid[i][1], 2), self.obs_r[i]], 2)
        self.grid_max_can = CUF.obstacle_circle(self.grid_max_can, [self.tar_grid[0], self.tar_grid[1], self.tar_r], 4)  # target
        for i in range(len(self.can_grid)):
            self.grid_max_can = CUF.obstacle_circle(self.grid_max_can, [self.can_grid[i][0], self.can_grid[i][1], circle_r], 3)

    def update_env(self, in_obs_pos, in_obs_grid):
        self.d_max = 2.0
        tm_tar_pos = copy.deepcopy(self.tar_pos)
        tm_tar_ori = [0.0, 0.0, 0.0]
        tm_obs_pos = copy.deepcopy(in_obs_pos)
        # tm_obs_pos.extend(self.obs_wall)
        ob = len(tm_obs_pos)
        tm_obs_ori = []
        for i in range(ob):
            tm_obs_ori.append([0.0, 0.0, 0.0])

        self.ore_order = NG_ore(tm_tar_pos, tm_obs_pos, self.tar_r, self.obs_r, self.rob_pos, self.ws_zero, [self.ws_w * self.GRID_SIZE, self.ws_d * self.GRID_SIZE])
        # self.ore_order = TM_noplot(ob, tm_tar_pos, tm_tar_ori, tm_obs_pos, tm_obs_ori, self.obs_wall, self.rob_pos, self.rob_pos, self.d_max)

        # check if the obstacles are rearranged then target is reachable
        while 1:
            tm_tar_pos = copy.deepcopy(self.tar_pos)
            tm_obs_pos = copy.deepcopy(self.obs_pos)
            tm_ore_pos = []
            for i in self.ore_order:
                if i != 'T':
                    tm_obs_pos[i] = [5.0, 0.0]
                    # tm_obs_pos.remove(self.obs_pos[i])
            # tm_obs_pos.extend(self.obs_wall)

            tmp_order = NG_ore(tm_tar_pos, tm_obs_pos, self.tar_r, self.obs_r, self.rob_pos, self.ws_zero, [self.ws_w * self.GRID_SIZE, self.ws_d * self.GRID_SIZE])
            # tmp_order = TM_noplot(ob, tm_tar_pos, tm_tar_ori, tm_obs_pos, tm_obs_ori, self.obs_wall,self.rob_pos, self.rob_pos, self.d_max)
            # print "after removing:", tmp_order
            if tmp_order[0] != 'T':
                # print "tricky environment so extend", self.ore_order
                self.ore_order.pop()
                # print "delete target", self.ore_order
                self.ore_order.extend(tmp_order)
                # print "to", self.ore_order
            else:
                if tmp_order[0] == 'T':
                    # print "ok", self.ore_order
                    break

        self.ore_grid = []
        self.ore_pos = []
        self.ore_r = []

        self.obs_re_grid = copy.deepcopy(in_obs_grid)
        self.obs_re_pos = copy.deepcopy(in_obs_pos)
        self.obs_re_r = copy.deepcopy(self.obs_r)

        for i in self.ore_order:
            if i != 'T':
                self.ore_grid.append(self.obs_re_grid[i])
                self.ore_pos.append(self.obs_re_pos[i])
                self.ore_r.append(self.obs_re_r[i])
        for i in self.ore_order:
            if i != 'T':
                self.obs_re_grid.remove(self.obs_grid[i])
                self.obs_re_pos.remove(self.obs_pos[i])
                self.obs_re_r.remove(self.obs_r[i])

        self.grid_ori = copy.deepcopy(self.grid_act)
        for i in range(len(self.obs_r)):
            self.grid_ori = CUF.obstacle_circle(self.grid_ori, [round(self.obs_grid[i][0], 2), round(self.obs_grid[i][1], 2), self.obs_r[i]], 2)
        self.grid_ori = CUF.obstacle_circle(self.grid_ori, [self.tar_grid[0], self.tar_grid[1], self.tar_r], 4)  # target

        self.grid_del = copy.deepcopy(self.grid_act)
        for i in range(len(self.obs_re_r)):
            self.grid_del = CUF.obstacle_circle(self.grid_del, [round(self.obs_re_grid[i][0], 2), round(self.obs_re_grid[i][1], 2), self.obs_re_r[i]], 2)
        self.grid_del = CUF.obstacle_circle(self.grid_del, [self.tar_grid[0], self.tar_grid[1], self.tar_r], 4)  # target
        self.ore_order.pop()
        # else:
        #     self.order_error_flag = 0

    def get_max_can(self, input_grid, bt_num, trial_num, ):
        bt_circle = []
        circle_r = max(self.ore_r)+0.03
        for bt in range(bt_num):
            can_grid = []
            grid_can = copy.deepcopy(input_grid)  # get original scene from the grid_set
            empt_grid, occu_grid = CUF.getEmpOcc(grid_can)
            for i in range(trial_num):
                pick_cen = np.random.randint(0, len(empt_grid))
                check_sum = 0
                for oc in range(len(occu_grid)):
                    d_w = empt_grid[pick_cen][0] - occu_grid[oc][0]
                    d_d = empt_grid[pick_cen][1] - occu_grid[oc][1]
                    d_c = (d_w * d_w + d_d * d_d) ** 0.5 * self.GRID_SIZE
                    if d_c <= circle_r:
                        check_sum = 1

                if check_sum == 0:
                    can_grid.append(empt_grid[pick_cen])
                    for em in range(len(empt_grid)):
                        d_w = empt_grid[pick_cen][0] - empt_grid[em][0]
                        d_d = empt_grid[pick_cen][1] - empt_grid[em][1]
                        d_c = (d_w * d_w + d_d * d_d) ** 0.5 * self.GRID_SIZE
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

        self.grid_max_can = copy.deepcopy(bt_circle[max_trial][1])
        t_can_grid = bt_circle[max_trial][0]

        can_dist = []
        for i in t_can_grid:
            can_dist.append(np.sqrt((self.tar_pos[0] - i[0]) ** 2 + (self.tar_pos[1] - i[1]) ** 2))
        sort_can_dist = copy.deepcopy(can_dist)
        sort_can_dist.sort(reverse=True)
        can_grid = []
        for i in range(len(can_dist)):
            can_grid.append(t_can_grid[can_dist.index(sort_can_dist[i])])
        self.can_grid = can_grid

        self.can_pos = []
        for i in self.can_grid:
            xi, yi = i
            self.can_pos.append([self.ws_zero[0] + xi * self.GRID_SIZE, self.ws_zero[1] + yi * self.GRID_SIZE])

    def get_obs_wall(self, OBJ_R):
        ws_side = []
        ws_side.append(
            [self.ws_cen[0] - self.ws_w * self.GRID_SIZE * 0.5, self.ws_cen[1] - self.ws_d * self.GRID_SIZE * 0.5 - OBJ_R])  # left low point
        ws_side.append([self.ws_cen[0] + self.ws_w * self.GRID_SIZE * 0.5 + OBJ_R,
                        self.ws_cen[1] - self.ws_d * self.GRID_SIZE * 0.5 - OBJ_R])  # right low point
        ws_side.append([self.ws_cen[0] + self.ws_w * self.GRID_SIZE * 0.5 + OBJ_R,
                        self.ws_cen[1] + self.ws_d * self.GRID_SIZE * 0.5 + OBJ_R])  # right high point
        ws_side.append(
            [self.ws_cen[0] - self.ws_w * self.GRID_SIZE * 0.5, self.ws_cen[1] + self.ws_d * self.GRID_SIZE * 0.5 + OBJ_R])  # left high point

        obs_wall = []
        obs_wall.extend(CUF.linspace2D(ws_side[0], ws_side[1], round(self.ws_w * self.GRID_SIZE / OBJ_R)))
        obs_wall.extend(CUF.linspace2D(ws_side[1], ws_side[2], round(self.ws_d * self.GRID_SIZE / OBJ_R)))
        obs_wall.extend(CUF.linspace2D(ws_side[2], ws_side[3], round(self.ws_w * self.GRID_SIZE / OBJ_R)))
        return obs_wall

    def get_can_info(self, in_can_info, in_obs_pos, in_obs_re_pos, in_ore_order, in_tar_pos):
        tmp_can_info = []
        for i in range(len(in_ore_order)):
            tmp_can_info.append(copy.deepcopy(in_can_info))
        tmp_obs_pos = copy.deepcopy(in_obs_pos)
        tmp_obs_re_pos = copy.deepcopy(in_obs_re_pos)
        tmp_ore_order = copy.deepcopy(in_ore_order)
        tmp_tar_pos = copy.deepcopy(in_tar_pos)
        # print("\nCheck if candidate blocks the target")
        for step_i in range(len(tmp_ore_order)):
            for i in range(len(tmp_can_info[step_i])):
                vfh_obs_pos = copy.deepcopy(tmp_obs_re_pos)
                vfh_obs_pos.append(tmp_can_info[step_i][i].pos)
                vfh_obs_pos.extend(self.obs_wall)
                vfh_tar_pos = copy.deepcopy(tmp_tar_pos)
                ob = len(vfh_obs_pos)
                vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, self.rob_pos, self.d_max, self.eta)
                # vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, self.rob_pos, self.obs_r, self.tar_r)
                if vfh[3] == 0:
                    tmp_can_info[step_i][i].BT = 1  # BT == 1 : The candidate blocks the target.
                else:                       # BT == 0 : The candidate does not block the target.
                    tmp_can_info[step_i][i].BT = 0

            # print("\nCheck if the candidate is accessible.")
            for i in range(len(tmp_can_info[step_i])):
                vfh_tar_pos = copy.deepcopy(tmp_can_info[step_i][i].pos)
                vfh_obs_pos = copy.deepcopy(tmp_obs_pos)
                for si in range(step_i+1):
                    # print "\nstep", si, "\nbefore", vfh_obs_pos
                    vfh_obs_pos.remove(tmp_obs_pos[tmp_ore_order[si]])
                    # print "after", vfh_obs_pos
                vfh_obs_pos.append(tmp_tar_pos)
                vfh_obs_pos.extend(self.obs_wall)
                ob = len(vfh_obs_pos)
                vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, self.rob_pos, self.d_max, self.eta)
                if vfh[3] == 0:                      # A == 1 : The candidate is accessible.
                    tmp_can_info[step_i][i].A = 0    # A == 0 : The candidate is not accessible.
                else:                   #
                    tmp_can_info[step_i][i].A = 1

            # print("\nCheck the candidate ORC.")
            for i in range(len(tmp_can_info[step_i])):
                vfh_tar_pos = copy.deepcopy(tmp_can_info[step_i][i].pos)
                vfh_obs_pos = copy.deepcopy(tmp_obs_pos)
                vfh_obs_pos.append(tmp_tar_pos)
                vfh_obs_pos.extend(self.obs_wall)
                ob = len(vfh_obs_pos)
                vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, self.rob_pos, self.d_max, self.eta)
                if vfh[3] == 0:              # A == 1 : The candidate is accessible.
                    tmp_can_info[step_i][i].A = 0    # A == 0 : The candidate is not accessible.
                    tm_tar_pos = copy.deepcopy(vfh_tar_pos)
                    tm_tar_ori = [0.0, 0.0, 0.0]
                    tm_obs_pos = copy.deepcopy(tmp_obs_pos)
                    # tm_obs_pos.extend(self.obs_wall)
                    ob = len(tm_obs_pos)
                    tm_obs_ori = []
                    for obs_ori_i in range(ob):
                        tm_obs_ori.append([0.0, 0.0, 0.0])

                    ore_order = NG_ore(tm_tar_pos, tm_obs_pos, self.tar_r, self.obs_r, self.rob_pos, self.ws_zero, [self.ws_w * self.GRID_SIZE, self.ws_d * self.GRID_SIZE])
                    # ore_order = TM_noplot(ob, tm_tar_pos, tm_tar_ori, tm_obs_pos, tm_obs_ori, self.rob_pos, self.rob_pos, self.d_max)
                    ore_order.pop()  # The last order is always the target so we need to pop the last element.
                    tmp_can_info[step_i][i].ORC = ore_order
                else:                   #
                    tmp_can_info[step_i][i].A = 1

        return tmp_can_info

    def get_can_A(self, in_can_info, in_obs_pos, in_tar_pos):
        tmp_can_info = copy.deepcopy(in_can_info)
        # print("\nCheck if the candidate is accessible.")
        for ci in range(len(tmp_can_info)):
            vfh_tar_pos = copy.deepcopy(tmp_can_info[ci].pos)
            vfh_obs_pos = copy.deepcopy(in_obs_pos)
            vfh_obs_pos.append(copy.deepcopy(in_tar_pos))
            vfh_obs_pos.extend(self.obs_wall)
            ob = len(vfh_obs_pos)
            vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, self.rob_pos, self.d_max, self.eta)
            if vfh[3] == 0:  # A == 1 : The candidate is accessible.
                tmp_can_info[ci].A = 0  # A == 0 : The candidate is not accessible.
            else:  #
                tmp_can_info[ci].A = 1
        return tmp_can_info

    def init_BT(self, in_can_info):
        for ci in in_can_info:
            ci.BT = 0
        return in_can_info

    def get_can_BT(self, in_can_info, in_obs_pos, in_tar_pos):
        tmp_can_info = copy.deepcopy(in_can_info)
        # print("\nCheck if candidate blocks the target")
        for ci in range(len(tmp_can_info)):
            if tmp_can_info[ci].BT == 0:
                vfh_obs_pos = copy.deepcopy(in_obs_pos)
                vfh_obs_pos.append(tmp_can_info[ci].pos)
                vfh_obs_pos.extend(self.obs_wall)
                vfh_tar_pos = copy.deepcopy(in_tar_pos)
                ob = len(vfh_obs_pos)
                vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, self.rob_pos, self.d_max, self.eta)
                if vfh[3] == 0:
                    tmp_can_info[ci].BT = 1  # BT == 1 : The candidate blocks the target.
                else:                        # BT == 0 : The candidate does not block the target.
                    tmp_can_info[ci].BT = 0
                    # print "can", ci, "bt=0"

        return tmp_can_info

    def get_cf(self, in_can_info):
        tmp_cf = []
        tmp_cf_index = []
        tmp_can_info = copy.deepcopy(in_can_info)

        # print("\nCheck the candidate ORC.")
        for ci in range(len(tmp_can_info)):
            # print "\ncan ", ci, "th has A, BT :", tmp_can_info[ci].A, tmp_can_info[ci].BT
            if tmp_can_info[ci].A == 1 and tmp_can_info[ci].BT == 0:
                tmp_cf.append(tmp_can_info[ci])
                tmp_cf_index.append(ci)
        return tmp_cf, tmp_cf_index

    def get_cf_b(self, in_cf, in_obs_pos):
        tmp_cf = copy.deepcopy(in_cf)
        tmp_obs_pos = copy.deepcopy(in_obs_pos)
        tmp_b = []
        for cb in range(len(tmp_cf)):  # cb: The candidate that will check the b value
            b = 0
            for ci in range(len(tmp_cf)):  # ci: Other candidates for checking the b value
                if cb != ci:
                    # print "\ntar", tmp_cf_pos[ci]
                    # print "obs", tmp_obs_pos
                    vfh_tar_pos = copy.deepcopy(tmp_cf[ci].pos)
                    vfh_obs_pos = copy.deepcopy(tmp_obs_pos)
                    vfh_obs_pos.append(tmp_cf[cb].pos)
                    vfh_obs_pos.extend(self.obs_wall)
                    ob = len(vfh_obs_pos)
                    vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, self.rob_pos, self.d_max, self.eta)
                    if vfh[3] == 0:
                        b = b + 1
            tmp_b.append(b)
        return tmp_b

    def get_cp(self, in_can_info):
        tmp_cp = []
        tmp_cp_index = []
        tmp_can_info = copy.deepcopy(in_can_info)

        # print("\nCheck the candidate ORC.")
        for ci in range(len(tmp_can_info)):
            # print "\ncan ", ci, "th has A, BT :", tmp_can_info[ci].A, tmp_can_info[ci].BT
            if tmp_can_info[ci].A == 0:
                tmp_cp.append(tmp_can_info[ci])
                tmp_cp_index.append(ci)
        return tmp_cp, tmp_cp_index

    # def get_can_A(self, in_can_info, in_obs_pos, in_tar_pos):
    #     tmp_can_info = copy.deepcopy(in_can_info)
    #     # print("\nCheck if the candidate is accessible.")
    #     for ci in range(len(tmp_can_info)):
    #         vfh_tar_pos = copy.deepcopy(tmp_can_info[ci].pos)
    #         vfh_obs_pos = copy.deepcopy(in_obs_pos)
    #         vfh_obs_pos.append(copy.deepcopy(in_tar_pos))
    #         vfh_obs_pos.extend(self.obs_wall)
    #         ob = len(vfh_obs_pos)
    #         vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, self.rob_pos, self.d_max, self.eta)
    #         if vfh[3] == 0:  # A == 1 : The candidate is accessible.
    #             tmp_can_info[ci].A = 0  # A == 0 : The candidate is not accessible.
    #             # print "c:", ci, ".A = 0"
    #         else:  #
    #             tmp_can_info[ci].A = 1
    #             # print "c:", ci, ".A = 1"
    #     return tmp_can_info
    #
    # def get_can_BT(self, in_can_info, in_obs_pos, in_tar_pos):
    #     tmp_can_info = copy.deepcopy(in_can_info)
    #     # print("\nCheck if candidate blocks the target")
    #     for ci in range(len(tmp_can_info)):
    #         vfh_obs_pos = copy.deepcopy(in_obs_pos)
    #         vfh_obs_pos.append(tmp_can_info[ci].pos)
    #         vfh_obs_pos.extend(self.obs_wall)
    #         vfh_tar_pos = copy.deepcopy(in_tar_pos)
    #         ob = len(vfh_obs_pos)
    #         vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, self.rob_pos, self.d_max, self.eta)
    #         if vfh[3] == 0:
    #             tmp_can_info[ci].BT = 1  # BT == 1 : The candidate blocks the target.
    #             # print "c:", ci, ".BT = 1"
    #         # else:                        # BT == 0 : The candidate does not block the target.
    #         #     tmp_can_info[ci].BT = 0
    #         #     print "c:", ci, ".BT = 0"
    #
    #     return tmp_can_info

    def get_c_ore(self, in_can_info):
        # print "input", in_can_info
        # print in_can_info[0].pos
        t_c_order = []
        for ci in range(len(in_can_info)):
            tm_tar_pos = in_can_info[ci].pos
            tm_obs_pos = copy.deepcopy(self.obs_pos)
            tm_obs_pos.append(self.tar_pos)
            obs_r = []
            for i in tm_obs_pos:
                obs_r.append(0.035)
            ore_order = NG_ore(tm_tar_pos, tm_obs_pos, self.tar_r, obs_r, self.rob_pos, self.ws_zero, [self.ws_w * self.GRID_SIZE, self.ws_d * self.GRID_SIZE])
            while 1:
                tm_tar_pos = in_can_info[ci].pos
                tm_obs_pos = copy.deepcopy(self.obs_pos)
                tm_obs_pos.append(self.tar_pos)
                obs_r = []
                for i in tm_obs_pos:
                    obs_r.append(0.035)
                for i in ore_order:
                    if i != 'T':
                        tm_obs_pos[i] = [4.0, 0.0]

                tmp_order = NG_ore(tm_tar_pos, tm_obs_pos, self.tar_r, obs_r, self.rob_pos, self.ws_zero, [self.ws_w * self.GRID_SIZE, self.ws_d * self.GRID_SIZE])
                # tmp_order = TM_noplot(ob, tm_tar_pos, tm_tar_ori, tm_obs_pos, tm_obs_ori, self.obs_wall,self.rob_pos, self.rob_pos, self.d_max)
                # print "after removing:", tmp_order
                if tmp_order[0] != 'T':
                    # print "not ok"
                    t_c_order.append([])
                    break
                    # if tmp_order[0] == -1:
                    #     print "no path"
                    #     t_c_order.append([])
                    #     break
                    # if len(ore_order) > len(self.obs_pos):
                    #     print "no path"
                    #     t_c_order.append([])
                    #     break
                    # print "tricky environment for c_ore so extend", ore_order
                    # ore_order.pop()
                    # print "delete target", ore_order
                    # ore_order.extend(tmp_order)
                    # print "to", ore_order
                else:
                    if tmp_order[0] == 'T':
                        # print "ok", ore_order
                        ore_order.pop()

                        if len(tm_obs_pos) in ore_order:
                            # print "\n\nThere is target!! warning!!!\n\n"
                            t_c_order.append([])
                        else:
                            t_c_order.append(ore_order)
                        break
        return t_c_order


class CanInfo:
    def __init__(self, type, pos, grid):
        self.type = type
        self.pos = pos
        self.grid = grid
        self.A = 0
        self.BT = 0
        self.b = 0
        self.ORC = []  # to access, need to remove

    def show(self):
        print "\nCandidate Info"

if __name__=="__main__":
    c = []
    # for i in range(10):
    #     c.append(CandidateInfo())
    #
    # print "c position", c[0].pos
    # print c