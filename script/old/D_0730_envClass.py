import S_custom_function as CUF
from VFHplus_change_radius import influence
from tree_making_no_plot import tree_making as TM_noplot
from tree_making_plot import tree_making as TM_plot

import numpy as np
import copy


class EnvInfo:

    def __init__(self, rob_pos, ws_width, ws_depth, ws_cen, grid_size, wall_r):
        self.rob_pos = rob_pos
        self.GRID_SIZE = grid_size
        self.ws_w = ws_width
        self.ws_d = ws_depth
        self.ws_cen = ws_cen
        self.ws_zero = [round(self.ws_cen[0] - ws_width * self.GRID_SIZE * 0.5, 2), round(self.ws_cen[1] - ws_depth * self.GRID_SIZE * 0.5, 2)]
        self.obs_wall = self.get_obs_wall(OBJ_R=wall_r)

        self.d_max = 2.0
        self.eta = 45

        grid_act = np.zeros([ws_width, ws_depth])
        self.grid_act = CUF.mark_edge_grid(grid_act)

    def get_env(self, obs_r, tar_r, min_ore):
        while 1:
            self.obs_grid = []
            grid_tmp = copy.deepcopy(self.grid_act)

            for ri in obs_r:
                grid_tmp, obs_center_tmp = CUF.place_circle_object_ig(grid_tmp, ri, 2)
                self.obs_grid.append(obs_center_tmp)
            grid_tmp, tar_tmp = CUF.place_circle_object_ig(grid_tmp, tar_r, 4)
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
            tm_obs_pos.extend(self.obs_wall)
            ob = len(tm_obs_pos)
            tm_obs_ori = []
            for i in range(ob):
                tm_obs_ori.append([0.0, 0.0, 0.0])

            ore_order = TM_noplot(ob, tm_tar_pos, tm_tar_ori, tm_obs_pos, tm_obs_ori, self.rob_pos, self.rob_pos, self.d_max)

            if len(ore_order) > min_ore:
                print"environment setting OK"
                print"target", self.tar_pos
                print"obstacles", self.obs_pos
                print"remove", ore_order, "th obstacles"
                tm_tar_pos = copy.deepcopy(self.tar_pos)
                tm_obs_pos = copy.deepcopy(self.obs_pos)
                tm_obs_pos.extend(self.obs_wall)
                ob = len(tm_obs_pos)
                self.ore_order = TM_plot(ob, tm_tar_pos, tm_tar_ori, tm_obs_pos, tm_obs_ori, self.rob_pos, self.rob_pos, self.d_max)

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
                tm_obs_pos.extend(self.obs_wall)
                ob = len(tm_obs_pos)
                tmp_order = TM_noplot(ob, tm_tar_pos, tm_tar_ori, tm_obs_pos, tm_obs_ori, self.rob_pos, self.rob_pos, self.d_max)
                if len(tmp_order) == 1:
                    break
        '''
        with out additional rearrangement
        '''
        if len(self.ore_order) > min_ore:
            self.ore_grid = []
            self.ore_pos = []
            self.ore_r = []

            self.obs_re_grid = copy.deepcopy(self.obs_grid)
            self.obs_re_pos = copy.deepcopy(self.obs_pos)
            self.obs_re_r = copy.deepcopy(obs_r)

            for i in self.ore_order:
                if i != 'T':
                    self.ore_grid.append(self.obs_re_grid[i])
                    self.ore_pos.append(self.obs_re_pos[i])
                    self.ore_r.append(self.obs_re_r[i])
            for i in self.ore_order:
                if i != 'T':
                    self.obs_re_grid.remove(self.obs_grid[i])
                    self.obs_re_pos.remove(self.obs_pos[i])
                    self.obs_re_r.remove(obs_r[i])

            self.grid_ori = copy.deepcopy(self.grid_act)
            for i in range(len(obs_r)):
                self.grid_ori = CUF.obstacle_circle(self.grid_ori, [round(self.obs_grid[i][0], 2), round(self.obs_grid[i][1], 2), obs_r[i]], 2)
            self.grid_ori = CUF.obstacle_circle(self.grid_ori, [self.tar_grid[0], self.tar_grid[1], tar_r], 4)  # target

            self.grid_del = copy.deepcopy(self.grid_act)
            for i in range(len(self.obs_re_r)):
                self.grid_del = CUF.obstacle_circle(self.grid_del, [round(self.obs_re_grid[i][0], 2), round(self.obs_re_grid[i][1], 2), self.obs_re_r[i]], 2)
            self.grid_del = CUF.obstacle_circle(self.grid_del, [self.tar_grid[0], self.tar_grid[1], tar_r], 4)  # target
            self.ore_order.pop()

    def get_max_can(self, input_grid, bt_num, trial_num, ):
        bt_circle = []
        can_grid = []
        circle_r = max(self.ore_r)+0.005
        for bt in range(bt_num):
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

        print(max_cir_num.index(max(max_cir_num)))
        max_trial = max_cir_num.index(max(max_cir_num))

        self.grid_max_can = copy.deepcopy(bt_circle[max_trial][1])
        self.can_grid = bt_circle[max_trial][0]
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
                    tm_obs_pos.extend(self.obs_wall)
                    ob = len(tm_obs_pos)
                    tm_obs_ori = []
                    for obs_ori_i in range(ob):
                        tm_obs_ori.append([0.0, 0.0, 0.0])

                    ore_order = TM_noplot(ob, tm_tar_pos, tm_tar_ori, tm_obs_pos, tm_obs_ori, self.rob_pos, self.rob_pos, self.d_max)
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

    def get_can_BT(self, in_can_info, in_obs_pos, in_tar_pos):
        tmp_can_info = copy.deepcopy(in_can_info)
        # print("\nCheck if candidate blocks the target")
        for ci in range(len(tmp_can_info)):
            vfh_obs_pos = copy.deepcopy(in_obs_pos)
            vfh_obs_pos.append(tmp_can_info[ci].pos)
            vfh_obs_pos.extend(self.obs_wall)
            vfh_tar_pos = copy.deepcopy(in_tar_pos)
            ob = len(vfh_obs_pos)
            vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, self.rob_pos, self.d_max, self.eta)
            if vfh[3] == 0:
                tmp_can_info[ci].BT = 1  # BT == 1 : The candidate blocks the target.
            # else:                        # BT == 0 : The candidate does not block the target.
            #     tmp_can_info[ci].BT = 0
                # print "can", ci, "bt=0"

        return tmp_can_info

    def get_cf(self, in_can_info):
        tmp_cf_pos = []
        tmp_can_info = copy.deepcopy(in_can_info)

        # print("\nCheck the candidate ORC.")
        for ci in range(len(tmp_can_info)):
            # print "\ncan ", ci, "th has A, BT :", tmp_can_info[ci].A, tmp_can_info[ci].BT
            if tmp_can_info[ci].A == 1 and tmp_can_info[ci].BT == 0:
                tmp_cf_pos.append(tmp_can_info[ci].pos)

        return tmp_cf_pos

    def get_cf_b(self, in_cf_pos, in_obs_pos):
        tmp_cf_pos = copy.deepcopy(in_cf_pos)
        tmp_obs_pos = copy.deepcopy(in_obs_pos)
        tmp_b = []
        for cb in range(len(tmp_cf_pos)):  # cb: The candidate that will check the b value
            b = 0
            for ci in range(len(tmp_cf_pos)):  # ci: Other candidates for checking the b value
                if cb != ci:
                    # print "\ntar", tmp_cf_pos[ci]
                    # print "obs", tmp_obs_pos
                    vfh_tar_pos = copy.deepcopy(tmp_cf_pos[ci])
                    vfh_obs_pos = copy.deepcopy(tmp_obs_pos)
                    vfh_obs_pos.append(tmp_cf_pos[cb])
                    vfh_obs_pos.extend(self.obs_wall)
                    ob = len(vfh_obs_pos)
                    vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, self.rob_pos, self.d_max, self.eta)
                    if vfh[3] == 0:
                        b = b + 1
            tmp_b.append(b)
        return tmp_b


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