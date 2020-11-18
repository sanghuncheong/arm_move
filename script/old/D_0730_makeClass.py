import numpy as np
import matplotlib.pyplot as plt
import copy
import time

from D_0730_envClass import EnvInfo as EI
from D_0730_envClass import CanInfo as CI


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


if __name__ == '__main__':

    ws_w, ws_d = 40, 80  # get table size in the v-rep
    ws_cen = [0.75, 0.00]
    rob_pos = [0.0, 0.0]
    OBJ_R = 0.035

    env = EI(rob_pos, ws_w, ws_d, ws_cen, grid_size=GRID_SIZE, wall_r=OBJ_R)

    # obs_r = [0.035, 0.035, 0.035, 0.035, 0.035, 0.035, 0.035, 0.035, 0.035, 0.035]
    obs_r = [0.035, 0.035, 0.035, 0.035, 0.035, 0.035, 0.035]
    # obs_r = [0.035, 0.035, 0.035, 0.035]
    tar_r = 0.035
    min_ore = 2

    env.get_env(obs_r, tar_r, min_ore)

    env.get_max_can(env.grid_ori, bt_num=1, trial_num=1000)  # We get "grid_max_can", "can_grid"

    '''
    Make object info!
    Type : target, obstacle, candidate
    Info : pos, grid, A, BT, b, ORC, ORE
    '''
    can_info = []
    for i in range(len(env.can_pos)):
        can_info.append((CI('candidate', env.can_pos[i], env.can_grid[i])))

    # check env info got right
    if 1:
        print "\n# of obstacles", len(env.obs_pos), "\n# of candidates", len(env.can_pos)

    '''
    GET candidates info
    '''
    # tmp_can_info = env.get_can_info(can_info, env.obs_pos, env.obs_re_pos, env.ore_order, env.tar_pos)
    # tmp_cf_pos = []
    # if 1:
    #     for step_i in range(len(env.ore_order)):
    #         tmp_cf_step = []
    #         for i in range(len(can_info)):
    #             # print "\n", i, "th candidtae A:", can_info[i].A, " BT:", can_info[i].BT, " ORC:", can_info[i].ORC
    #             # print "\n", i, "th candidtae A:", tmp_can_info[step_i][i].A, " BT:", tmp_can_info[step_i][i].BT, " ORC:", tmp_can_info[step_i][i].ORC
    #             # print "\n", i, "can pos:", tmp_can_info[step_i][i].pos
    #             if tmp_can_info[step_i][i].A == 1 and tmp_can_info[step_i][i].BT == 0:
    #                 tmp_cf_step.append(tmp_can_info[step_i][i].pos)
    #                 print "\n", i, "can pos:", tmp_can_info[step_i][i].pos
    #                 print "Feasible candidate!"
    #         tmp_cf_pos.append(tmp_cf_step)
    t_can_info = []
    # Check C.A
    for i in range(len(env.ore_order)):
        in_can_info = copy.deepcopy(can_info)
        in_obs_pos = copy.deepcopy(env.obs_pos)
        for ore_i in range(i+1):
            in_obs_pos.remove(env.obs_pos[env.ore_order[ore_i]])
        t_can_info.append(env.get_can_A(in_can_info, in_obs_pos, env.tar_pos))

    # Check C.BT
    for i in range(len(env.ore_order)):
        in_can_info = copy.deepcopy(t_can_info[i])
        in_obs_pos = copy.deepcopy(env.obs_pos)
        for ore_i in range(len(env.ore_order)):
            in_obs_pos.remove(env.obs_pos[env.ore_order[ore_i]])
        t_can_info[i] = env.get_can_BT(in_can_info, in_obs_pos, env.tar_pos)

    # Check C.BO
    for i in range(len(env.ore_order)):
        in_can_info = copy.deepcopy(t_can_info[i])
        in_obs_pos = copy.deepcopy(env.obs_pos)
        for ore_i in range(len(env.ore_order)):
            in_obs_pos.remove(env.obs_pos[env.ore_order[ore_i]])
        for j in range(len(env.ore_order)):
            if j > i:
                t_can_info[i] = env.get_can_BT(in_can_info, in_obs_pos, env.obs_pos[env.ore_order[j]])

    t_cf_pos = []
    for i in range(len(env.ore_order)):
        in_can_info = copy.deepcopy(t_can_info[i])
        in_obs_pos = copy.deepcopy(env.obs_pos)
        for ore_i in range(i+1):
            in_obs_pos.remove(env.obs_pos[env.ore_order[ore_i]])
        t_cf_pos.append(env.get_cf(in_can_info))
        print "\n step", i, " has # of cf pos:", len(t_cf_pos[i])

    t_b = []
    for i in range(len(env.ore_order)):
        in_obs_pos = copy.deepcopy(env.obs_pos)
        for ore_i in range(i+1):
            in_obs_pos.remove(env.obs_pos[env.ore_order[ore_i]])
        t_b.append(env.get_cf_b(t_cf_pos[i], in_obs_pos))
        print "\n step", i, " has cf b:", t_b[i]

    CUF.draw_grid_info(env.grid_ori)
    CUF.draw_grid_info(env.grid_del)

    CUF.draw_grid_info(env.grid_max_can)    # clean max can grid

    CUF.draw_grid_info(env.grid_max_can)
    for c_i in range(len(can_info)):
        plt.text(can_info[c_i].grid[0], can_info[c_i].grid[1], 'Can'+str(c_i), fontsize=20, ha='center', bbox=dict(facecolor='pink', alpha=0.8))
    for o_i in range(len(env.obs_grid)):
        plt.text(env.obs_grid[o_i][0], env.obs_grid[o_i][1], 'Obs'+str(o_i), fontsize=20, ha='center', bbox=dict(facecolor='red', alpha=0.8))

    for step_i in range(len(env.ore_order)):
        step_grid = copy.deepcopy(env.grid_act)
        step_obs_grid = copy.deepcopy(env.obs_grid)
        for ore_i in range(step_i+1):
            step_obs_grid.remove(env.obs_grid[env.ore_order[ore_i]])
        for i in range(len(step_obs_grid)):
            step_grid = CUF.obstacle_circle(step_grid, [round(step_obs_grid[i][0], 2), round(step_obs_grid[i][1], 2), obs_r[i]], 2)
        for ci in range(len(can_info)):
            xi, yi = can_info[ci].grid
            step_grid = CUF.obstacle_circle(step_grid, [xi, yi, 0.04], 3)

        step_grid = CUF.obstacle_circle(step_grid, [env.tar_grid[0], env.tar_grid[1], tar_r], 4)  # target

        CUF.draw_grid_info(step_grid)
        for cf_i in range(len(t_b[step_i])):
            plt.text((t_cf_pos[step_i][cf_i][0]-env.ws_zero[0])*G2P_SIZE, (t_cf_pos[step_i][cf_i][1]-env.ws_zero[1])*G2P_SIZE, 'b='+str(t_b[step_i][cf_i]), fontsize=20, ha='center', bbox=dict(facecolor='pink', alpha=0.8))
        for o_i in range(len(env.obs_grid)):
            plt.text(env.obs_grid[o_i][0], env.obs_grid[o_i][1], 'Obs' + str(o_i), fontsize=20, ha='center', bbox=dict(facecolor='red', alpha=0.8))
        plt.title('step'+str(step_i)+" obs: "+str(env.ore_order[step_i])+" rearranged")

    plt.show()