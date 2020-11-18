import numpy as np
import matplotlib.pyplot as plt
import copy
import time

from D_0808_envClass import EnvInfo as EI
from D_0808_envClass import CanInfo as CI


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

    obs_r = [0.035, 0.035, 0.035, 0.035, 0.035, 0.035, 0.035, 0.035, 0.035, 0.035]
    # obs_r = [0.035, 0.035, 0.035, 0.035, 0.035, 0.035, 0.035]
    # obs_r = [0.035, 0.035, 0.035, 0.035]
    tar_r = 0.035
    min_ore = 2

    env.get_env(obs_r, tar_r, min_ore)
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
    if 1:
        print "\n# of obstacles", len(env.obs_pos), "\n# of candidates", len(env.can_pos)

    '''
    GET candidates info
    '''
    t_ore_order = copy.deepcopy(env.ore_order)

    CUF.draw_grid_info(env.grid_ori)
    CUF.draw_grid_info(env.grid_del)
    CUF.draw_grid_info(env.grid_max_can)
    #
    # plt.show()
    while 1:    # this while loop is for the algorithm
        # Check C.A : just next step
        t_can_info = []
        for i in range(len(env.ore_order)):
            in_can_info = copy.deepcopy(can_info)
            in_obs_pos = copy.deepcopy(env.obs_pos)
            for ore_i in range(i + 1):
                in_obs_pos.remove(env.obs_pos[env.ore_order[ore_i]])
            t_can_info.append(env.get_can_A(in_can_info, in_obs_pos, env.tar_pos))

        # Check C.BT
        for i in range(len(env.ore_order)):
            in_can_info = copy.deepcopy(t_can_info[i])
            in_obs_pos = copy.deepcopy(env.obs_pos)
            for ore_i in range(len(env.ore_order)):     # after rearrange all ORE
                in_obs_pos.remove(env.obs_pos[env.ore_order[ore_i]])
            t_can_info[i] = env.get_can_BT(in_can_info, in_obs_pos, env.tar_pos)

        # Check C.BO : BO : other ORE, just before target
        for i in range(len(env.ore_order)):
            in_can_info = copy.deepcopy(t_can_info[i])
            in_obs_pos = copy.deepcopy(env.obs_pos)
            for ore_i in range(len(env.ore_order)):     # after rearrange all ORE
                in_obs_pos.remove(env.obs_pos[env.ore_order[ore_i]])
            for j in range(len(env.ore_order)):     # check other ORE just before target
                if j > i:
                    t_can_info[i] = env.get_can_BT(in_can_info, in_obs_pos, env.obs_pos[env.ore_order[j]])

        t_cf = []
        t_cf_index = []
        for i in range(len(env.ore_order)):
            in_can_info = copy.deepcopy(t_can_info[i])
            ret_can, ret_index = env.get_cf(in_can_info)
            t_cf.append(ret_can)
            t_cf_index.append(ret_index)
            print "\n step", i, " has # of cf pos:", len(t_cf[i]), "index", t_cf_index[i]
            # print "t_cf", type(t_cf), type(t_cf[0]), t_cf[0], t_cf[0][0].pos

        # See if this case if case0 or case1
        if len(t_cf[0]) >= len(env.ore_order):
            t_b = []
            for i in range(1):
                in_obs_pos = copy.deepcopy(env.obs_pos)
                for ore_i in range(i+1):
                    in_obs_pos.remove(env.obs_pos[env.ore_order[ore_i]])
                t_b.append(env.get_cf_b(t_cf[i], in_obs_pos))
                print "\n step", i, " has cf b:", t_b[i]

            draw_figs = 1
            if draw_figs == 1:
                for c_i in range(len(can_info)):
                    plt.text(can_info[c_i].grid[0], can_info[c_i].grid[1], 'Can' + str(c_i), fontsize=20, ha='center', bbox=dict(facecolor='pink', alpha=0.8))
                for o_i in range(len(env.obs_grid)):
                    plt.text(env.obs_grid[o_i][0], env.obs_grid[o_i][1], 'Obs' + str(o_i), fontsize=20, ha='center', bbox=dict(facecolor='red', alpha=0.8))

                for step_i in range(1):
                    step_grid = copy.deepcopy(env.grid_act)
                    step_obs_grid = copy.deepcopy(env.obs_grid)
                    for ore_i in range(step_i + 1):
                        step_obs_grid.remove(env.obs_grid[env.ore_order[ore_i]])
                    for i in range(len(step_obs_grid)):
                        step_grid = CUF.obstacle_circle(step_grid, [round(step_obs_grid[i][0], 2), round(step_obs_grid[i][1], 2), env.obs_r[i]], 2)
                    for ci in range(len(can_info)):
                        xi, yi = can_info[ci].grid
                        step_grid = CUF.obstacle_circle(step_grid, [xi, yi, 0.04], 30)

                    step_grid = CUF.obstacle_circle(step_grid, [env.tar_grid[0], env.tar_grid[1], tar_r], 4)  # target

                    for cf_i in range(len(t_b[step_i])):
                        xi = (t_cf[step_i][cf_i].pos[0] - env.ws_zero[0]) * G2P_SIZE
                        yi = (t_cf[step_i][cf_i].pos[1] - env.ws_zero[1]) * G2P_SIZE
                        step_grid = CUF.obstacle_circle(step_grid, [xi, yi, 0.04], 3)

                    CUF.draw_grid_info(step_grid)

                    for cf_i in range(len(t_b[step_i])):
                        xi = (t_cf[step_i][cf_i].pos[0] - env.ws_zero[0]) * G2P_SIZE
                        yi = (t_cf[step_i][cf_i].pos[1] - env.ws_zero[1]) * G2P_SIZE
                        plt.text(xi, yi, 'b=' + str(t_b[step_i][cf_i]), fontsize=20, ha='center', bbox=dict(facecolor='pink', alpha=0.8))
                    for ci in range(len(t_can_info[step_i])):
                        plt.text(t_can_info[step_i][ci].grid[0], t_can_info[step_i][ci].grid[1] - 2.0, '[A, BT] :' + str([t_can_info[step_i][ci].A, t_can_info[step_i][ci].BT]), fontsize=10, ha='center', bbox=dict(facecolor='pink', alpha=0.8))
                    for o_i in range(len(env.obs_grid)):
                        plt.text(env.obs_grid[o_i][0], env.obs_grid[o_i][1], 'Obs' + str(o_i), fontsize=20, ha='center', bbox=dict(facecolor='red', alpha=0.8))
                    plt.title('step' + str(step_i) + " obs: " + str(env.ore_order[step_i]) + " rearranged")
        elif len(t_cf[0]) < len(env.ore_order):
            print "Since we meet condition: N(CF) < N(ORE) by", len(t_cf[0]), "<", len(env.ore_order), ",\nwe have to remove additional obstacles."
            # step1 : "get t_cp", check candidates which have A = 0 and BT = 0
            # This means that a candidate is not reachable and it does not block the target object

            # Check A for this environment state
            in_can_info = copy.deepcopy(can_info)
            in_obs_pos = copy.deepcopy(env.obs_pos)
            t_can_add = copy.deepcopy(env.get_can_A(in_can_info, in_obs_pos, env.tar_pos))

            # Check C.BT for this environment state
            in_can_info = copy.deepcopy(t_can_add)
            in_obs_pos = copy.deepcopy(env.obs_pos)
            for ore_i in range(len(env.ore_order)):  # after rearrange all ORE
                in_obs_pos.remove(env.obs_pos[env.ore_order[ore_i]])
            t_can_add = env.get_can_BT(in_can_info, in_obs_pos, env.tar_pos)

            t_cp = []
            t_cp_index = []

            in_can_info = copy.deepcopy(t_can_add)
            ret_can, ret_index = env.get_cp(in_can_info)
            t_cp = ret_can
            t_cp_index = ret_index
            print "\n Our Cp:", t_cp, len(t_cp), "index", t_cp_index

            # step2 : check c_ore for each cp and pick min of it
            c_ore = []
            in_can_info = copy.deepcopy(t_cp)
            c_ore = env.get_c_ore(in_can_info)

            print "cp", t_cp, "c_ore", c_ore

            min_c_ore = CUF.min_len_list(c_ore)
            print "c ore:", c_ore, "min c ore:", min_c_ore
            print "picked ci index:", t_cp.index(t_cp[c_ore.index(min_c_ore)])
            print "picked ci address:", copy.deepcopy(t_cp[c_ore.index(min_c_ore)])
            cp = copy.deepcopy(t_cp[c_ore.index(min_c_ore)])
            print "cp", cp

            # step3 : "get t_cf", check candidates which have A = 1 and BT' = 0
            # Check A for this environment state T' is t_cp_i
            in_can_info = copy.deepcopy(can_info)
            # in_can_info.remove(in_can_info[t_cp_index[t_cp.index(t_cp[c_ore.index(min_c_ore)])]])
            in_obs_pos = copy.deepcopy(env.obs_pos)
            in_tar_pos = copy.deepcopy(cp.pos)
            t_can_add = copy.deepcopy(env.get_can_A(in_can_info, in_obs_pos, env.tar_pos))

            # Check C.BT for this environment state
            in_can_info = copy.deepcopy(t_can_add)
            in_obs_pos = copy.deepcopy(env.obs_pos)
            for ore_i in range(len(min_c_ore)):  # after rearrange all ORE
                in_obs_pos.remove(env.obs_pos[min_c_ore[ore_i]])
            in_tar_pos = copy.deepcopy(cp.pos)
            t_can_add = env.get_can_BT(in_can_info, in_obs_pos, in_tar_pos)

            t_cf = []
            t_cf_index = []

            in_can_info = copy.deepcopy(t_can_add)
            ret_can, ret_index = env.get_cf(in_can_info)
            t_cf.append(ret_can)
            t_cf_index.append(ret_index)
            print "\n Our Cf':", t_cf[0], len(t_cf[0]), "index", t_cf_index[0]

            # See if this case if case0 or case1
            if len(t_cf[0]) >= len(min_c_ore) - 1:
                if len(min_c_ore) == 1:
                    t_cf = []
                    t_cf_index = []
                    t_cf.append([cp])
                    t_cf_index.append([t_cp_index[t_cp.index(t_cp[c_ore.index(min_c_ore)])]])
                    t_b = [[0]]
                    print "\n In case1, len(min c ore) = 1 has cf b:", t_b[0]
                    print "t_cf_index:", t_cf_index, "t_cf_index[0]:", t_cf_index[0]
                else:
                    t_b = []
                    in_obs_pos = copy.deepcopy(env.obs_pos)
                    for ore_i in range(1):
                        in_obs_pos.remove(env.obs_pos[min_c_ore[ore_i]])
                    t_b.append(env.get_cf_b(t_cf[0], in_obs_pos))
                    print "\n In case1, len(min c ore) > 1 has cf b:", t_b[0]
                    print "t_cf_index:", t_cf_index, "t_cf_index[0]:", t_cf_index[0]
            env.ore_order = min_c_ore
            # while 1:    # This will be a loop!
            #     # Get cp which is candidates with potential
            #     if len(t_can_add) == 0:
            #         print "\n There is not enough space for rearrangement"
            #         end_al_flag = 1
            #         break  # end process
            #     break   # end while loop for additional rearrangement

            # if end_al_flag == 1:
            #     print "\n Since there is not enough space for rearrangement,\nwe end process"
            #     break  # end while loop for the algorithm.
            draw_figs = 1
            if draw_figs == 1:
                for c_i in range(len(can_info)):
                    plt.text(can_info[c_i].grid[0], can_info[c_i].grid[1], 'Can' + str(c_i), fontsize=20, ha='center', bbox=dict(facecolor='pink', alpha=0.8))
                for o_i in range(len(env.obs_grid)):
                    plt.text(env.obs_grid[o_i][0], env.obs_grid[o_i][1], 'Obs' + str(o_i), fontsize=20, ha='center', bbox=dict(facecolor='red', alpha=0.8))

                step_i = 0
                step_grid = copy.deepcopy(env.grid_act)
                step_obs_grid = copy.deepcopy(env.obs_grid)
                step_obs_grid.remove(env.obs_grid[env.ore_order[0]])
                print "step_obs_grid", step_obs_grid
                for i in range(len(step_obs_grid)):
                    print "i:", i, "step_obs_grid [i]:", step_obs_grid[i]
                    step_grid = CUF.obstacle_circle(step_grid, [round(step_obs_grid[i][0], 2), round(step_obs_grid[i][1], 2), env.obs_r[i]], 2)
                for ci in range(len(can_info)):
                    xi, yi = can_info[ci].grid
                    step_grid = CUF.obstacle_circle(step_grid, [xi, yi, 0.04], 30)

                step_grid = CUF.obstacle_circle(step_grid, [env.tar_grid[0], env.tar_grid[1], tar_r], 4)  # target

                for cf_i in range(len(t_b[step_i])):
                    xi = (t_cf[step_i][cf_i].pos[0] - env.ws_zero[0]) * G2P_SIZE
                    yi = (t_cf[step_i][cf_i].pos[1] - env.ws_zero[1]) * G2P_SIZE
                    step_grid = CUF.obstacle_circle(step_grid, [xi, yi, 0.04], 3)

                CUF.draw_grid_info(step_grid)

                for cf_i in range(len(t_b[step_i])):
                    xi = (t_cf[step_i][cf_i].pos[0] - env.ws_zero[0]) * G2P_SIZE
                    yi = (t_cf[step_i][cf_i].pos[1] - env.ws_zero[1]) * G2P_SIZE
                    plt.text(xi, yi, 'b=' + str(t_b[step_i][cf_i]), fontsize=20, ha='center', bbox=dict(facecolor='pink', alpha=0.8))
                for ci in range(len(t_can_info[step_i])):
                    plt.text(t_can_info[step_i][ci].grid[0], t_can_info[step_i][ci].grid[1] - 2.0, '[A, BT] :' + str([t_can_info[step_i][ci].A, t_can_info[step_i][ci].BT]), fontsize=10, ha='center', bbox=dict(facecolor='pink', alpha=0.8))
                for o_i in range(len(env.obs_grid)):
                    plt.text(env.obs_grid[o_i][0], env.obs_grid[o_i][1], 'Obs' + str(o_i), fontsize=20, ha='center', bbox=dict(facecolor='red', alpha=0.8))
                plt.title('step' + str(step_i) + " obs: " + str(env.ore_order[step_i]) + " rearranged")



        for i in range(len(t_cf[0])):
            print "\ncf: grid", t_cf[0][i].grid

        #move obstacle to can(min(b))
        find_b = copy.deepcopy(t_b[0])
        print "move to c_", find_b.index(min(find_b))
        t_sel_can_index = [i for i in range(len(find_b)) if find_b[i] == (min(find_b))]
        print "t sel can index", t_sel_can_index
        t_sel_can_dist = []
        print "\ntar grid: ", env.tar_grid
        print "\ntar pos: ", env.tar_pos
        for i in range(len(t_sel_can_index)):
            print "t_cf grid x,y:", t_sel_can_index[i], t_cf[0][t_sel_can_index[i]].grid[0], t_cf[0][t_sel_can_index[i]].grid[1]
            print "t_cf pos x,y:", t_sel_can_index[i], t_cf[0][t_sel_can_index[i]].pos[0], t_cf[0][t_sel_can_index[i]].pos[1]
            t_sel_can_dist.append(np.sqrt((env.tar_pos[0]-t_cf[0][t_sel_can_index[i]].pos[0])**2 + (env.tar_pos[1]-t_cf[0][t_sel_can_index[i]].pos[1])**2))
        print "t sel can dist", t_sel_can_dist
        sel_can_index = t_sel_can_index[t_sel_can_dist.index(max(t_sel_can_dist))]
        print "sel can index", sel_can_index

        sel_can_pos = can_info[t_cf_index[0][sel_can_index]].pos
        sel_can_grid = can_info[t_cf_index[0][sel_can_index]].grid

        sel_obs_pos = env.obs_pos[env.ore_order[0]]
        sel_obs_grid = env.obs_grid[env.ore_order[0]]

        env.obs_pos[env.ore_order[0]] = sel_can_pos
        env.obs_grid[env.ore_order[0]] = sel_can_grid

        can_info[t_cf_index[0][sel_can_index]].pos = sel_obs_pos
        can_info[t_cf_index[0][sel_can_index]].grid = sel_obs_grid

        env.update_env(env.obs_pos, env.obs_grid)
        print "after move order is:", env.ore_order
        CUF.draw_grid_info(env.grid_ori)
        for c_i in range(len(can_info)):
            plt.text(can_info[c_i].grid[0], can_info[c_i].grid[1], 'Can' + str(c_i), fontsize=20, ha='center', bbox=dict(facecolor='pink', alpha=0.8))
        for o_i in range(len(env.obs_grid)):
            plt.text(env.obs_grid[o_i][0], env.obs_grid[o_i][1], 'Obs' + str(o_i), fontsize=20, ha='center', bbox=dict(facecolor='red', alpha=0.8))

        if len(env.ore_order) == 0:
            print "end rearrangement"
            plt.title('rearrangement finished')
            break
        else:
            plt.title('after rearrngement')

    plt.show()