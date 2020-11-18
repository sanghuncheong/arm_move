import numpy as np
import matplotlib.pyplot as plt
import copy
import timeit
import time

from D_0828_envClass4altest import EnvInfo as EI
from D_0828_envClass4altest import CanInfo as CI

# from VFHplus_change_radius import influence
# from tree_making_no_plot import tree_making as TM_noplot
# from tree_making_plot import tree_making as TM_plot
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

    ws_w, ws_d = 50, 80  # get table size in the v-rep
    ws_cen = [0.75, 0.00]
    rob_pos = [0.0, 0.0]
    OBJ_R = 0.035

    env = EI(rob_pos, ws_w, ws_d, ws_cen, grid_size=GRID_SIZE, wall_r=OBJ_R)

    # for n_obs_max in range(10, 15+1):
    for n_obs_max in range(10, 12):
        for test_i in range(10):
            obs_r = []
            rearr_cnt = 0
            for i in range(n_obs_max):
                obs_r.append(0.035)
            # obs_r = [0.035, 0.035, 0.035, 0.035, 0.035, 0.035, 0.035]
            # obs_r = [0.035, 0.035, 0.035, 0.035]
            tar_r = 0.035
            min_ore = 3

            order_time = 0
            order_time2 = 0
            order_cnt = 0

            space_err = 0
        # while not space_err:    # this loop will go until space error occurs
            env.get_env(obs_r, tar_r, min_ore)
            algorithm_start = timeit.default_timer()
            algorithm_start2 = time.clock()
            env.get_max_can(env.grid_ori, bt_num=1, trial_num=1000)  # We get "grid_max_can", "can_grid"

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
            rearr_ore = len(env.ore_order)
            # for i in range(len(can_info)):
            #     print "can", i, ":", can_info[i].pos

            # CUF.draw_grid_info(env.grid_ori)
            # CUF.draw_grid_info(env.grid_del)
            # CUF.draw_grid_info(env.grid_max_can)
            #
            # plt.show()
            CUF.draw_grid_info(env.grid_ori)
            for c_i in range(len(can_info)):
                plt.text(can_info[c_i].grid[0], can_info[c_i].grid[1], 'Can' + str(c_i), fontsize=20, ha='center', bbox=dict(facecolor='pink', alpha=0.8))
            for o_i in range(len(env.obs_grid)):
                plt.text(env.obs_grid[o_i][0], env.obs_grid[o_i][1], 'Obs' + str(o_i), fontsize=20, ha='center', bbox=dict(facecolor='red', alpha=0.8))

            end_t_cp_flag = 0
            print "\n Test", test_i, "order:", len(env.ore_order), env.ore_order, "# can:", len(can_info), "# obs:", len(obs_r)
            while len(env.ore_order):   # this while loop is for the algorithm
                                        # algorithm will go on until it can access to target

                print "\niter:", rearr_cnt
                print "OR:", env.ore_order
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

                s_v = []
                s_v_index = []
                for i in range(len(env.ore_order)):
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
                        for ore_i in range(i+1):
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

                    s_e = []    # s_e: extra candidate spots

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
                    t_s_r = []   # s_r: candidate spot relocate plan
                    in_can_info = copy.deepcopy(t_s_e)
                    tmp_order_time_start = timeit.default_timer()
                    tmp_order_time_start2 = time.clock()
                    t_s_r = env.get_c_ore(in_can_info)
                    tmp_order_time_end = timeit.default_timer()
                    tmp_order_time_end2 = time.clock()
                    order_time = order_time + tmp_order_time_end - tmp_order_time_start
                    order_time2 = order_time2 + tmp_order_time_end2 - tmp_order_time_start2
                    order_cnt = order_cnt + 100*len(t_s_e)
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
                            print "can", s_e_index[i], "grid:", s_e[i].grid, ", s_r:", s_r[i]
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
                        in_obs_pos = copy.deepcopy(env.obs_pos)

                        print "in_obs_pos:", in_obs_pos
                        sorted_min_s_r = copy.deepcopy(min_s_r)
                        sorted_min_s_r.sort(reverse=True)
                        print "sorted min_s_r:", sorted_min_s_r
                        if sorted_min_s_r[0] == n_obs_max+1:  #if OR' has o_t ! remove s_e
                            print "o_t is in OR'"
                            s_e.remove(s_e[s_r.index(min_s_r)])
                            s_e_index.remove(s_e_index[s_r.index(min_s_r)])
                            s_r.remove(s_r[s_r.index(min_s_r)])
                        else:
                            for ore_i in range(len(min_s_r)):  # after rearrange all OR'
                                in_obs_pos.remove(in_obs_pos[sorted_min_s_r[ore_i]])
                            in_tar_pos = copy.deepcopy(cp.pos)
                            t_can_add = env.get_can_BT(in_can_info, in_obs_pos, in_tar_pos)

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
                    # print "no possible extra candidate exist"
                    break

                #move obstacle to can(min(b))
                # print "s_v", s_v
                # print "s_v[0]", s_v[0]
                # print "s_v[0][0]", s_v[0][0]
                # print "s_v[0][0].pos", s_v[0][0].pos
                print "t_b[0]", t_b[0]


                find_b = copy.deepcopy(t_b[0])
                # print "move to c_", find_b.index(min(find_b))
                t_sel_can_index = [i for i in range(len(find_b)) if find_b[i] == (min(find_b))]
                t_sel_can_dist = []
                # print "\ntar grid: ", env.tar_grid
                # print "\ntar pos: ", env.tar_pos
                print "t sel can index", t_sel_can_index
                for i in range(len(t_sel_can_index)):
                    # print "t_cf grid x,y:", t_sel_can_index[i], t_cf[0][t_sel_can_index[i]].grid[0], t_cf[0][t_sel_can_index[i]].grid[1]
                    # print "t_cf pos x,y:", t_sel_can_index[i], s_v[0][t_sel_can_index[i]].pos[0], s_v[0][t_sel_can_index[i]].pos[1]
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

                tmp_order_time_start = timeit.default_timer()
                tmp_order_time_start2 = time.clock()
                env.update_env(env.obs_pos, env.obs_grid)
                tmp_order_time_end = timeit.default_timer()
                tmp_order_time_end2 = time.clock()
                order_time = order_time + tmp_order_time_end - tmp_order_time_start
                order_time2 = order_time2 + tmp_order_time_end2 - tmp_order_time_start2
                order_cnt = order_cnt + 1

                rearr_cnt = rearr_cnt + 1
                if env.order_error_flag == 0:
                    print "\nretry for another environment"
                    space_err = 1
                    break

                print "after move order is:", env.ore_order

                CUF.draw_grid_info(env.grid_ori)
                for c_i in range(len(can_info)):
                    plt.text(can_info[c_i].grid[0], can_info[c_i].grid[1], 'Can' + str(c_i), fontsize=20, ha='center', bbox=dict(facecolor='pink', alpha=0.8))
                for o_i in range(len(env.obs_grid)):
                    plt.text(env.obs_grid[o_i][0], env.obs_grid[o_i][1], 'Obs' + str(o_i), fontsize=20, ha='center', bbox=dict(facecolor='red', alpha=0.8))


                if len(env.ore_order) == 0:
                    print "end rearrangement"
                #     plt.title('rearrangement finished')
                    break
                # else:
                #     plt.title('after rearrngement')

                # plt.show()

            algorithm_end = timeit.default_timer()
            algorithm_end2 = time.clock()
            tot_time = algorithm_end - algorithm_start
            algorithm_time = tot_time - order_time

            tot_time2 = algorithm_end2 - algorithm_start2
            algorithm_time2 = tot_time2 - order_time2

            print "test:", test_i, ",n_obs:", len(obs_r), ",order:", rearr_ore, ",actual rearr:", rearr_cnt
            print "tot time:", tot_time, "\norder time:", order_time, "cnt", order_cnt, "\nalg time:", algorithm_time
            print "tot time2:", tot_time2, "\norder time2:", order_time2, "cnt", order_cnt, "\nalg time2:", algorithm_time2

            plt.show()
