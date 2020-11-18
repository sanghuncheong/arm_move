import numpy as np
import matplotlib.pyplot as plt
import copy
import time
from S_0701_VFH_accCheck import influence

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
            return grid_list, emp_g[ran_c]


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


def get_obstacle_re(ob, target_ori, obs_pos_in, Body_position, d_max):
    obstacle_rearr = []
    obs_pos = copy.deepcopy(obs_pos_in)
    vfh, km = influence(ob, target_ori, obs_pos, Body_position, d_max)
    if vfh == 1:
        print("no need to rearrange")
        return 0
    while 1:
        ob = len(obs_pos)
        vfh, km = influence(ob, target_ori, obs_pos, Body_position, d_max)
        if vfh == 1:
            # obstacle_rearr.append(obs_pos[km[0][1]])
            # obstacle_rearr.append(target_ori)
            print("find way to rearrange \n list is:", obstacle_rearr)
            return obstacle_rearr
        elif vfh == 0:
            print("we have to rearrange:", obs_pos[km[0][1]])
            obstacle_rearr.append(obs_pos[km[0][1]])
            target_ori = obs_pos[km[0][1]]
            obs_pos.remove(obs_pos[km[0][1]])


if __name__ == '__main__':

    ws_w, ws_d = 80, 50
    # GRID_SIZE = 0.01

    grid_acc = np.zeros([ws_w, ws_d])
    grid_acc = mark_edge_grid(grid_acc)
    r_list = []
    obs_n = 9
    for i in range(obs_n):
        r_list.append(0.035)
    while 1:
        o_g = []
        # r_list = [0.035, 0.035, 0.035, 0.035, 0.035, 0.035, 0.035]
        tr = 0.035
        grid_env = copy.deepcopy(grid_acc)
        for ri in r_list:
            print("try to make a circle", ri)
            grid_env, cp = place_circle_object_ig(copy.deepcopy(grid_env), ri, 2)
            o_g.append(cp)
        print("picked obstacles in grid", o_g)
        # get target object
        grid_env, t_g = place_circle_object_ig(copy.deepcopy(grid_env), tr, 4)
        # draw_grid_info(grid_acc)
        # plt.show()
        t_p = [t_g[0] * GRID_SIZE, t_g[1] * GRID_SIZE]   # target object!
        r_p = [np.shape(grid_env)[0] * GRID_SIZE * 0.5, -0.2]
        d_max = 0.8

        ob = len(o_g)
        o_p = []
        for i in o_g:
            xi, yi = i
            o_p.append([xi * GRID_SIZE, yi * GRID_SIZE])
        ''' 
            This part is to delete the obstacles and get the candidates!
        '''
        ore_g = get_obstacle_re(ob, t_p, o_p, r_p, d_max)

        if ore_g == 0:
            print("retry")
        else:
            '''for selecting more rearr'''
            if len(ore_g) < 3:
                ore_g = 0
            else:
                print("environment setting OK")
                break

    print("\ntarget grid", t_g)
    print("obstacle grid list", o_g)
    print("rearrange obstacle list", ore_g)

    r_remove = []
    r_can = []
    for i in range(len(ore_g)):
        r_remove.append(o_g.index([int(ore_g[i][0] * G2P_SIZE), int(ore_g[i][1] * G2P_SIZE)]))
        print("remove [", ore_g[i][0] * G2P_SIZE, ore_g[i][1] * G2P_SIZE, "]")
        o_g.remove([int(ore_g[i][0] * G2P_SIZE), int(ore_g[i][1] * G2P_SIZE)])
    for i in r_remove:
        r_can.append(r_list.pop(i))

    # draw_grid_info(grid_env)

    grid_del = copy.deepcopy(grid_acc)
    print("obstacle grid", o_g)
    for i in range(len(r_list)):
        # print("draw a circle", [o_g[i][0], o_g[i][1], r_list[i]])
        grid_del = copy.deepcopy(obstacle_circle(grid_del, [int(o_g[i][0]), int(o_g[i][1]), ri], 2))
    grid_del = copy.deepcopy(obstacle_circle(grid_del, [t_g[0], t_g[1], tr], 4))  # target

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
    circle_center = []
    print("max r of the rearrange needed obstacles:", r_can, max(r_can))
    circle_r = max(r_can)+0.005
    print("candidates radius", circle_r)
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
                circle_center.append(empt_grid[pick_cen])
                for em in range(len(empt_grid)):
                    d_w = empt_grid[pick_cen][0] - empt_grid[em][0]
                    d_d = empt_grid[pick_cen][1] - empt_grid[em][1]
                    d_c = (d_w * d_w + d_d * d_d) ** 0.5 * GRID_SIZE
                    if d_c <= circle_r:
                        grid_can[empt_grid[em][0]][empt_grid[em][1]] = 3
                        grid_can[empt_grid[pick_cen][0]][empt_grid[pick_cen][1]] = 3
                        occu_grid.append([empt_grid[em][0], empt_grid[em][1]])
        bt_circle.append([circle_center, grid_can])

    max_cir_num = []
    for i in range(len(bt_circle)):
        print(i, "bt c", len(bt_circle[i][0]))
        max_cir_num.append([len(bt_circle[i][0])])

    print(max_cir_num.index(max(max_cir_num)))
    max_trial = max_cir_num.index(max(max_cir_num))

    grid_max_can = copy.deepcopy(bt_circle[max_trial][1])
    circle_center = bt_circle[max_trial][0]

    #  function that returns grid_max_can and circle_center

    # print "grid info \n", grid_info
    if len(circle_center) > 0:
        print("\n========================\n")
        print(circle_center)

    end_al = time.time()

    ob = len(o_g)
    o_p = []
    for i in o_g:
        xi, yi = i
        o_p.append([xi * GRID_SIZE, yi * GRID_SIZE])

    # vfh, km = influence(ob, t_p, o_p, r_p, d_max)
    # print("vfh", vfh, "obstacle to remove:", km)
    # draw_grid_info(grid_max_can)
    # plt.show()

    print("\ncheck if candidate occlude the target")
    can_pan = []
    for i in circle_center:
        xi, yi = i
        ob = len(o_g) + 1
        obs_pos1 = copy.deepcopy(o_p)
        obs_pos1.append([xi * GRID_SIZE, yi * GRID_SIZE])
        vfh, km = influence(ob, t_p, obs_pos1, r_p, d_max)
        # print("vfh", vfh)
        if vfh == 0:
            print("candidate", i, "occludes the target", t_p)
            print("candidate penalty")
            can_pan.append(i)

    print("penalty", can_pan)
    print("circle before penalty", circle_center)
    for i in can_pan:
        circle_center.remove(i)
    print("circle after penalty", circle_center)

    print("\ncheck if the candidate occlude the obstacle to remove")
    can_pan = []
    # obstacle_re = [[50, 10]]
    for i in circle_center:
        for oi in ore_g:
            xi, yi = i
            target = [oi[0] * GRID_SIZE, oi[1] * GRID_SIZE]
            ob = len(o_g) + 1
            obs_pos1 = copy.deepcopy(o_p)
            obs_pos1.append([xi * GRID_SIZE, yi * GRID_SIZE])
            vfh, km = influence(ob, target, obs_pos1, r_p, d_max)
            if vfh == 0:
                print("candidate", i, "occludes the obstacle to remove", oi)
                print("candidate penalty")
                can_pan.append(i)

    print("penalty", can_pan)
    print("circle before penalty", circle_center)
    for i in can_pan:
        circle_center.remove(i)
    print("circle after penalty", circle_center)

    print("\ncheck if the candidate cannot be rearranged")
    can_pan = []
    for i in circle_center:
        xi, yi = i
        target = [xi * GRID_SIZE, yi * GRID_SIZE]
        ob = len(o_g) + 1
        obs_pos1 = copy.deepcopy(o_p)
        obs_pos1.append(t_p)
        vfh, km = influence(ob, target, obs_pos1, r_p, d_max)
        if vfh == 0:
            print("obstacle", xi*GRID_SIZE, yi*GRID_SIZE, "can not be rearranged")
            print("candidate penalty")
            can_pan.append(i)

    print("penalty", can_pan)
    print("circle before penalty", circle_center)
    for i in can_pan:
        circle_center.remove(i)
    print("circle after penalty", circle_center)

    for i in circle_center:
        xi, yi = i
        grid_val_can = copy.deepcopy(obstacle_circle(grid_del, [xi, yi, 0.04], 3))  # target

    print("obs list", ob)
    print("circle packing time =", end_al - start_al)

    print("\ncheck the occ for the candidates")
    '''
    variable
    o_p : obstacle positions in list
    ore_g : obstacle to be rearranged
    '''


    '''
    check from the grid!
    '''
    draw_grid_info(grid_env)
    draw_grid_info(grid_del)
    draw_grid_info(grid_max_can)
    if len(circle_center) == 0:
        print("There are no candidates!")
    else:
        draw_grid_info(grid_val_can)



    plt.show()