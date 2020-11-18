

def compute_empty_slots(ws_size, ws_center_pos, tar_pos, target_r, objs_pos, objs_r, n_relocate):
    import copy
    import numpy as np
    import S0116_custom_function as CUF
    from D_0909_envClass4altest import CanInfo as CI

    from VFHplus_change_radius import influence

    GRID_SIZE = 0.01
    ws_width = int(ws_size[0]*100)
    ws_depth = int(ws_size[1]*100)

    grid_init = np.zeros([ws_width, ws_depth])
    grid_act = CUF.mark_edge_grid(grid_init)
    for obj_i in range(len(objs_pos)):
        obj_i_grid = [round(ws_width/2 - (ws_center_pos[0] - objs_pos[obj_i][0])*100, 1), round(ws_depth/2 - (ws_center_pos[1] - objs_pos[obj_i][1])*100, 1)]
        print "obj", obj_i, "pos:", objs_pos[obj_i]
        obstacle_xyr_grid = [obj_i_grid[0], obj_i_grid[1], objs_r[obj_i]]
        grid_act = CUF.obstacle_circle(grid_act, obstacle_xyr_grid, 2)

    # Considering the y axis of the target while packing circles
    tar_grid = [round(ws_width/2 - (ws_center_pos[0] - tar_pos[0])*100, 1), round(ws_depth/2 - (ws_center_pos[1] - tar_pos[1])*100, 1)]
    print "Target pos:", tar_pos
    for x_i in range(ws_width):
        for y_i in range(int(target_r*100*1.5)):
            if int(tar_grid[1])+y_i < ws_depth and int(tar_grid[1])+y_i > 0:
                #print tar_grid[1], y_i
                grid_act[x_i][int(tar_grid[1])+y_i] = 2
                grid_act[x_i][int(tar_grid[1])-y_i] = 2
            else:
                print "out of plane"

    grid_ori = grid_act
    bt_num = 3
    trial_num = 2000

    bt_circle = []
    circle_r = target_r
    for bt in range(bt_num):
        can_grid = []
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

    max_trial = max_cir_num.index(max(max_cir_num))

    grid_max_can = copy.deepcopy(bt_circle[max_trial][1])
    t_can_grid = bt_circle[max_trial][0]

    can_pos = []
    #ws_zero = [round(ws_cen[0] - ws_width * GRID_SIZE * 0.5, 2), round(ws_cen[1] - ws_depth * GRID_SIZE * 0.5, 2)]
    ws_zero = [ws_center_pos[0] - ws_size[0] * 0.5, ws_center_pos[1] - ws_size[1] * 0.5]

    for i in t_can_grid:
        xi, yi = i
        can_pos.append([ws_zero[0] + xi * GRID_SIZE, ws_zero[1] + yi * GRID_SIZE])

    can_info = []
    for i in range(len(can_pos)):
        can_info.append((CI('candidate', can_pos[i])))

    # =============Next part=================
    # Check the feasibility of each candidate
    walls = CUF.get_obs_wall(ws_center_pos, ws_size, OBJ_R=0.04, GRID_SIZE=0.01)
    can_info, can_info_a_list = CUF.get_can_A(can_info, objs_pos, tar_pos, walls)
    print "\nAccessible check (accessible: 1)"
    print can_info_a_list

    # Check if the candidate blocks the target
    can_info, can_info_bt_list = CUF.get_can_BT(can_info, objs_pos, tar_pos, walls)
    print "\nBlocking Target check (blocking: 1)"
    print can_info_bt_list

    # Check the b value of all candidates
    vc_list, b_list = CUF.get_b(can_info, objs_pos, walls)
    print "\nB value check (not vs: -1)"
    print vc_list, b_list

    add_cans_pos = []
    for i in range(len(can_info)):
        if can_info[i].A == 0:
            add_cans_pos.append(can_info[i].pos)
    print "\n additional candidate"
    print add_cans_pos

    can_pos_list = []
    for i in range(len(vc_list)):
        can_pos_list.append([b_list[i], can_info[vc_list[i]].pos])
    print "\n Candidate pose list with b value"
    print can_pos_list
    # make valid candidate list

    return can_pos, grid_max_can, can_info, can_pos_list

if __name__ == '__main__':
    import matplotlib.pyplot as plt
    import S0116_custom_function as CUF
    print "====== rearrange test ======"
    ws_size = [0.3, 0.5]#[0.6, 0.25]
    ws_center_pos = [0.73, 0.0]
    tar_pos = [0.8, 0.0]
    target_r = 0.04
    objs_pos = [[0.75, -0.15], [0.75, 0.15]]
    objs_r = [0.04, 0.04, 0.04, 0.04]

    can_pos, grid_max_can, can_info, can_pos_list = compute_empty_slots(ws_size, ws_center_pos, tar_pos, target_r, objs_pos, objs_r, 2)
    print "final can pos:\n", len(can_pos), can_pos

    # wall = CUF.get_obs_wall(ws_center_pos, ws_size, OBJ_R=0.04, GRID_SIZE=0.01)
    wall = []
    print "num wall", len(wall)

    #========================================
    #============ ploting part ==============

    fs = 15
    new_fig = plt.figure(figsize=(fs, fs))

    ax = plt.gca()
    # change default range so that new disks will work
    plt.axis('equal')
    ax.set_xlim((-0.1, 2.0))
    ax.set_ylim((-0.75, 0.75))

    # ax.set_axis_off()
    patch_names = []

    # workspace = plt.Rectangle([ws_center_pos[0]-ws_size[0]*0.5, ws_center_pos[1]-ws_size[1]*0.5], ws_size[0], ws_size[1], color='k', fill=False)
    # patch_names.append(workspace)
    robot = plt.Circle([0, 0], 0.04, color='gray')
    patch_names.append(robot)

    for obs_i in objs_pos:
        obstacle = plt.Circle(obs_i, 0.04, color='red')
        patch_names.append(obstacle)

    for i in range(len(can_info)):
        if can_info[i].A == 1 and can_info[i].BT == 0:
            # if the candidate is accessible -> fill pink
            candidate = plt.Circle(can_info[i].pos, 0.04, color='pink', alpha=0.5)
        else:
            # if the candidate is not accessible -> no fill
            candidate = plt.Circle(can_info[i].pos, 0.04, color='pink', fill=False, linewidth=2, linestyle=':')
        patch_names.append(candidate)

    for c_i in range(len(can_info)):
        plt.text(can_info[c_i].pos[0], can_info[c_i].pos[1], 'Can' + str(c_i), fontsize=10, ha='center', bbox=dict(facecolor='pink', alpha=0.2))

    for wall_i in wall:
        wall = plt.Circle(wall_i, 0.04, color='gray', alpha=0.5)
        patch_names.append(wall)

    target = plt.Circle(tar_pos, 0.04, color='green')
    patch_names.append(target)

    for i in patch_names:
        plt.gca().add_patch(i)
    plt.show()
