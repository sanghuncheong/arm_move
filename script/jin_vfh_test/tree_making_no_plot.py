def tree_making(ob, target_position, target_orientation, obstacle_position, obstacle_orientation, Body_position, Jaco_tip_position, d_max):

    import VFHplus_change_radius as VP
    import object_orientation as OO
    import plot_test as PT
    from scipy.spatial import distance
    import timeit

    start = timeit.default_timer()

    target_p_real = [] + target_position
    target_o_real = [] + target_orientation
    obstacle_p_real = [] + obstacle_position
    obstacle_o_real = [] + obstacle_orientation

    tree_end = 0
    eta_block = 45
    eta_round = 45
    eta_cup = 25
    r_l = 0.09
    r_r = 0.035
    r_s = 0.025
    r_t = 0.035
    r_total = 2 * r_r + 2 * r_s + r_t

    ob_n = []

    for i in range(ob):
        ob_n.append(i)
    ob_n.append('T')

    tree = []

    dummy_position = OO.object_orientation(target_p_real, target_o_real)
    VFH = VP.influence(ob, target_p_real, obstacle_p_real, dummy_position[1], d_max, eta_block)
    km = VFH[6]

    area_first = []
    for i in range(ob):
        if km[i][0] < 2 * eta_block:
            area_first.append(km[i][1])

    dist_list_area = []
    for i in range(len(area_first)):
        dist = distance.euclidean(target_position, obstacle_position[area_first[i]])
        if dist <= r_total:
            dist_list_area.append([dist, area_first[i]])
    dist_list_area.sort()

    random_sequence = []
    for i in range(ob):
        random_sequence.append(i)

    dist_sequence_area = []
    for i in range(len(dist_list_area)):
        dist_sequence_area.append(dist_list_area[i][1])
        random_sequence.remove(dist_sequence_area[i])

    area_sequence = dist_sequence_area + random_sequence

    # print(dist_sequence_area)

    cycle = 0
    tmp = []
    if VFH[3] != 0:
        tmp.append('T')
        tree.append(tmp)
        tree_end = 1
        print("target can be catched")
        cycle += 1
        check_flag = 1

    for i in range(ob):
        temp = obstacle_position[area_sequence[i]]
        obstacle_position[area_sequence[i]] = target_position
        target_position = temp
        temp_orientation = obstacle_orientation[area_sequence[i]]
        obstacle_orientation[area_sequence[i]] = target_orientation
        target_orientation = temp_orientation

        # VFH_a = 0
        # VFH_b = 0
        # if area_sequence[i] == 3 or area_sequence[i] == 4 or area_sequence[i] == 6 or area_sequence[i] == 10 or area_sequence[i] == 13:
        #     dummy_position = OO.object_orientation(target_position, target_orientation)
        #     VFH = VP.influence(ob, target_position, obstacle_position, dummy_position[0], d_max, eta_block)
        #     VFH_a = VFH[3]
        #     VFH = VP.influence(ob, target_position, obstacle_position, dummy_position[1], d_max, eta_block)
        #     VFH_b = VFH[3]
        # elif area_sequence[i] == 7 or area_sequence[i] == 8 or area_sequence[i] == 9 or area_sequence[i] == 11 or area_sequence[i] == 14:
        #     dummy_position = OO.object_orientation(target_position, target_orientation)
        #     VFH = VP.influence(ob, target_position, obstacle_position, dummy_position[1], d_max, eta_cup)
        # else:
        #     VFH = VP.influence(ob, target_position, obstacle_position, Body_position, d_max, eta_round)
        VFH = VP.influence(ob, target_position, obstacle_position, Body_position, d_max, eta_round)
        tmp = []
        if VFH[3] != 0:
            tmp.append(area_sequence[i])
            tree.append(tmp)

        cycle += 1

    print("first tree", tree)

    target_position = [] + target_p_real
    target_orientation = [] + target_o_real
    obstacle_position = [] + obstacle_p_real
    obstacle_orientation = [] + obstacle_o_real

    pre_len = [0, len(tree)]
    m = 0

    while tree_end == 0:

        tree_first = []
        tree_second = []
        for i in range(pre_len[m], len(tree)):
            ob_re = [] + ob_n

            for j in range(len(tree[i])):
                ob_re.remove(tree[i][j])
            #print("line", m, tree[i], ob_re)

            ob_ex = []
            ob_ex_o = []
            for p in range(len(ob_re) - 1):
                ob_ex.append(obstacle_p_real[ob_re[p]])
                ob_ex_o.append(obstacle_o_real[ob_re[p]])

            d_max = distance.euclidean(target_p_real, obstacle_p_real[tree[i][len(tree[i])-1]]) + r_l
            dummy_position = OO.object_orientation(target_p_real, target_o_real)
            VFH = VP.influence(len(ob_ex), target_p_real, ob_ex, dummy_position[1], d_max, eta_block)

            km = VFH[6]

            area_first = []
            for q in range(len(ob_ex)):
                if km[q][0] < 2 * eta_block:
                    area_first.append(km[q][1])

            # print(ob_re[area_first[0]])
            dist_list_area = []
            for q in range(len(area_first)):
                dist = distance.euclidean(target_position, ob_ex[area_first[q]])
                if dist <= r_total:
                    dist_list_area.append([dist, area_first[q]])
            dist_list_area.sort()

            random_sequence = []
            for s in range(len(ob_ex)):
                random_sequence.append(s)

            dist_sequence_area = []
            for q in range(len(dist_list_area)):
                dist_sequence_area.append(dist_list_area[q][1])
            for q in range(len(dist_list_area)):
                random_sequence.remove(dist_sequence_area[q])

            area_sequence = dist_sequence_area + random_sequence
            # print(ob_re[area_sequence[0]])
            # print(area_sequence)

            tmp = []
            if VFH[3] != 0:
                tmp += tree[i]
                tmp.append('T')
                tree.append(tmp)
                tree_end = 1
                print("target can be catched")
                cycle += 1
                break

            for k in range(len(ob_ex)):
                temp = ob_ex[area_sequence[k]]
                ob_ex[area_sequence[k]] = target_position
                target_position = temp
                temp_orientation = ob_ex_o[area_sequence[k]]
                ob_ex_o[area_sequence[k]] = target_orientation
                target_orientation = temp_orientation

                # VFH_a = 0
                # VFH_b = 0

                d_max = distance.euclidean(target_position, obstacle_p_real[tree[i][len(tree[i])-1]]) + r_l
                # if ob_re[area_sequence[k]] == 3 or ob_re[area_sequence[k]] == 4 or ob_re[area_sequence[k]] == 6 or ob_re[area_sequence[k]] == 10 or ob_re[area_sequence[k]] == 13:
                #     dummy_position = OO.object_orientation(target_position, target_orientation)
                #     VFH = VP.influence(len(ob_ex), target_position, ob_ex, dummy_position[0], d_max, eta_block)
                #     VFH_a = VFH[3]
                #     VFH = VP.influence(len(ob_ex), target_position, ob_ex, dummy_position[1], d_max, eta_block)
                #     VFH_b = VFH[3]
                #
                # elif ob_re[area_sequence[k]] == 7 or ob_re[area_sequence[k]] == 8 or ob_re[area_sequence[k]] == 9 or ob_re[area_sequence[k]] == 11 or ob_re[area_sequence[k]] == 14:
                #     dummy_position = OO.object_orientation(target_position, target_orientation)
                #     VFH = VP.influence(len(ob_ex), target_position, ob_ex, dummy_position[1], d_max, eta_cup)
                #
                # else:
                #     VFH = VP.influence(len(ob_ex), target_position, ob_ex, Body_position, d_max, eta_round)
                VFH = VP.influence(len(ob_ex), target_position, ob_ex, Body_position, d_max, eta_round)

                tmp = []
                if VFH[3] != 0:
                    tmp += tree[i]
                    tmp.append(ob_re[area_sequence[k]])
                    if len(dist_sequence_area) != 0:
                        for q in range(len(dist_sequence_area)):
                            if ob_re[area_sequence[k]] == ob_re[dist_sequence_area[q]]:
                                tree_first_pre = tmp
                                tree_first.append(tree_first_pre)
                            else:
                                tree_second_pre = tmp
                                tree_second.append(tree_second_pre)
                    else:
                        tree_second_pre = tmp
                        tree_second.append(tree_second_pre)

                    # tree_first.append(tree_second)
                    # tree.append(tmp)

                temp = ob_ex[area_sequence[k]]
                ob_ex[area_sequence[k]] = target_position
                target_position = temp
                temp_orientation = ob_ex_o[area_sequence[k]]
                ob_ex_o[area_sequence[k]] = target_orientation
                target_orientation = temp_orientation

                cycle += 1

            target_position = [] + target_p_real
            target_orientation = [] + target_o_real
            obstacle_position = [] + obstacle_p_real
            obstacle_orientation = [] + obstacle_o_real

        if tree_end == 0:
            tree.extend(tree_first + tree_second)

        pre_len.append(len(tree))
        m = m + 1

    node_pre = 0
    for i in range(len(tree)):
        if len(tree[i]) < len(tree[len(tree)-1]):
            node_pre = node_pre + 1
    print("node_pre", node_pre)

    print("Total tree =", tree)
    print("last tree", tree[len(tree)-1])
    print("# of nodes", len(tree))
    print("the number of checking", cycle)


    stop = timeit.default_timer()
    print('process time =', stop - start)
    if check_flag == 1:
        return ['T']
    else:
        # PT.map_plotting(ob, target_position, target_orientation, obstacle_position, obstacle_orientation, Body_position, Jaco_tip_position, tree[len(tree)-1])
        dummy = 1
    return tree[len(tree)-1]