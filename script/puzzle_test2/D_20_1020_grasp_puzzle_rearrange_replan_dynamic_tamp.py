#!/usr/bin/env python

import sys
import numpy as np
import random as rn
import time
import copy
import networkx as nx
import matplotlib
import matplotlib.pyplot as plt
import tmi_relocate as mp
import VFHplus_immobile as vfh
import D_0723_relocate_package as rp


# distance(x, y): compute the Euclidean distance between 2D points x and y
def distance(point_one, point_two):
    return ((point_one[0] - point_two[0]) ** 2 +
            (point_one[1] - point_two[1]) ** 2) ** 0.5

# unique_list(sequence): remove duplicated elements in the input list 'sequence'
def unique_list(seq):
    seen = set()
    seen_add = seen.add
    return [x for x in seq if not (x in seen or seen_add(x))]


if __name__ == "__main__":
    ### BEGIN Initialization ###
    # plot_on = False
    plot_on = True

    plot_compare_on = True
    #plot_compare_on = False

    plan_result = False
    execution_on = False#True
    # motion_planning_on = False
    motion_planning_on = True
    time_out = False

    if motion_planning_on:
        mp.go_home()
        mp.go_ready()
        mp.hand_open()
        mp.table_AL_test()
    # N: the number of all objects including the target
    #N = 8, 12, 16, 20 objects including the target
    N = 12
    #N = 12
    # h, r: mean values of the random heights and radii of objects
    #h = 0.07
    h = 0.095
    # r = 0.027
    r = 0.023
    #r = 0.022
    robot_height = 0.075

    # miscellaneous parameters
    min_len = 10000
    min_weight = min_len * 10
    no_trials = 10

    edges_add = []
    edges_all = []
    objects = []
    obstacles = []
    walls = []

    # Generation of N random objects
    c = 0
    iter = 0
    max_iter = 10000

    # Generate random heights and radii of objects
    H = []
    for i in range(0, N):
        H.append(round(h + 0.001 * rn.randint(-5, 6), 3))
    H.append(robot_height)

    R = []
    for i in range(0, N):
        R.append(round(r + 0.001 * rn.randint(-2, 3), 3))#U(0.025, 0.03)
    maxR = max(R)
    R.append(maxR)

    # Generate random 2D poses of objects without overlapping
    X_size = 0.8
    X_lb = -0.40
    X_ub = 0.40

    Y_size = 0.45
    Y_lb = 0.8637 + 0.03
    Y_ub = 1.3137 + 0.03

    resize = 1.0
    Y_0 = []
    X_0 = []
    X_all = []
    Y_all = []
    x_s0 = X_lb + resize * (X_size/6.0) #5 objects
    x_s = X_lb + resize * (X_size/10.0) #5 objects
    y_s = Y_lb + resize * (Y_size/8.0) #4 rows

    x_interval0 = resize * (X_size/6.0)*2
    x_interval = resize * (X_size/10.0)*2
    y_interval = resize * (Y_size/8.0)*2 #4 rows

    x_noise0 = 50
    x_noise = 60
    y_noise = 20

    #for i in range(0, 3):
    #    X_0.append(x_s0 + x_interval0 * i + 0.001 * rn.randint(-x_noise0, x_noise0))

    for i in range(0, 4):
        for j in range(0, 5):
            X_all.append(x_s + x_interval * j + 0.001 * rn.randint(-x_noise, x_noise))

    for i in range(0, 4):
        for j in range(0, 5):
            Y_all.append(y_s + y_interval * (i) + 0.001 * rn.randint(-y_noise, y_noise))

    Y_all[2]= Y_all[2] + 0.04
    Y_all[7]= Y_all[7] + 0.04
    Y_all[12]= Y_all[12] + 0.04
    Y_all[16]= 1.127
    Y_all[17]= Y_all[17] + 0.04

    # Correct values outside the table size
    for i in range(0, len(X_all)):
        if X_all[i] < X_lb + maxR:
            X_all[i] = X_lb + maxR
        elif X_all[i] > X_ub - maxR:
            X_all[i] = X_ub - maxR
        if Y_all[i] < Y_lb + maxR:
            Y_all[i] = Y_lb + maxR
        elif Y_all[i] > Y_ub - maxR:
            Y_all[i] = Y_ub - maxR

    #N = 5, 9, 13, 17 => 6, 10, 14, 18 objects including the target
    X = []
    Y = []
    targets = []
    if N == 8:
        pose_candidates = [5, 6, 7, 8, 9, 11, 12, 13, 16, 17, 18]
        target_candidates = [11, 12, 13, 16, 17, 18]
        pose_idx_list = rn.sample(list(range(0, len(pose_candidates))), N)
        for i in range(0, len(pose_idx_list)):
            pose_idx = pose_candidates[pose_idx_list[i]]
            X.append(X_all[pose_idx])
            Y.append(Y_all[pose_idx])
            if pose_idx in target_candidates:
                targets.append(i)
    elif N == 12:
        pose_candidates = [5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 18]
        target_candidates = [10, 11, 12, 13, 14, 16, 17, 18]
        pose_idx_list = rn.sample(list(range(0, len(pose_candidates))), N)
        for i in range(0, len(pose_idx_list)):
            pose_idx = pose_candidates[pose_idx_list[i]]
            X.append(X_all[pose_idx])
            Y.append(Y_all[pose_idx])
            if pose_idx in target_candidates:
                targets.append(i)
    elif N == 16:
        pose_candidates = [0, 1, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19]
        target_candidates = [10, 11, 12, 13, 14, 15, 16, 17, 18, 19]
        pose_idx_list = rn.sample(list(range(0, len(pose_candidates))), N)
        for i in range(0, len(pose_idx_list)):
            pose_idx = pose_candidates[pose_idx_list[i]]
            X.append(X_all[pose_idx])
            Y.append(Y_all[pose_idx])
            if pose_idx in target_candidates:
                targets.append(i)
    elif N == 20:
        pose_candidates = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19]
        target_candidates = [10, 11, 12, 13, 14, 15, 16, 17, 18, 19]
        #target_candidates = [4, 5, 6]
        pose_idx_list = rn.sample(list(range(0, len(pose_candidates))), N)
        for i in range(0, len(pose_idx_list)):
            pose_idx = pose_candidates[pose_idx_list[i]]
            X.append(X_all[pose_idx])
            Y.append(Y_all[pose_idx])
            if pose_idx in target_candidates:
                targets.append(i)
    else:
        sys.exit('Wrong N: N should be one of 8, 12, 16, 20')

    # X = [-0.033, 0.374, 0.34299999999999997, 0.22600000000000003, -0.318, 0.358, -0.17099999999999999, 0.20200000000000004, -0.185, -0.011, -0.012, -0.148, -0.323, -0.417, -0.415, 0.394, -0.011, 0.13200000000000006, -0.16299999999999998, 0.19500000000000006]
    # Y = [1.1187500000000001, 0.76425, 1.10075, 0.97325, 1.07975, 0.85475, 0.77625, 0.76025, 0.86875, 1.01925, 0.78825, 1.1199999999999999, 0.75225, 0.9622499999999999, 0.87375, 0.98825, 0.9177500000000001, 1.09975, 0.97925, 0.85275]
    # R = [0.026, 0.028, 0.03, 0.029, 0.03, 0.026, 0.027, 0.025, 0.03, 0.03, 0.027, 0.029, 0.029, 0.027, 0.026, 0.027, 0.029, 0.025, 0.028, 0.029, 0.03]
    # targets = [3]

    # Generating bounding box (Wall)
    x_min = -0.40
    x_max =  0.40
    y_min =  0.8637 + 0.03
    y_max =  1.3137 + 0.03

    X_t = list(np.linspace(x_min, x_max, int(np.ceil((x_max - x_min)/(2*r)))))
    Y_t = list(np.linspace(y_min, y_max, int(np.ceil((y_max - y_min)/(2*r)))))

    X_w = [x_min]*len(Y_t) + X_t
    X_w = X_w + [x_max]*len(Y_t)
    Y_w = Y_t + [y_max]*len(X_t)
    Y_w = Y_w + Y_t

    X_r = ['%.2f' % elem for elem in X]
    Y_r = ['%.2f' % elem for elem in Y]

    print("X_r, Y_r")
    print(X_r)
    print(Y_r)

    # print("X = ", X)
    # print("Y = ", Y)
    print("R = ", R)


    M = len(X_w)
    R_wall = [np.sqrt(r**2 + r**2)] * M
    for i in range(0, M):
        walls.append([X_w[i], Y_w[i]])

    #camera_pose = [0.0, 0.45]
    camera_pose = [0.0, -0.45]
    robot_pose = list(camera_pose)

    for i in range(0, N):
        objects.append([X[i], Y[i]])
    objects.append(robot_pose)

    objects_init = copy.deepcopy(objects)
    all_objects = list(objects)
    all_objects.pop(-1)
    all_R = list(R)
    all_R.pop(-1)
    all_nodes_mp = list(range(0, N)) #exclude the robot hand node


    # N+1 because the robot base pose is also a node
    all_nodes = list(range(0, N+1))

    ### Task planning ###
    target = targets[rn.randint(0, len(targets) - 1)]
    target_x = objects[target][0]
    target_y = objects[target][1]

    task_time_tic = time.time()

    G, edges_add = rp.graph_construction(target, objects, walls, all_nodes, N, M, R, R_wall)
    accessible_nodes = rp.get_acc_obj(G, N)
    print('targets = [', target, ']')
    print("Accessible: ", accessible_nodes)

    # Find the min-hop path bewteen each pair of a visible (accessible) object and the target
    path = rp.relocation_path(target, objects, N, G, min_len, min_weight)
    print("Path: ", path)

    print("X_r, Y_r")
    print(X_r)
    print(Y_r)
    if plot_on:
        rp.plot_figure(target, path, all_nodes, objects, walls, X, Y, X_w, Y_w, N, M, R, R_wall[0], G, edges_add)

    if len(path) > 1:
        relocate = copy.deepcopy(path)# save things to relocate
        relocate.pop(-1)# remove the target id from the list
        relocate_seq = []
        relocated = []
        while len(relocate) > 0 and len(relocated) < no_trials:
            ### 3. Find buffer nodes ###
            X_e, Y_e, combined_space, buffer_nodes, R, L = rp.buffer_sampling(objects, R, X_w, Y_w, N, M, max_iter, x_s, y_s, X_ub, Y_ub, maxR)

            # A. Terminate if it is not solvable (#B < k)
            if L < 1:#len(path):
                sys.exit('Infeasible: not enough buffers for relocation')

            # Check Accessible Buffers (AB)
            accessible_buffer_nodes, _ = rp.find_accessible_buffers(objects, path, combined_space, all_nodes, buffer_nodes, walls, R, R_wall, N, M, maxR, camera_pose, [])

            # Check if placing objects in the AB node obstructs other AB nodes
            valid_buffer_nodes = rp.find_valid_buffers(combined_space, accessible_buffer_nodes, maxR, camera_pose)
            print('%d valid buffer(s) found' % len(valid_buffer_nodes))

            if plot_on:
                rp.plot_figure_buffer(target, path, all_nodes, objects, walls, X, Y, X_w, Y_w, N, M, L, R, R_wall[0], G, edges_add,
                                      edges_all, list(range(0, N)) + [100] * len(buffer_nodes), buffer_nodes, accessible_buffer_nodes, valid_buffer_nodes, combined_space,
                                      relocate, [], maxR, camera_pose)


            if len(valid_buffer_nodes) < 1:#Now we have no valid (red) buffer---all gray buffer. Try to obtain valid buffers (red)
                print('*No valid buffer. Rearrange to acquire additional valid buffers.*')
                valid_buffer_acquired = False
                accessible_nodes = rp.get_acc_obj(G, N)
                node_idx = 0
                while not valid_buffer_acquired:
                    if node_idx >= len(accessible_nodes):
                       sys.exit('Cannot acquire additional buffers')
                    objects_rearrange = copy.deepcopy(objects)
                    combined_space_rearrange = copy.deepcopy(combined_space)

                    #remove node_idx-th accessible nodes (sending back)
                    objects_rearrange[accessible_nodes[node_idx]] = [0.0, 10.0]
                    combined_space_rearrange[accessible_nodes[node_idx]] = [0.0, 10.0]

                    # Check #AB
                    accessible_buffer_nodes_prev, _ = rp.find_accessible_buffers(objects_rearrange, path, combined_space_rearrange, all_nodes,
                                                                            buffer_nodes, walls, R, R_wall, N, M, maxR, camera_pose, [])

                    # Check if placing objects in the AB node obstructs other AB nodes
                    valid_buffer_nodes_prev = rp.find_valid_buffers(combined_space_rearrange, accessible_buffer_nodes_prev, maxR, camera_pose)
                    valid_buffer_nodes_new = copy.deepcopy(valid_buffer_nodes_prev)

                    for buffer_idx in valid_buffer_nodes_new:
                        #virtually place the object to the buffer_idx (in the prev config)
                        objects_rearrange[accessible_nodes[node_idx]] = combined_space_rearrange[buffer_idx]
                        combined_space_rearrange[accessible_nodes[node_idx]] = combined_space_rearrange[buffer_idx]
                        combined_space_rearrange[buffer_idx] = objects[accessible_nodes[node_idx]]
                        # Check #AB
                        accessible_buffer_nodes_new, _ = rp.find_accessible_buffers(objects_rearrange, path, combined_space_rearrange, all_nodes,
                                                                                buffer_nodes, walls, R, R_wall, N, M, maxR, camera_pose, [])

                        # Check if placing objects in the AB node obstructs other AB nodes
                        valid_buffer_nodes_new = rp.find_valid_buffers(combined_space_rearrange, accessible_buffer_nodes_new, maxR, camera_pose)
                        if valid_buffer_nodes_new:
                            # physically place the object to the buffer_idx (in the prev config)

                            #if motion_planning_on:
                            #    mp.del_objects(all_nodes_mp) #just to make sure all objects are removed
                            #    mp.add_objects(objects, all_nodes_mp, all_R)
                            #    mp.motion_planning_picking: 1. move the hand from the home pose to the object at "objects_rearrange[accessible_nodes[node_idx]]" 2. attach the object (ID: accessible_nodes[node_idx]) and back to the home pose
                            #    mp.motion_planning_placing: 1. move the hand and the object from the home pose to "combined_space_rearrange[buffer_idx]" 2. detach the object and back to the home pose

                            objects = copy.deepcopy(objects_rearrange)
                            combined_space = copy.deepcopy(combined_space_rearrange)
                            valid_buffer_acquired = True
                            break #break the for loop if any valid buffer is found
                        print('%d valid buffer(s) found' % len(valid_buffer_nodes_new))
                    node_idx = node_idx + 1

                if plot_on:
                    rp.plot_figure_buffer(target, path, all_nodes, objects, walls, X, Y, X_w, Y_w, N, M, L, R, R_wall[0], G,
                                          edges_add, edges_all, list(range(0, N)) + [100] * len(buffer_nodes), buffer_nodes,
                                          accessible_buffer_nodes, valid_buffer_nodes, combined_space, relocate, [], maxR, camera_pose)

            else:#if len(valid_buffer_nodes) >= 1:
                # B. If #AB >=k, Do A* search.
                # After every expansion, update #AB and compare to the current k
                # Cost = #(obstacles not removed yet from O_R)---no need to consider where they are placed
                goal_found = 0
                objects_expand = copy.deepcopy(objects)
                current_state = list(range(0, N)) + [100] * len(buffer_nodes)  # ex) [3, 1, 2, 100, 100, ..., 100]
                search_queue = []
                p_cost = 1#path cost

                cost = 10000
                dist = -10000
                for i in valid_buffer_nodes:
                    # Expansion (save the sequence for every expansion)
                    frontier_state = copy.deepcopy(current_state)  # (N + len(accessible_buffer_nodes))
                    object_expand = relocate[0]
                    object_index = current_state.index(object_expand)
                    empty_index = i
                    frontier_state[empty_index] = object_expand #relocate the first object in path to buffer i
                    frontier_state[object_index] = -100  # the location where the relocated object was placed is now empty, with unknown accessibility. Check later once expansion is done
                    # 100: unoccupied accessible node, -100: unoccupied path node (accessibility not checked yet)
                    h_cost = 0#heuristic cost
                    for j in relocate:
                        path_index = current_state.index(j) # for each object in the path
                        if abs(frontier_state[path_index]) != abs(100): # check if the location where the object was placed is emptied.
                            h_cost = h_cost + 1 # if not emptied, the heuristic cost gets a unit score (penalized).
                    expanded_cost = p_cost + h_cost
                    expanded_distance = distance(objects[target], combined_space[i])
                    search_queue.append(frontier_state)
                    if cost > expanded_cost: #find the min-cost expansion: the most objects in the path are cleared.
                        dist = expanded_distance
                        cost = expanded_cost
                        expanded_state = copy.deepcopy(frontier_state)
                        object_index_done = object_index
                        empty_index_done = empty_index
                    elif cost == expanded_cost and dist < expanded_distance: # if there's a tie, relocate to the farthest buffer
                        dist = expanded_distance
                        expanded_state = copy.deepcopy(frontier_state)
                        object_index_done = object_index
                        empty_index_done = empty_index

                current_state = copy.deepcopy(expanded_state)

                # Update "objects, empty_space, buffer_nodes" variable to reflect the expansion
                objects_expand[object_index_done] = combined_space[empty_index_done]
                buffer_nodes[buffer_nodes.index(empty_index_done)] = object_index_done

                if motion_planning_on:
                   motion_picking = False
                   motion_placing = False
                   mp.del_objects(all_nodes_mp) #just to make sure all objects are removed
                #
                   motion_picking = mp.motion_planning_picking(objects, all_nodes_mp, all_R, object_index_done)
                   #: 1. move the hand from the home pose to the object at "objects_expand[object_index_done]", 2. attach the object (ID: objects_index_done), 3. back to the home pose
                   if motion_picking:
                      motion_placing = mp.motion_planning_placing(objects, all_nodes_mp, all_R, object_index_done, combined_space[empty_index_done])
                      #1. move the hand and the object from the home pose to "combined_space[empty_index_done]", 2. detach the object, 3. back to the home pose
                #    else:
                #        #Remove the edge from R to the object, and then continue (going to the beginning of the current while loop and check the condition)
                #        G.remove_edges_from([(object_index_done, N), (N, object_index_done)])
                #        path = rp.relocation_path(target, objects, N, G, min_len, min_weight)
                #        if not path:
                #            sys.exit('No path to the target.')
                #        print("New path: ", path)
                #        relocate = path.copy()
                #        relocate.pop(-1)
                #        if not relocate:
                #            sys.exit('Nothing to relocate.')
                #        continue
                #    if motion_placing:
                #        #Execute pick & place
                #    else:
                #        #Just continue because new buffers will be sampled again
                #        #Do not use this line (just for reference): valid_buffer_nodes.remove(empty_index_done)
                #        continue

                objects = copy.deepcopy(objects_expand)
                relocate_seq.append([object_index_done, empty_index_done])#object actually moved from object_index_done to empty_index_done
                print('Relocate Obj %d -> Node %d' % (object_index_done, empty_index_done))

                relocated.append(relocate.pop(0))

                if plot_on:
                    rp.plot_figure_buffer(target, path, all_nodes, objects, walls, X, Y, X_w, Y_w, N, M, L, R, R_wall[0], G, edges_add,
                                          edges_all, current_state, buffer_nodes, accessible_buffer_nodes, valid_buffer_nodes, combined_space,
                                          relocate, relocated, maxR, camera_pose)

                #### Reflect the relocation result: update the graph and path
                G, edges_add = rp.graph_construction(target, objects, walls, all_nodes, N, M, R, R_wall)
                path = rp.relocation_path(target, objects, N, G, min_len, min_weight)
                if not path:
                    sys.exit('No path to the target.')
                print("New path: ", path)
                relocate = copy.deepcopy(path)
                relocate.pop(-1)
                if not relocate:
                    break

        ### END of the algorithm
        print('** Final result **')
        for i in range(0, len(relocate_seq)):
            print('Relocate Obj %d -> Node %d' % (relocate_seq[i][0], relocate_seq[i][1]))
        print('Total %d objects relocated.' % (len(relocate_seq)))

        if G.has_edge(N, target):
            print("Finished. Target accessible. Rearrangement succeeded!")
        elif len(relocated) >= no_trials:
            print("Couldn't finish. Rearrangement failed!")
        else:
            print("Finished but target not accessible. Rearrangement failed!")
        task_time = time.time() - task_time_tic
        print("Task planning time:", task_time, "sec")
        print("Task planning time per object:", task_time/len(relocate_seq), "sec")
        #print("Motion planning time:", motion_time, "sec")
        #print("Total planning time:", task_time + motion_time, "sec for", relocated_objs, "objects.",
        #      (task_time + motion_time) / (relocated_objs), "sec per object.", replanning_occurrence, "times of replanning")

    else:
        sys.exit('Target directly accessible. Nothing to relocate.')
        #if plot_on:
        #    rp.plot_figure(target, path, all_nodes, objects, walls, X, Y, X_w, Y_w, N, M, R, R_wall[0], G, edges_add)
        ### Relocation ###
        #plan_result, relocated_ids = rp.relocation_planning(target, target_x, target_y, path, all_nodes, objects, walls,
        #                                                    edges_add, X, Y, X_w, Y_w, N, M, R, R_wall, G, H, robot_pose,
        #                                                    robot_height, min_len, min_weight, plot_on, motion_planning_on,
        #                                                    task_time, time_out)


    if plot_on:
        rp.plot_figure(target, [], all_nodes, objects, walls, X, Y, X_w, Y_w, N, M, R, R_wall[0], G, edges_add)
        #rp.plot_figure_buffer(target, path, all_nodes, objects, walls, X, Y, X_w, Y_w, N, M, L, R, R_wall[0], G, edges_add,
                              #edges_all, current_state, buffer_nodes, accessible_buffer_nodes, valid_buffer_nodes, combined_space,
                              #relocate, relocated, maxR, camera_pose)

    if plot_compare_on:
        rp.plot_figure_compare(target, [], all_nodes, objects_init, objects, walls, X, Y, X_w, Y_w, N, M, R, R_wall[0])

    ################## Execution of the plan
    '''
    if execution_on and plan_result:
        relocated = True

        all_nodes = list(range(0, N))
        objects = list(all_objects)
        if motion_planning_on:
            mp.del_objects(all_nodes)
            mp.add_objects(all_objects, all_nodes, all_R)

        all_nodes.append(N)
        objects.append(robot_pose)
        R = list(all_R)
        R.append(maxR)

        executed_ids = []
        while relocated_ids and relocated:
            goal_id = relocated_ids.pop(0)
            print("Execute to remove Object", goal_id)
            objects_ids = list(set(all_nodes) - set([N]) - set(executed_ids))
            obstacle_ids = list(set(objects_ids) - set([goal_id]))

            d_max = distance(objects[goal_id], robot_pose)
            radius = R[goal_id]
            _, _, _, collision_free, approaching_angle = vfh.influence(len(objects_ids) + M - 1, objects[goal_id],
                                                                       [objects[k] for k in obstacle_ids] + walls,
                                                                       robot_pose, d_max,
                                                                       [R[k] for k in obstacle_ids] + R_wall, radius, 1)
            if not collision_free:
                sys.exit("Execution failed: not accessible")
            print((approaching_angle + 180.0) % 360.0)
            if motion_planning_on:
                relocated, _ = mp.action_execution(objects[goal_id], goal_id, np.deg2rad((approaching_angle + 180.0) % 360.0))  # goal_pose[2] - np.pi)
            else:
                relocated = True
            if not relocated:
                sys.exit("Execution failed: no execution")
            else:
                executed_ids.append(goal_id)
            #time.sleep(10)
    '''