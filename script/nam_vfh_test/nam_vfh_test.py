# Planning for removing obstacles to grasp an unknown target object in clutter
# Author: Changjoo Nam (cjnam@kist.re.kr)
# Date: 10/29/2018
# Description: A certain target in a known and static environment

import sys
import numpy as np
import random as rn
import networkx as nx
import VFHplus_mobile as vfh

import copy

import matplotlib
import matplotlib.pyplot as plt

# distance(x, y): compute the Euclidean distance between 2D points x and y
def distance(point_one, point_two):
    return ((point_one[0] - point_two[0]) ** 2 +
            (point_one[1] - point_two[1]) ** 2) ** 0.5

# unique_list(sequence): remove duplicated elements in the input list 'sequence'
def unique_list(seq):
    seen = set()
    seen_add = seen.add
    return [x for x in seq if not (x in seen or seen_add(x))]

def find_ORE(n_ob, obs_pos, obs_r, wall_min_max, rob_pos):
    ### BEGIN Initialization ###
    # N: the number of all objects including the target
    #N = 6, 10, 14, 18 objects including the target

    ### Modify (# of objects including the target)
    N = n_ob
    # h, r: mean values of the random heights and radii of objects
    h = 0.07
    r = 0.030
    robot_height = 0.075

    # miscellaneous parameters
    min_len = 10000
    min_weight = min_len * 10

    edges_add = []
    edges_all = []
    objects = []
    obstacles = []
    walls = []

    # Generation of N random objects
    c = 0
    iter = 0
    max_iter = 10000

    ### Modify (radius)
    R = copy.deepcopy(obs_r)

    ### Modify (objects x, y cooridnates, X = [0.2, 0.4, ], Y = X = [0.2, 0.4, ])
    ### For algorithm tests
    # X = [0] * N
    # Y = [0] * N
    # for i in range(0, N):
    #     newCircleFound = False
    #     while not newCircleFound and c <= N and iter < max_iter:
    #         # The number (r*1.35) represents the minimum distance between circles (from a centroid to a centroid)
    #         x = r * 1.5 * rn.randint(0, round((N + 2)/2))/(N**0.02)
    #         y = r * 1.5 * rn.randint(0, round((N + 2)/2))/(N**0.02)
    #         #x = r*1.35 * rn.randint(0, round((N+1)/2))
    #         #y = r*1.35 * rn.randint(0, round((N+1)/2))
    #         if i == 0:
    #             prevCirclesX = 0
    #             prevCirclesY = 0
    #             distFromPrevCircles = [np.sqrt(((prevCirclesX - x)**2 + (prevCirclesY - y)**2))]
    #         else:
    #             prevCirclesX = X[0:i]
    #             prevCirclesY = Y[0:i]
    #             distFromPrevCircles = np.matrix.tolist(np.sqrt(np.square(np.matrix(prevCirclesX[0:i]) - np.matrix([x]*i))
    #                                     +  np.square(np.matrix(prevCirclesY[0:i]) - np.matrix([y]*i))))[0]
    #
    #         if i == 0 or (min(distFromPrevCircles) <= 2*max(R)) == False:
    #             newCircleFound = True
    #             c = c + 1
    #             X[i] = x
    #             Y[i] = y
    #         iter = iter + 1
    #
    #         if iter >= max_iter:
    #             sys.exit('Not enough space!')

    # Generating bounding box (Wall)
    x_min = wall_min_max[0]
    x_max = wall_min_max[1]
    y_min = wall_min_max[2]
    y_max = wall_min_max[3]
    X_t = list(np.linspace(x_min, x_max, int(np.ceil((x_max - x_min)/(2*r)))))
    Y_t = list(np.linspace(y_min, y_max, int(np.ceil((y_max - y_min)/(2*r)))))

    X_w = [x_min]*len(Y_t) + X_t
    X_w = X_w + [x_max]*len(Y_t)
    Y_w = Y_t + [y_max]*len(X_t)
    Y_w = Y_w + Y_t
    #
    M = len(X_w)

    R_wall = [np.sqrt(r ** 2 + r ** 2)] * M
    for i in range(0, M):
        walls.append([X_w[i], Y_w[i]])

    # Camera pose:
    #   x: middle of the space
    #   y: bottom of the space
    camera_pose = rob_pos
    robot_pose = rob_pos

    objects = copy.deepcopy(obs_pos)

    # Create an empty graph
    G = nx.Graph()
    # N+1 because the robot base pose is also a node
    all_nodes = list(range(0, N+1))
    ### END Initialization ###
    G.add_nodes_from(all_nodes)


    ### BEGIN Planning ###

    # Connect edges of the graph using VFH+
    #  Description: for each pair of node i and node j, check if an edge (i, j) can be connected between them (i is not equal to j)
    #               Note that (i, j) and (j, i) are different (directed edges a.k.a. "arrows")
    #               (i ,j) connected?: the end-effector can move any object from Object i's pose to Object j's pose without collision (if Objects i and j are removed)
    edges_add = []
    nodes_done = []
    for i in all_nodes:
        # Exclude myself (node i) and previously checked nodes
        nodes_wo_me = list(set(all_nodes) - set([i]) - set(nodes_done))
        nodes_done.append(i)
        for j in nodes_wo_me:
            # end_pose (the end-effector pose):
            # the end-effector is located at node j's pose (picking from node j's location-> go to i's location)
            end_pose = objects[j]
            edge = (i, j) #A path i to j: any object can move from i to j and j to i
            # VFH+ checks within d_max (between Object i and the end-effector)
            d_max = distance(objects[i], end_pose)
            # Except i and j, other objects are regarded as obstacles
            obstacles_sub = list(set(all_nodes) - set([i, j]) - set([N]))
            # radius (to compute the augmented radius): choose the largest radius among all objects because any object in the scene should be able to move between i and j without collision
            radius = max(R)
            # Run VFH+ (in VFHplus_mobile.py): the wall is included as obstacles
            _, _, _, collision_free = vfh.influence(len(all_nodes) - 2 + M - 1, objects[i], [objects[k] for k in obstacles_sub] + walls, end_pose, d_max, [R[k] for k in obstacles_sub] + R_wall, radius)
            _, _, _, collision_free_r = vfh.influence(len(all_nodes) - 2 + M - 1, objects[j], [objects[k] for k in obstacles_sub] + walls, objects[i], d_max, [R[k] for k in obstacles_sub] + R_wall, radius)
            if collision_free == 1 and collision_free_r == 1:
                edges_add.append(edge)
                # N is the robot node. Exclude from the final path
                if i != N and j != N:
                    edges_all.append(edge)
    G.add_edges_from(edges_add)

    # Find accessibile objects
    #  Description: the nodes connected to the robot node can be accessed by the robot since there are paths between the robot node and its neighbors
    all_sources = [N]#robot node
    accessible_nodes = list(G.neighbors(N))
    accessible_nodes.sort()

    # Nodes that are not accessible are target candidates (Assuming inaccessible == invisible)
    targets = list((set(all_nodes) - set([N])) - set(accessible_nodes))
    # Determine the target which is one of the inaccesible (invisible) objects in the beginning (randomly)
    target = targets[rn.randint(0, len(targets) - 1)]


    print'Accessible: ',
    print accessible_nodes
    print('Target: %d' % target)

    # Find the min-hop path bewteen each pair of a visible (accessible) object and the target
    for source in all_sources:
        if nx.has_path(G, source, target):
            # Find all min-hop paths (all ties)
            paths = list(nx.all_shortest_paths(G, source, target, weight=None))
            path_weights = [0] * len(paths)
            # Compute the Euclidean distance of each path for tie breaking
            for i in range(0, len(paths)):
                for j in range(0, len(paths[i]) - 1):
                    path_weights[i] = path_weights[i] + distance(objects[paths[i][j]], objects[paths[i][j + 1]])

            # Choose the path with the minimum Euclidean distance if there are multiple min-hop paths
            idx = path_weights.index(min(path_weights))
            if len(paths[idx]) < min_len:
                path = paths[idx]
                min_len = len(path)
                min_weight = min(path_weights)
            elif len(paths[idx]) == min_len and min(path_weights) < min_weight:
                path = paths[idx]
                min_len = len(path)
                min_weight = min(path_weights)

    # If there is a single path found (min_len remains 10000 if no path found), add it to the path explored until now
    if min_len < 10000:
        path.pop(0)#remove the robot node from the path
    else:
        sys.exit('No path found')

    ### END Planning ###

    # Delete node N (robot base) from the graph
    G.remove_node(N)

    # Print
    # print X
    # print Y

    print('\nSource node: %d' % (path[0]))
    print('Sink node: %d' % (target))

    print'Path: ',
    print path
    # Plot the final graph
    plt.figure(2)

    # Plot the configuration (Fig. 8a in the paper)
    plt.subplot(121)

    plt.gcf()
    ax = plt.gca()

    all_nodes = list(range(0, N))
    all_obstacles = list(set(all_nodes) - set([target]))
    for i in all_obstacles:
        obstacle_plot = plt.Circle((objects[i][0], objects[i][1]), R[i], color='r', clip_on = False)
        ax.add_artist(obstacle_plot)

    source_plot =  plt.Circle((objects[path[0]][0], objects[path[0]][1]), R[path[0]], color='gray', clip_on = False)
    ax.add_artist(source_plot)

    target_plot =  plt.Circle((objects[target][0], objects[target][1]), R[target], color='lime', clip_on = False)
    ax.add_artist(target_plot)

    for i in range(0, M):
        wall_plot = matplotlib.patches.Rectangle((walls[i][0] - 0.04, walls[i][1] - 0.04), 0.08, 0.08, color='sienna', clip_on=False)
        ax.add_patch(wall_plot)

    plt.plot()
    plt.axis('scaled')

    # plt.xlim(min(X_w)-max(R), max(X_w)+max(R))
    # plt.ylim(min(Y_w)-max(R), max(Y_w)+max(R))

    # Plot the graph (Fig. 8b in the paper)
    plt.subplot(122)

    pos = {}
    for i in all_nodes:
        pos[all_nodes[i]] = objects[i]

    labels = {}
    for node_name in all_nodes:
        labels[node_name] =str(node_name)

    nodes = nx.draw_networkx_nodes(G, pos, all_nodes, node_color = 'r', node_size = 500)
    nodes_t = nx.draw_networkx_nodes(G, pos, nodelist = [target], node_color = 'lime', node_size = 500)
    nodes_s = nx.draw_networkx_nodes(G, pos, nodelist = [path[0]], node_color = 'gray', node_size = 500)
    nodes_s.set_edgecolor('black')
    nodes.set_edgecolor('black')
    nodes_t.set_edgecolor('black')
    nx.draw_networkx_labels(G, pos, labels, font_size = 20)

    path_edges = []
    for i in range(0, len(path)-1):
        path_edges.append((path[i], path[i+1]))
    nx.draw_networkx_edges(G, pos, edgelist = edges_all, width = 0.5, arrowsize=20)
    nx.draw_networkx_edges(G, pos, edgelist = path_edges, width = 2, edge_color = 'red', arrowsize=20)

    # plt.xlim(min(X)-2*max(R), max(X)+2*max(R))
    # plt.ylim(min(Y)-2*max(R), max(Y)+2*max(R))
    plt.axis('off')
    plt.axis('scaled')
    plt.plot()

    plt.show()
    while True:
      try:
        plt.show()
      except UnicodeDecodeError:
        continue
      break