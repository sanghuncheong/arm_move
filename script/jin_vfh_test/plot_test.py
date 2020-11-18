def map_plotting(ob, target_position, target_orientation, obstacle_position, obstacle_orientation, Body_position, Jaco_tip_position, tree):

    import matplotlib
    import matplotlib.pyplot as plt
    from matplotlib.path import Path
    import matplotlib.patches as patches
    import numpy as np

    fig = plt.figure()
    ax = fig.add_subplot(111)
    #ax.grid()

    circle = matplotlib.patches.Circle((Jaco_tip_position[0], Jaco_tip_position[1]), radius=0.04, color='BLUE', lw=3, label="End-effector")
    ax.add_patch(circle)

    th_edge = 0.05
    tv_edge = 0.025

    verts = np.array([
        [target_position[0] - th_edge, target_position[1] - tv_edge],  # left, bottom
        [target_position[0] - th_edge, target_position[1] + tv_edge],  # left, top
        [target_position[0] + th_edge, target_position[1] + tv_edge],  # right, top
        [target_position[0] + th_edge, target_position[1] - tv_edge],  # right, bottom
        [0., 0.],  # ignored
    ])
    R = np.array([[np.cos(target_orientation[2]), -np.sin(np.sin(target_orientation[2]))], [np.sin(target_orientation[2]), np.cos(target_orientation[2])]])

    verts_new = [0] * len(verts)
    for i in range(len(verts)):
        verts[i] -= [target_position[0], target_position[1]]
        verts_new[i] = np.matmul(R, np.transpose(verts[i])) + [target_position[0], target_position[1]]

    codes = [Path.MOVETO, Path.LINETO, Path.LINETO, Path.LINETO, Path.CLOSEPOLY]
    path = Path(verts_new, codes)

    patch = patches.PathPatch(path, facecolor='green', lw=4, label="Target")
    ax.add_patch(patch)

    for i in range(ob):
        # R = np.array([[np.cos(obstacle_orientation[i][2]), -np.sin(np.sin(obstacle_orientation[i][2]))], [np.sin(obstacle_orientation[i][2]), np.cos(obstacle_orientation[i][2])]])
        # if i == 3 or i == 4 or i == 6 or i == 10 or i == 13:
        #     oh_edge = 0.05
        #     ov_edge = 0.02
        #     verts = np.array([
        #         [obstacle_position[i][0] - oh_edge, obstacle_position[i][1] - ov_edge],  # left, bottom
        #         [obstacle_position[i][0] - oh_edge, obstacle_position[i][1] + ov_edge],  # left, top
        #         [obstacle_position[i][0] + oh_edge, obstacle_position[i][1] + ov_edge],  # right, top
        #         [obstacle_position[i][0] + oh_edge, obstacle_position[i][1] - ov_edge],  # right, bottom
        #         [0., 0.],  # ignored
        #     ])
        #
        #     verts_new = [0] * len(verts)
        #     for j in range(len(verts)):
        #         verts[j] -= [obstacle_position[i][0], obstacle_position[i][1]]
        #         verts_new[j] = np.matmul(R, np.transpose(verts[j])) + [obstacle_position[i][0], obstacle_position[i][1]]
        #
        #     codes = [Path.MOVETO, Path.LINETO, Path.LINETO, Path.LINETO, Path.CLOSEPOLY]
        #     path = Path(verts_new, codes)
        #
        #     patch = patches.PathPatch(path, facecolor='purple', lw=0)
        #     ax.add_patch(patch)
        #
        # elif i == 7 or i == 8 or i == 9 or i == 11 or i == 14:
        #     circle = matplotlib.patches.Circle((obstacle_position[i][0], obstacle_position[i][1]), radius=0.04, color='magenta', lw=0)
        #     ax.add_patch(circle)
        #
        # else:
        #     circle = matplotlib.patches.Circle((obstacle_position[i][0], obstacle_position[i][1]), radius=0.035, color='RED', lw=0)
        #     ax.add_patch(circle)
        circle = matplotlib.patches.Circle((obstacle_position[i][0], obstacle_position[i][1]), radius=0.035, color='RED', lw=0)
        ax.add_patch(circle)

    ax.plot([Jaco_tip_position[0], obstacle_position[tree[0]][0]], [Jaco_tip_position[1], obstacle_position[tree[0]][1]], color='black', lw=5)
    ax.plot([Jaco_tip_position[0], obstacle_position[tree[0]][0]], [Jaco_tip_position[1], obstacle_position[tree[0]][1]], 'o', color='gold', ms=10, alpha=5)
    for i in range(len(tree) - 1):
        if i == len(tree) - 2:
            ax.plot([obstacle_position[tree[i]][0], target_position[0]], [obstacle_position[tree[i]][1], target_position[1]], color='black', lw=5)
            ax.plot([obstacle_position[tree[i]][0], target_position[0]], [obstacle_position[tree[i]][1], target_position[1]], 'o', color='gold', ms=10, alpha=5)
        else:
            ax.plot([obstacle_position[tree[i]][0], obstacle_position[tree[i + 1]][0]], [obstacle_position[tree[i]][1], obstacle_position[tree[i + 1]][1]], color='black', lw=5)
            ax.plot([obstacle_position[tree[i]][0], obstacle_position[tree[i + 1]][0]], [obstacle_position[tree[i]][1], obstacle_position[tree[i + 1]][1]], 'o', color='gold', ms=10, alpha=5)

    # ax.set_xlim(0.5, 1.25)
    # ax.set_ylim(-0.25, 0.35)
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    ax.legend()
    plt.show()