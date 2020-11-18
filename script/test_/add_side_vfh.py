GRID_SIZE = 0.01
OBJ_RADIUS = 0.035

import numpy as np
import matplotlib.pyplot as plt
import S_sh_custom_function as CF


if __name__ == "__main__":
    print "This file has list of custom made functions"
    CF.linspace2D()

    ws_w, ws_d = 50, 80  # get table size in the v-rep
    ws_cen = [0.75, 0.00]
    rob_pos = [0.0, 0.0]

    ws_zero = [round(ws_cen[0] - ws_w * GRID_SIZE * 0.5, 2), round(ws_cen[1] - ws_d * GRID_SIZE * 0.5, 2)]
    ws_point = []
    ws_point.append([ws_cen[0]-ws_w * GRID_SIZE*0.5, ws_cen[1]-ws_d * GRID_SIZE*0.5])  # left low point
    ws_point.append([ws_cen[0]+ws_w * GRID_SIZE*0.5, ws_cen[1]-ws_d * GRID_SIZE*0.5])  # right low point
    ws_point.append([ws_cen[0]+ws_w * GRID_SIZE*0.5, ws_cen[1]+ws_d * GRID_SIZE*0.5])  # right high point
    ws_point.append([ws_cen[0]-ws_w * GRID_SIZE*0.5, ws_cen[1]+ws_d * GRID_SIZE*0.5])  # left high point

    ws_side = []
    ws_side.append([ws_cen[0]-ws_w * GRID_SIZE*0.5, ws_cen[1]-ws_d * GRID_SIZE*0.5-OBJ_RADIUS])  # left low point
    ws_side.append([ws_cen[0]+ws_w * GRID_SIZE*0.5+OBJ_RADIUS, ws_cen[1]-ws_d * GRID_SIZE*0.5-OBJ_RADIUS])  # right low point
    ws_side.append([ws_cen[0]+ws_w * GRID_SIZE*0.5+OBJ_RADIUS, ws_cen[1]+ws_d * GRID_SIZE*0.5+OBJ_RADIUS])  # right high point
    ws_side.append([ws_cen[0]-ws_w * GRID_SIZE*0.5, ws_cen[1]+ws_d * GRID_SIZE*0.5+OBJ_RADIUS])  # left high point

    obs_wall = []
    print("number of width side wall", ws_w*GRID_SIZE/OBJ_RADIUS)
    print("number of depth side wall", ws_d*GRID_SIZE/OBJ_RADIUS)
    obs_wall.extend(np.linspace(ws_side[0], ws_side[1], round(ws_w*GRID_SIZE/OBJ_RADIUS)))
    print("obstacles for wall", len(obs_wall), type(obs_wall), obs_wall)
    obs_wall.extend(np.linspace(ws_side[1], ws_side[2], round(ws_d*GRID_SIZE/OBJ_RADIUS)))
    print("obstacles for wall", len(obs_wall), type(obs_wall), obs_wall)
    obs_wall.extend(np.linspace(ws_side[2], ws_side[3], round(ws_w*GRID_SIZE/OBJ_RADIUS)))
    print("obstacles for wall", len(obs_wall), type(obs_wall), obs_wall)

    print("obstacles for wall", len(obs_wall), type(obs_wall), obs_wall)
    plt.figure()
    for i in range(len(ws_point)):
        plt.scatter(ws_side[i][0], ws_side[i][1], color='red', alpha=0.5)
        plt.scatter(ws_point[i][0], ws_point[i][1], color='black', alpha=0.8)

    for i in range(len(obs_wall)):
        plt.scatter(obs_wall[i][0], obs_wall[i][1], color='brown')
    plt.axis('equal')
    plt.show()