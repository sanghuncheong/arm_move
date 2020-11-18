import nam_vfh_test as ORE
from D_0730_envClass import EnvInfo as EI

import numpy as np

rob_pos = [0.0, 0.0]

obs_pos = []
obs_ori = []
obs_pos.append([-0.2, 0.2])
obs_pos.append([-0.1, 0.2])
obs_pos.append([-0.00, 0.2])
obs_pos.append([0.1, 0.2])
obs_pos.append([0.2, 0.2])
obs_pos.append([-0.2, 0.3])
obs_pos.append([-0.1, 0.3])
obs_pos.append([-0.00, 0.3])
obs_pos.append([0.1, 0.3])
obs_pos.append([0.2, 0.3])

tar_pos = [0.1, 0.35]
tar_ori = [0.0, 0.0, 0.0]
# obstacles [[0.27, 1.07], [0.11, 0.97], [0.16, 0.86], [-0.08, 0.97], [-0.33, 1.09], [-0.27, 0.95], [-0.23, 0.82], [-0.03, 0.85], [0.25, 0.91]]
# target [-0.08, 1.0]
# obs_pos2 =[[0.27, 1.07], [0.11, 0.97], [0.16, 0.86], [-0.08, 0.97], [-0.33, 1.09], [-0.27, 0.95], [-0.23, 0.82], [-0.03, 0.85], [0.25, 0.91]]
# tar_pos2 =[-0.08, 1.1]

ws_w, ws_d = 40, 80  # get table size in the v-rep
ws_cen = [0.75, 0.00]
rob_pos = [0.0, 0.0]
OBJ_R = 0.035
GRID_SIZE = 0.01
OBJ_R = 0.035
env = EI(rob_pos, ws_w, ws_d, ws_cen, grid_size=GRID_SIZE, wall_r=OBJ_R)
x_min = env.ws_zero[0]
x_max = env.ws_zero[0] + ws_w * GRID_SIZE
y_min = env.ws_zero[1] - ws_d * GRID_SIZE
y_max = env.ws_zero[1]

wall_minmax = [x_min, x_max, y_min, y_max]
wall_pos = env.obs_wall
wall_r = []
for i in range(len(wall_pos)):
    wall_r.append(0.035)

d_max = 2.0
obj_pos = [tar_pos]+obs_pos
ob = len(obj_pos)
obj_r = []
for i in range(ob):
    obs_ori.append([0.0, 0.0, 0.0])
    obj_r.append(0.035)

ret1 = ORE.find_ORE(ob, obj_pos, obj_r, wall_minmax, rob_pos)
print "answer", ret1

#def tree_making(ob, target_position, target_orientation, obstacle_position, obstacle_orientation, Body_position, Jaco_tip_position, d_max):
