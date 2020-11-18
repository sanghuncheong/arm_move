import tree_making_area1 as TM
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

d_max = 2.0
ob = len(obs_pos)
for i in range(ob):
    obs_ori.append([0.0, 0.0, 0.0])

ret1 = TM.tree_making(ob, tar_pos, tar_ori, obs_pos, obs_ori, rob_pos, rob_pos, d_max)
print "answer", ret1

#def tree_making(ob, target_position, target_orientation, obstacle_position, obstacle_orientation, Body_position, Jaco_tip_position, d_max):
