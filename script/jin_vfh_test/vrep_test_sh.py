import tree_making_area1 as TM

rob_pos = [0.0, 0.0]

obs_pos = []
obs_ori = []
obs_pos.append([-0.2, 0.2])
obs_pos.append([-0.1, 0.2])
obs_pos.append([-0.01, 0.2])
obs_pos.append([0.01, 0.2])
obs_pos.append([0.1, 0.2])
obs_pos.append([0.2, 0.2])

tar_pos = [0.02, 0.3]
d_max = 1.0
ob = len(obs_pos)
for i in range(ob):
    obs_ori.append([0])
ret1 = TM.tree_making(ob, tar_pos, target_orientation=0, obstacle_position=obs_pos, obstacle_orientation=obs_ori, )

print ret1
#def tree_making(ob, target_position, target_orientation, obstacle_position, obstacle_orientation, Body_position, Jaco_tip_position, d_max):
