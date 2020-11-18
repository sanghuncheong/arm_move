import numpy as np
import copy

tar_pos = [1.1, 1.1]
can_pos = [[1.0, 1.0], [0.9, 0.9], [0.8, 0.8]]

can_dist = []
for i in can_pos:
    can_dist.append(np.sqrt((tar_pos[0] - i[0]) ** 2 + (tar_pos[1] - i[1]) ** 2))

print "candidate dist from target", can_dist
sort_can_dist = copy.deepcopy(can_dist)
sort_can_dist.sort(reverse=True)
print "sort can dist:", can_dist
sort_can_pos = []
for i in range(len(can_dist)):
    sort_can_pos.append(can_pos[can_dist.index(sort_can_dist[i])])

print "sorted candidates:", sort_can_pos