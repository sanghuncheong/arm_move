import numpy as np
from itertools import combinations
import matplotlib.pyplot as plt
import copy

obs = [[0, 0], [0, 1], [0, 2], [0, 3], [0, 4], [0, 5], [0, 6]]
ore_order = [2, 4]  # index

ore = []
for i in ore_order:
    ore.append(obs[i])
obs_re = copy.deepcopy(obs)
for i in ore:
    obs_re.remove(i)

print "given obstacles", obs
print "rearrange order", ore_order, "which is", ore
print "after rearrangemet", obs_re

step_gain = 2

ore_add_tried = []
print "\n IF WE NEED TO REARRANGE MORE with gain", step_gain
# for i in range(step_gain):
#     ore_add_tmp = np.random.