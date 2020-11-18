import numpy as np
#
# a = [[0,1], [1,1], [2,2], [0,1]]
# a.pop(1)
# print a
#
# obs_ori = np.zeros([8,3])
# print obs_ori
# print obs_ori[0], type(obs_ori)
#
# print list(obs_ori)
#
# print [0,1] in a
# print [0,2] in a

a = ['T', 2, 3, 4, 5, 6]
a.extend([2])
a.append(2)
a.extend([2, 3])
a.append([2, 3])
print a
print len(a)