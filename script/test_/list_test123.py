import numpy as np
import copy
a = [[10, 1, 0, 0, 2, 3]]

b = copy.deepcopy(a[0])
print b.index(min(b))
print b.index(min(b))
print [i for i in range(len(b)) if b[i]==(min(b))]

print np.sqrt((1.0-4.0)**2+(1.0-2.0)**2)

tar_grid = [28, 32]
GRID_SIZE = 0.01
ws_cen = [0.75, 0.00]
ws_w, ws_d = 40, 80
ws_zero = [round(ws_cen[0] - ws_w * GRID_SIZE * 0.5, 2), round(ws_cen[1] - ws_d * GRID_SIZE * 0.5, 2)]

print "tar_pos", [round(tar_grid[0] * GRID_SIZE + ws_zero[0], 2), round(tar_grid[1] * GRID_SIZE + ws_zero[1], 2)]  # target object!
