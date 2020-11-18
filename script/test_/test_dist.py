import numpy as np

tar = [0.83, -0.08]
t_cf1 = [0.79, -0.35]
t_cf2 = [0.87, 0.05]
t_cf3 = [0.85, 0.29]
t_cf4 = [0.79, 0.34]


print np.sqrt((tar[0] - t_cf1[0])**2+(tar[1] - t_cf1[1])**2)
print np.sqrt((tar[0] - t_cf2[0])**2+(tar[1] - t_cf2[1])**2)
print np.sqrt((tar[0] - t_cf3[0])**2+(tar[1] - t_cf3[1])**2)
print np.sqrt((tar[0] - t_cf4[0])**2+(tar[1] - t_cf4[1])**2)

# t sel can dist [1.1815244390193544, 1.1710678887237922, 1.2133424908079335]

# tar grid:  [28, 32]
#
# tar grid:  [0.83, -0.08]
# t_cf grid x,y: 2 24 5
# t_cf grid x,y: 2 0.79 -0.35
# t_cf grid x,y: 5 32 45
# t_cf grid x,y: 5 0.87 0.05
# t_cf grid x,y: 13 30 69
# t_cf grid x,y: 13 0.85 0.29
# t_cf grid x,y: 19 24 74
# t_cf grid x,y: 19 0.79 0.34
# t sel can dist [0.8709190547921202, 0.9508417323613853, 0.9302150289046076, 0.8709190547921202]