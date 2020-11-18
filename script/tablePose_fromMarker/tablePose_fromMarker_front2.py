import numpy as np
import math
import tf.transformations as ttf
import matplotlib.pyplot as plt

# assume marker 1: left marker, 2: right marker
w = 0.2
d = 0.15
theta = 10
marker_1 = [0.1, 0.1, 0.0, 0.0]
marker_2 = [marker_1[0]+w*math.cos(math.radians(theta)), marker_1[1]+w*math.sin(math.radians(theta)), 0, 0]
marker_3 = [marker_1[0]+w*math.cos(math.radians()), marker_1[1]+w*math.sin(math.radians(theta)), 0, 0]
marker_c = [0.15, 0.15, 0.0, 0.0]

# l = math.sqrt(((marker_1[0]-marker_2[0])**2)+((marker_1[1]-marker_2[1])**2))
cross_line = math.sqrt((w**2 + d**2))
# theta = math.asin((marker_2[1]-marker_1[1])/w)

theta = math.radians(10)
print theta, math.degrees(theta)

z_axis = (0.0, 0.0, 1)
Rz = ttf.rotation_matrix(theta, z_axis)

r1 = np.matmul(Rz, marker_1)
r2 = np.matmul(Rz, marker_2)
r3 = np.add(r1, r2)/2
rc = np.matmul(Rz, marker_c)

d_x = r1[0] - marker_1[0]
d_y = r1[1] - marker_1[1]

t1 = [r1[0] - d_x, r1[1] - d_y]
# t2 = [r2[0] - d_x, r2[1] - d_y]
# t3 = [r3[0] - d_x, r3[1] - d_y]
# tc = [rc[0] - d_x, rc[1] - d_y]

plt.figure(figsize=(20, 20))
plt.scatter(marker_1[0], marker_1[1], c='r', s=1000, alpha=0.5, edgecolors='none')
# plt.scatter(marker_2[0], marker_2[1], c='r', s=1000, alpha=0.5, edgecolors='none')
# plt.scatter(marker_3[0], marker_3[1], c='r', s=1000, alpha=0.5, edgecolors='none')
# plt.scatter(marker_c[0], marker_c[1], c='r', s=1000, alpha=0.5, edgecolors='none')

plt.scatter(r1[0], r1[1], c='green', s=1000, alpha=0.5, edgecolors='none')
# plt.scatter(r2[0], r2[1], c='green', s=1000, alpha=0.5, edgecolors='none')
# plt.scatter(r3[0], r3[1], c='green', s=1000, alpha=0.5, edgecolors='none')
# plt.scatter(rc[0], rc[1], c='green', s=1000, alpha=0.5, edgecolors='none')

plt.scatter(t1[0], t1[1], c='green', s=500, alpha=0.8, edgecolors='none')
# plt.scatter(t2[0], t2[1], c='green', s=500, alpha=0.8, edgecolors='none')
# plt.scatter(t3[0], t3[1], c='green', s=500, alpha=0.8, edgecolors='none')
# plt.scatter(tc[0], tc[1], c='green', s=500, alpha=0.8, edgecolors='none')
plt.scatter(0, 0, c='black', s=500)
# plt.axis('equal')
plt.xlim(-0.5, 0.5)
plt.ylim(-0.5, 0.5)
plt.show()