import numpy as np
import math
import tf.transformations as ttf
import matplotlib.pyplot as plt

# assume marker 1: left marker, 2: right marker
w = 0.2
d = 0.1
theta = math.radians(10)
theta = math.radians(-80)
# phi = math.atan2(d, w)  # front markers
phi = math.atan2(-d, w)  # back markers
print math.degrees(phi)
diagonal = math.sqrt((w**2 + d**2))
print diagonal

marker_1 = [0.2, 0.1, 0.0, 0.0]
marker_2 = [marker_1[0]+w*math.cos(theta), marker_1[1]+w*math.sin(theta), 0, 0]
# marker_c = [marker_1[0]+(diagonal/2.0)*math.cos(theta+phi), marker_1[1]+(diagonal/2.0)*math.sin(theta+phi), 0, 0]

l = math.sqrt(((marker_1[0]-marker_2[0])**2)+((marker_1[1]-marker_2[1])**2))
theta_get = math.asin((marker_2[1]-marker_1[1])/w)

# theta = math.radians(10)
print "get_theta", math.degrees(theta_get)

z_axis = (0.0, 0.0, 1)
Rz = ttf.rotation_matrix(theta, z_axis)

marker_c = [marker_1[0]+(diagonal/2.0)*math.cos(theta+phi), marker_1[1]+(diagonal/2.0)*math.sin(theta+phi), 0, 0]

plt.figure(figsize=(20, 20))
plt.scatter(marker_1[0], marker_1[1], c='r', s=1000, alpha=0.5, edgecolors='none')
plt.scatter(marker_2[0], marker_2[1], c='r', s=1000, alpha=0.5, edgecolors='none')
plt.scatter(marker_c[0], marker_c[1], c='r', s=1000, alpha=0.5, edgecolors='none')

plt.scatter(0, 0, c='black', s=500)
# plt.axis('equal')
plt.xlim(-0.5, 0.5)
plt.ylim(-0.5, 0.5)
plt.show()