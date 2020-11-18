import numpy as np
import math
import tf.transformations as ttf
import matplotlib.pyplot as plt
import D_0902_client_function as CLF

def make_env_vis_shelf(self, dx, dy):
    self.env_name = ['shelf0', 'shelf1', 'shelf2', 'side0', 'side1', 'side2']
    env_info = []
    env_info.append([[0 + dx, 0 + dy, 0.90 / 2.0], [0, 0, 0, 0], [0.45, 0.95, 0.90]])  # information for shelf0
    env_info.append([[0 + dx, 0 + dy, 1.30 - 0.018 / 2.0], [0, 0, 0, 0], [0.45, 0.95, 0.018]])  # information for shelf1
    env_info.append([[0 + dx, 0 + dy, 1.50 - 0.018 / 2.0], [0, 0, 0, 0], [0.45, 0.95, 0.018]])  # information for shelf2

    env_info.append([[0 + dx, 0 + dy - 0.95 / 2.0 + 0.018 / 2.0, 1.50 / 2.0], [0, 0, 0, 0], [0.45, 0.018, 1.5]])  # information for side0
    env_info.append([[0 + dx + 0.45 / 2.0 - 0.018 / 2.0, 0 + dy, 1.50 / 2.0], [0, 0, 0, 0], [0.018, 0.95, 1.5]])  # information for side1
    env_info.append([[0 + dx, 0 + dy + 0.95 / 2.0 - 0.018 / 2.0, 1.50 / 2.0], [0, 0, 0, 0], [0.45, 0.018, 1.5]])  # information for side2
    for i in range(len(env_info)):
        CLF.add_box_client(self.env_name[i], env_info[i][0], env_info[i][1], env_info[i][2], 'gray')
    return 1


def del_shelf(self):
    for i in self.env_name:
        CLF.del_box_client(i)


def make_env_counter(self, dx, dy):
    env_name = ['counter']
    env_info = []
    env_info.append([[0 + dx, 0 + dy, 0.90 / 2.0], [0, 0, 0, 0], [0.41 + 0.17, 0.8 + 0.17, 0.90]])  # information for counter
    for i in range(len(env_info)):
        CLF.add_box_client(env_name[i], env_info[i][0], env_info[i][1], env_info[i][2], 'gray')
    return 1

# assume marker 1: left marker, 2: right marker
base = 'counter'
base = 'shelf'
if base == 'counter':
    w = 0.295
    d = 0.335 / 2.0
elif base == 'shelf':
    w = 0.355
    d = 0.335 / 2.0
theta = math.radians(20.0)
# theta = math.radians(-80)
# phi = math.atan2(d, w)  # front markers
phi = math.atan2(-w, d)  # back markers
print "phi:", math.degrees(phi)
diagonal = math.sqrt((w**2 + d**2))
print "dig:", diagonal

marker_1 = [0.2, 0.3, 0.0, 0.0]
marker_2 = [marker_1[0]+2*d*math.cos(theta), marker_1[1]+2*d*math.sin(theta), 0, 0]
print "marker_1 (x,y):", marker_1
print "marker_2 (x,y):", marker_2
# marker_c = [marker_1[0]+(diagonal/2.0)*math.cos(theta+phi), marker_1[1]+(diagonal/2.0)*math.sin(theta+phi), 0, 0]

l = math.sqrt(((marker_1[0]-marker_2[0])**2)+((marker_1[1]-marker_2[1])**2))
print "depth:", l
theta_get = math.asin((marker_2[1]-marker_1[1])/(2*d))

# theta = math.radians(10)
print "get_theta", math.degrees(theta_get)

z_axis = (0.0, 0.0, 1)
Rz = ttf.rotation_matrix(theta, z_axis)

print "total angle:", math.degrees(theta+phi), "cos, sin:", math.cos(theta+phi), math.sin(theta+phi)
print "add x:", (diagonal)*math.cos(theta+phi), "add y:", (diagonal)*math.sin(theta+phi)
marker_c = [marker_1[0]+(diagonal)*math.cos(theta+phi), marker_1[1]+(diagonal)*math.sin(theta+phi), 0, 0]

print "center (x,y):", marker_c
plt.figure(figsize=(20, 20))
plt.scatter(marker_1[0], marker_1[1], c='r', s=1000, alpha=0.5, edgecolors='none')
plt.scatter(marker_2[0], marker_2[1], c='r', s=1000, alpha=0.5, edgecolors='none')
plt.scatter(marker_c[0], marker_c[1], c='r', s=1000, alpha=0.5, edgecolors='none')

plt.scatter(0, 0, c='black', s=500)
# plt.axis('equal')
plt.xlim(-1, 1)
plt.ylim(-1, 1)
plt.show()