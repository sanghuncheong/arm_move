import numpy as np
import matplotlib.pyplot as plt
import math
from copy import deepcopy

def rel2world(robot_world_xyz, robot_world_theta, object_rel_xyz, object_rel_theta):
    print "make relative coordinate to world coordinate"
    coor_r = 0.1

    # robot center position and xy axes in world coordinate.
    rb_c = deepcopy(robot_world_xyz)
    rb_x = [rb_c[0] + coor_r * math.cos(math.radians(robot_world_theta)), rb_c[1] + coor_r * math.sin(math.radians(robot_world_theta))]
    rb_y = [rb_c[0] + coor_r * math.cos(math.radians(robot_world_theta+90)), rb_c[1] + coor_r * math.sin(math.radians(robot_world_theta+90))]

    print rb_c, object_rel_xyz
    ob_world_c = np.add(np.array(rb_c) + np.array(object_rel_xyz))
    print "object world center:", ob_world_c, type(ob_world_c)


    plt.figure(figsize=(20, 20))
    plt.scatter(robot_world_xyz[0], robot_world_xyz[1], c='black', s=300, alpha=0.5, edgecolors='none')
    plt.scatter(rb_x[0], rb_x[1], c='black', s=300, alpha=0.5, edgecolors='none')
    plt.scatter(rb_y[0], rb_y[1], c='black', s=300, alpha=0.5, edgecolors='none')

    # object rel point
    plt.scatter(object_rel_xyz[0], object_rel_xyz[1], c='red', s=300, alpha=0.5, edgecolors='none')

    # world zero point
    plt.scatter(0, 0, c='black', s=300, alpha=0.8, edgecolors='none')

    # plt.axis('equal')
    plt.xlim(-1, 1)
    plt.ylim(-1, 1)
    plt.show()

if __name__ == '__main__':

    rb_x = 0.1      # robot position.z in world coordinate
    rb_y = 0.1      # robot position.y in world coordinate
    rb_z = 0.0      # robot position.z in world coordinate
    rb_world_theta = 45.0 # robot rotation in Z axis on world coordinate
    rb_world_c = [rb_x, rb_y, rb_z]
    rb_world_x = [rb_x+0.1, rb_y, rb_z]
    rb_world_y = [rb_x, rb_y+0.1, rb_z]

    ob_x = 0.1
    ob_y = 0.1
    ob_z = 0.0
    ob_world_theta = 45.0
    ob_rel_c = [ob_x, ob_y, ob_z]

    rel2world(rb_world_c, rb_world_theta, ob_rel_c, ob_world_theta)
