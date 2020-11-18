import S0116_custom_function as CUF
from VFHplus_change_radius import influence
# from VFHplus_mobile import influence
# from tree_making_no_plot import tree_making as TM_noplot
# from tree_making_no_plot2 import tree_making as TM_noplot
# from tree_making_plot import tree_making as TM_plot
import D_0902_client_function as CLF

import numpy as np
import copy
import sensor_msgs
import std_msgs
import moveit_msgs
import math
import time
from tf.transformations import quaternion_from_euler
import matplotlib.pyplot as plt


GRID_SIZE = 0.01

def get_obs_wall(ws_cen, ws_size, OBJ_R):
    ws_w, ws_d = ws_size
    ws_side = []
    ws_side.append(
        [ws_cen[0] - ws_w * GRID_SIZE * 0.5, ws_cen[1] - ws_d * GRID_SIZE * 0.5 - OBJ_R])  # left low point
    ws_side.append([ws_cen[0] + ws_w * GRID_SIZE * 0.5 + OBJ_R,
                    ws_cen[1] - ws_d * GRID_SIZE * 0.5 - OBJ_R])  # right low point
    ws_side.append([ws_cen[0] + ws_w * GRID_SIZE * 0.5 + OBJ_R,
                    ws_cen[1] + ws_d * GRID_SIZE * 0.5 + OBJ_R])  # right high point
    ws_side.append(
        [ws_cen[0] - ws_w * GRID_SIZE * 0.5, ws_cen[1] + ws_d * GRID_SIZE * 0.5 + OBJ_R])  # left high point

    obs_wall = []
    obs_wall.extend(CUF.linspace2D(ws_side[0], ws_side[1], round(ws_w * GRID_SIZE / OBJ_R)))
    obs_wall.extend(CUF.linspace2D(ws_side[1], ws_side[2], round(ws_d * GRID_SIZE / OBJ_R)))
    obs_wall.extend(CUF.linspace2D(ws_side[2], ws_side[3], round(ws_w * GRID_SIZE / OBJ_R)))
    return obs_wall

def get_can_info(self, in_can_info, in_obs_pos, in_obs_re_pos, in_ore_order, in_tar_pos):
    tmp_can_info = []
    for i in range(len(in_ore_order)):
        tmp_can_info.append(copy.deepcopy(in_can_info))
    tmp_obs_pos = copy.deepcopy(in_obs_pos)
    tmp_obs_re_pos = copy.deepcopy(in_obs_re_pos)
    tmp_ore_order = copy.deepcopy(in_ore_order)
    tmp_tar_pos = copy.deepcopy(in_tar_pos)
    # print("\nCheck if candidate blocks the target")
    for step_i in range(len(tmp_ore_order)):
        for i in range(len(tmp_can_info[step_i])):
            vfh_obs_pos = copy.deepcopy(tmp_obs_re_pos)
            vfh_obs_pos.append(tmp_can_info[step_i][i].pos)
            vfh_obs_pos.extend(self.obs_wall)
            vfh_tar_pos = copy.deepcopy(tmp_tar_pos)
            ob = len(vfh_obs_pos)
            vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, self.rob_pos, self.d_max, self.eta)
            # vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, self.rob_pos, self.obs_r, self.tar_r)
            if vfh[3] == 0:
                tmp_can_info[step_i][i].BT = 1  # BT == 1 : The candidate blocks the target.
            else:                       # BT == 0 : The candidate does not block the target.
                tmp_can_info[step_i][i].BT = 0

        # print("\nCheck if the candidate is accessible.")
        for i in range(len(tmp_can_info[step_i])):
            vfh_tar_pos = copy.deepcopy(tmp_can_info[step_i][i].pos)
            vfh_obs_pos = copy.deepcopy(tmp_obs_pos)
            for si in range(step_i+1):
                # print "\nstep", si, "\nbefore", vfh_obs_pos
                vfh_obs_pos.remove(tmp_obs_pos[tmp_ore_order[si]])
                # print "after", vfh_obs_pos
            vfh_obs_pos.append(tmp_tar_pos)
            vfh_obs_pos.extend(self.obs_wall)
            ob = len(vfh_obs_pos)
            vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, self.rob_pos, self.d_max, self.eta)
            if vfh[3] == 0:                      # A == 1 : The candidate is accessible.
                tmp_can_info[step_i][i].A = 0    # A == 0 : The candidate is not accessible.
            else:                   #
                tmp_can_info[step_i][i].A = 1

        # print("\nCheck the candidate ORC.")
        for i in range(len(tmp_can_info[step_i])):
            vfh_tar_pos = copy.deepcopy(tmp_can_info[step_i][i].pos)
            vfh_obs_pos = copy.deepcopy(tmp_obs_pos)
            vfh_obs_pos.append(tmp_tar_pos)
            vfh_obs_pos.extend(self.obs_wall)
            ob = len(vfh_obs_pos)
            vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, self.rob_pos, self.d_max, self.eta)
            if vfh[3] == 0:              # A == 1 : The candidate is accessible.
                tmp_can_info[step_i][i].A = 0    # A == 0 : The candidate is not accessible.
                tm_tar_pos = copy.deepcopy(vfh_tar_pos)
                tm_tar_ori = [0.0, 0.0, 0.0]
                tm_obs_pos = copy.deepcopy(tmp_obs_pos)
                # tm_obs_pos.extend(self.obs_wall)
                ob = len(tm_obs_pos)
                tm_obs_ori = []
                for obs_ori_i in range(ob):
                    tm_obs_ori.append([0.0, 0.0, 0.0])

                ore_order = NG_ore(tm_tar_pos, tm_obs_pos, self.tar_r, self.obs_r, self.rob_pos, self.ws_zero, [self.ws_w * self.GRID_SIZE, self.ws_d * self.GRID_SIZE])
                # ore_order = TM_noplot(ob, tm_tar_pos, tm_tar_ori, tm_obs_pos, tm_obs_ori, self.rob_pos, self.rob_pos, self.d_max)
                ore_order.pop()  # The last order is always the target so we need to pop the last element.
                tmp_can_info[step_i][i].ORC = ore_order
            else:                   #
                tmp_can_info[step_i][i].A = 1

    return tmp_can_info

def get_can_A(self, in_can_info, in_obs_pos, in_tar_pos):
    tmp_can_info = copy.deepcopy(in_can_info)
    # print("\nCheck if the candidate is accessible.")
    for ci in range(len(tmp_can_info)):
        vfh_tar_pos = copy.deepcopy(tmp_can_info[ci].pos)
        vfh_obs_pos = copy.deepcopy(in_obs_pos)
        vfh_obs_pos.append(copy.deepcopy(in_tar_pos))
        vfh_obs_pos.extend(self.obs_wall)
        ob = len(vfh_obs_pos)
        vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, self.rob_pos, self.d_max, self.eta)
        if vfh[3] == 0:  # A == 1 : The candidate is accessible.
            print ci, "A = 0 (vfh)"
            tmp_can_info[ci].A = 0  # A == 0 : The candidate is not accessible.
        else:  #
            print ci, "A = 1 (vfh)"

            print "\nangle:", vfh[-1]
            xi, yi = tmp_can_info[ci].pos[0], tmp_can_info[ci].pos[1]
            CLF.add_box_client('can_check', [self.object_z, -yi, xi], [-0.707, 0.0, -0.707, 0.0], [0.06, 0.06, 0.2], 'pink')

            planner_name = 'RRTConnect'
            # planner_name = 'BiTRRT'
            n_attempt = 10
            c_time = 0.5
            n_repeat = 5
            start_state = moveit_msgs.msg.RobotState()
            joint_state = sensor_msgs.msg.JointState()
            joint_state.header = std_msgs.msg.Header()
            joint_state.name = ['j2n6s300_joint_1', 'j2n6s300_joint_2', 'j2n6s300_joint_3', 'j2n6s300_joint_4', 'j2n6s300_joint_5', 'j2n6s300_joint_6']
            joint_state.position = [3.1415927410125732, 4.537856101989746, 5.93411922454834, -0.6108652353286743, 1.7453292608261108, -0.5235987901687622]
            start_state.joint_state = joint_state
            # goal_pose:
            # goal_orientation:
            # goal_pose = [self.object_z, -yi, xi - 0.05]
            # goal_orientation = [-0.00145713772037, -0.998970756926, 0.0364956710831, 0.0268955302573]
            z = self.object_z
            goal_pose = [z + 0.7, -yi, xi]
            # Set the grasp pose: substract 17cm from the z value of the object centroid
            goal_pitches = []
            goal_pitch = np.deg2rad(vfh[-1])
            # goal_pitch = vfh[-1] + math.pi/2
            goal_pitches.append(goal_pitch)  # approaching_angle: vfh[-1] from the input
            for i in range(1):
                goal_pitches.append(goal_pitch + (i + 1) * (math.pi / 36))
                goal_pitches.append(goal_pitch - (i + 1) * (math.pi / 36))
            # Get the grasp orientation (currently the front direction)
            goal_orientations = []
            for i in goal_pitches:
                goal_orientations.append(quaternion_from_euler(-i, math.radians(-5.0), math.radians(90.0), axes='rxyz'))
            l = 0.001
            goal_poses = []
            for i in goal_pitches:
                dx = math.sin(i - math.pi) * l
                dy = math.cos(i - math.pi) * l
                goal_poses.append([z + 0.1, -yi + dx, xi + dy])
                # goal_poses.append([z + 0.07, goal_pose[1] + dx, goal_pose[2] - dy])

            # plt.figure()
            # for i in range(len(in_obs_pos)):
            #     plt.scatter(in_obs_pos[i][0], in_obs_pos[i][1], s=200, c='red')
            # plt.scatter(xi, yi, s=200, c='pink')
            # plt.scatter(xi + dy, yi + dx, s=200, c='blue')
            #
            # plt.figure()
            # for i in range(len(in_obs_pos)):
            #     plt.scatter(in_obs_pos[i][0], in_obs_pos[i][1], s=200, c='red')
            # plt.scatter(xi, yi, s=200, c='pink')
            # plt.scatter(xi + dy, yi - dx, s=200, c='blue')
            #
            # plt.show()

            feasibility1 = 0
            i = 0
            while not feasibility1 and i < len(goal_pitches):
                CLF.add_box_client('can_bottom_x', [goal_poses[i][0]-0.15, goal_poses[i][1], goal_poses[i][2]], goal_orientations[i], [0.1, 0.005, 0.005], 'red')
                CLF.add_box_client('can_bottom_y', [goal_poses[i][0]-0.15, goal_poses[i][1], goal_poses[i][2]], goal_orientations[i], [0.005, 0.1, 0.005], 'blue')
                CLF.add_box_client('can_bottom_z', [goal_poses[i][0]-0.15, goal_poses[i][1], goal_poses[i][2]], goal_orientations[i], [0.005, 0.005, 0.1], 'green')
                [feasibility1, trajectory1] = CLF.feasible_check_obj_joint_client('arm', 'gripper', start_state, goal_poses[i], goal_orientations[i], [], planner_name, n_attempt, c_time, n_repeat)
                i = i + 1
                time.sleep(1)
                CLF.del_box_client('can_bottom_x')
                CLF.del_box_client('can_bottom_y')
                CLF.del_box_client('can_bottom_z')
            if feasibility1 == 0:  # A == 1 : The candidate is accessible.
                print ci, "A = 0 (MP)"
                tmp_can_info[ci].A = 0  # A == 0 : The candidate is not accessible.
            else:  #
                print ci, "A = 1 (MP)"
                tmp_can_info[ci].A = 1
            CLF.del_box_client('can_check')
            tmp_can_info[ci].A = 1
    return tmp_can_info

def init_BT(self, in_can_info):
    for ci in in_can_info:
        ci.BT = 0
    return in_can_info

def get_can_BT(self, in_can_info, in_obs_pos, in_tar_pos):
    tmp_can_info = copy.deepcopy(in_can_info)
    # print("\nCheck if candidate blocks the target")
    for ci in range(len(tmp_can_info)):
        if tmp_can_info[ci].A == 1:
            if tmp_can_info[ci].BT == 0:
                vfh_obs_pos = copy.deepcopy(in_obs_pos)
                vfh_obs_pos.append(tmp_can_info[ci].pos)
                vfh_obs_pos.extend(self.obs_wall)
                vfh_tar_pos = copy.deepcopy(in_tar_pos)
                ob = len(vfh_obs_pos)
                vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, self.rob_pos, self.d_max, self.eta)
                if vfh[3] == 0:
                    print ci, "BT = 1 (vfh)"
                    tmp_can_info[ci].BT = 1  # BT == 1 : The candidate blocks the target.
                else:                        # BT == 0 : The candidate does not block the target.
                    print ci, "BT = 0 (vfh)"

                    print "\nangle:", vfh[-1]
                    xi, yi = tmp_can_info[ci].pos[0], tmp_can_info[ci].pos[1]
                    CLF.add_box_client('can_check', [self.object_z, -yi, xi], [-0.707, 0.0, -0.707, 0.0], [0.06, 0.06, 0.2], 'pink')

                    planner_name = 'RRTConnect'
                    # planner_name = 'BiTRRT'
                    n_attempt = 10
                    c_time = 0.5
                    n_repeat = 5
                    start_state = moveit_msgs.msg.RobotState()
                    joint_state = sensor_msgs.msg.JointState()
                    joint_state.header = std_msgs.msg.Header()
                    joint_state.name = ['j2n6s300_joint_1', 'j2n6s300_joint_2', 'j2n6s300_joint_3', 'j2n6s300_joint_4', 'j2n6s300_joint_5', 'j2n6s300_joint_6']
                    joint_state.position = [3.1415927410125732, 4.537856101989746, 5.93411922454834, -0.6108652353286743, 1.7453292608261108, -0.5235987901687622]
                    start_state.joint_state = joint_state
                    # goal_pose:
                    # goal_orientation:
                    # goal_pose = [self.object_z, -yi, xi - 0.05]
                    # goal_orientation = [-0.00145713772037, -0.998970756926, 0.0364956710831, 0.0268955302573]
                    z = self.object_z
                    goal_pose = [z + 0.12, -yi, xi]
                    # Set the grasp pose: substract 17cm from the z value of the object centroid
                    goal_pitches = []
                    goal_pitch = np.deg2rad(vfh[-1])
                    # goal_pitch = vfh[-1] + math.pi/2
                    goal_pitches.append(goal_pitch)  # approaching_angle: vfh[-1] from the input
                    for i in range(1):
                        goal_pitches.append(goal_pitch + (i + 1) * (math.pi / 36))
                        goal_pitches.append(goal_pitch - (i + 1) * (math.pi / 36))
                    # Get the grasp orientation (currently the front direction)
                    goal_orientations = []
                    for i in goal_pitches:
                        goal_orientations.append(quaternion_from_euler(-i, math.radians(-5.0), math.radians(90.0), axes='rxyz'))
                    l = 0.001
                    goal_poses = []
                    for i in goal_pitches:
                        dx = math.sin(i - math.pi) * l
                        dy = math.cos(i - math.pi) * l
                        goal_poses.append([z + 0.1, -in_tar_pos[1] + dx, in_tar_pos[0] + dy])
                        # goal_poses.append([z + 0.07, goal_pose[1] + dx, goal_pose[2] - dy])

                    # plt.figure()
                    # for i in range(len(in_obs_pos)):
                    #     plt.scatter(in_obs_pos[i][0], in_obs_pos[i][1], s=200, c='red')
                    # plt.scatter(xi, yi, s=200, c='pink')
                    # plt.scatter(xi + dy, yi + dx, s=200, c='blue')
                    #
                    # plt.figure()
                    # for i in range(len(in_obs_pos)):
                    #     plt.scatter(in_obs_pos[i][0], in_obs_pos[i][1], s=200, c='red')
                    # plt.scatter(xi, yi, s=200, c='pink')
                    # plt.scatter(xi + dy, yi - dx, s=200, c='blue')
                    #
                    # plt.show()

                    feasibility1 = 0
                    i = 0
                    while not feasibility1 and i < len(goal_pitches):
                        CLF.add_box_client('can_bottom_x', [goal_poses[i][0]-0.15, goal_poses[i][1], goal_poses[i][2]], goal_orientations[i], [0.1, 0.005, 0.005], 'red')
                        CLF.add_box_client('can_bottom_y', [goal_poses[i][0]-0.15, goal_poses[i][1], goal_poses[i][2]], goal_orientations[i], [0.005, 0.1, 0.005], 'blue')
                        CLF.add_box_client('can_bottom_z', [goal_poses[i][0]-0.15, goal_poses[i][1], goal_poses[i][2]], goal_orientations[i], [0.005, 0.005, 0.1], 'green')
                        [feasibility1, trajectory1] = CLF.feasible_check_obj_joint_client('arm', 'gripper', start_state, goal_poses[i], goal_orientations[i], [], planner_name, n_attempt, c_time, n_repeat)
                        i = i + 1
                        time.sleep(1)
                        CLF.del_box_client('can_bottom_x')
                        CLF.del_box_client('can_bottom_y')
                        CLF.del_box_client('can_bottom_z')
                    if feasibility1 == 0:  # A == 1 : The candidate is accessible.
                        print ci, "BT = 1 (MP)"
                        tmp_can_info[ci].BT = 1  # A == 0 : The candidate is not accessible.
                    else:  #
                        print ci, "BT = 0 (MP)"
                        tmp_can_info[ci].BT = 0
                    CLF.del_box_client('can_check')
        else:
            print ci, "A = 0 => BT = no matter"

    return tmp_can_info

def get_cf(self, in_can_info):
    tmp_cf = []
    tmp_cf_index = []
    tmp_can_info = copy.deepcopy(in_can_info)

    # print("\nCheck the candidate ORC.")
    for ci in range(len(tmp_can_info)):
        # print "\ncan ", ci, "th has A, BT :", tmp_can_info[ci].A, tmp_can_info[ci].BT
        if tmp_can_info[ci].A == 1 and tmp_can_info[ci].BT == 0:
            tmp_cf.append(tmp_can_info[ci])
            tmp_cf_index.append(ci)
    return tmp_cf, tmp_cf_index

def get_cf_b(self, in_cf, in_obs_pos):
    tmp_cf = copy.deepcopy(in_cf)
    tmp_obs_pos = copy.deepcopy(in_obs_pos)
    tmp_b = []
    for cb in range(len(tmp_cf)):  # cb: The candidate that will check the b value
        b = 0
        for ci in range(len(tmp_cf)):  # ci: Other candidates for checking the b value
            if cb != ci:
                # print "\ntar", tmp_cf_pos[ci]
                # print "obs", tmp_obs_pos
                vfh_tar_pos = copy.deepcopy(tmp_cf[ci].pos)
                vfh_obs_pos = copy.deepcopy(tmp_obs_pos)
                vfh_obs_pos.append(tmp_cf[cb].pos)
                vfh_obs_pos.extend(self.obs_wall)
                ob = len(vfh_obs_pos)
                vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, self.rob_pos, self.d_max, self.eta)
                if vfh[3] == 0:
                    b = b + 1
        tmp_b.append(b)
    return tmp_b

def get_cp(self, in_can_info):
    tmp_cp = []
    tmp_cp_index = []
    tmp_can_info = copy.deepcopy(in_can_info)

    # print("\nCheck the candidate ORC.")
    for ci in range(len(tmp_can_info)):
        # print "\ncan ", ci, "th has A, BT :", tmp_can_info[ci].A, tmp_can_info[ci].BT
        if tmp_can_info[ci].A == 0:
            tmp_cp.append(tmp_can_info[ci])
            tmp_cp_index.append(ci)
    return tmp_cp, tmp_cp_index

# def get_can_A(self, in_can_info, in_obs_pos, in_tar_pos):
#     tmp_can_info = copy.deepcopy(in_can_info)
#     # print("\nCheck if the candidate is accessible.")
#     for ci in range(len(tmp_can_info)):
#         vfh_tar_pos = copy.deepcopy(tmp_can_info[ci].pos)
#         vfh_obs_pos = copy.deepcopy(in_obs_pos)
#         vfh_obs_pos.append(copy.deepcopy(in_tar_pos))
#         vfh_obs_pos.extend(self.obs_wall)
#         ob = len(vfh_obs_pos)
#         vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, self.rob_pos, self.d_max, self.eta)
#         if vfh[3] == 0:  # A == 1 : The candidate is accessible.
#             tmp_can_info[ci].A = 0  # A == 0 : The candidate is not accessible.
#             # print "c:", ci, ".A = 0"
#         else:  #
#             tmp_can_info[ci].A = 1
#             # print "c:", ci, ".A = 1"
#     return tmp_can_info
#
# def get_can_BT(self, in_can_info, in_obs_pos, in_tar_pos):
#     tmp_can_info = copy.deepcopy(in_can_info)
#     # print("\nCheck if candidate blocks the target")
#     for ci in range(len(tmp_can_info)):
#         vfh_obs_pos = copy.deepcopy(in_obs_pos)
#         vfh_obs_pos.append(tmp_can_info[ci].pos)
#         vfh_obs_pos.extend(self.obs_wall)
#         vfh_tar_pos = copy.deepcopy(in_tar_pos)
#         ob = len(vfh_obs_pos)
#         vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, self.rob_pos, self.d_max, self.eta)
#         if vfh[3] == 0:
#             tmp_can_info[ci].BT = 1  # BT == 1 : The candidate blocks the target.
#             # print "c:", ci, ".BT = 1"
#         # else:                        # BT == 0 : The candidate does not block the target.
#         #     tmp_can_info[ci].BT = 0
#         #     print "c:", ci, ".BT = 0"
#
#     return tmp_can_info

def get_c_ore(self, in_can_info):
    # print "input", in_can_info
    # print in_can_info[0].pos
    t_c_order = []
    for ci in range(len(in_can_info)):
        tm_tar_pos = in_can_info[ci].pos
        tm_tar_ori = [0.0, 0.0, 0.0]
        tm_obs_pos = copy.deepcopy(self.obs_pos)
        tm_obs_pos.append(self.tar_pos)
        # tm_obs_pos.extend(self.obs_wall)
        tm_ob = len(tm_obs_pos)
        tm_obs_ori = []
        for i in range(tm_ob):
            tm_obs_ori.append([0.0, 0.0, 0.0])

        ore_order = NG_ore(tm_tar_pos, tm_obs_pos, self.tar_r, self.obs_r, self.rob_pos, self.ws_zero, [self.ws_w * self.GRID_SIZE, self.ws_d * self.GRID_SIZE])
        while 1:
            tm_tar_pos = in_can_info[ci].pos
            tm_obs_pos = copy.deepcopy(self.obs_pos)
            tm_obs_pos.append(self.tar_pos)
            obs_r = []
            for i in tm_obs_pos:
                obs_r.append(0.035)
            for i in ore_order:
                if i != 'T':
                    tm_obs_pos[i] = [4.0, 0.0]

            tmp_order = NG_ore(tm_tar_pos, tm_obs_pos, self.tar_r, obs_r, self.rob_pos, self.ws_zero, [self.ws_w * self.GRID_SIZE, self.ws_d * self.GRID_SIZE])
            # tmp_order = TM_noplot(ob, tm_tar_pos, tm_tar_ori, tm_obs_pos, tm_obs_ori, self.obs_wall,self.rob_pos, self.rob_pos, self.d_max)
            # print "after removing:", tmp_order
            if tmp_order[0] != 'T':
                # print "not ok"
                t_c_order.append([])
                break
                # if tmp_order[0] == -1:
                #     print "no path"
                #     t_c_order.append([])
                #     break
                # if len(ore_order) > len(self.obs_pos):
                #     print "no path"
                #     t_c_order.append([])
                #     break
                # print "tricky environment for c_ore so extend", ore_order
                # ore_order.pop()
                # print "delete target", ore_order
                # ore_order.extend(tmp_order)
                # print "to", ore_order
            else:
                if tmp_order[0] == 'T':
                    # print "ok", ore_order
                    ore_order.pop()

                    if len(tm_obs_pos) in ore_order:
                        # print "\n\nThere is target!! warning!!!\n\n"
                        t_c_order.append([])
                    else:
                        t_c_order.append(ore_order)
                    break
    return t_c_order


class CanInfo:
    def __init__(self, type, pos):
        self.type = type
        self.pos = pos
        self.A = 0
        self.BT = 1
        self.b = 0
        self.ORC = []  # to access, need to remove

    def show(self):
        print "\nCandidate Info"

if __name__=="__main__":
    c = []
    # for i in range(10):
    #     c.append(CandidateInfo())
    #
    # print "c position", c[0].pos
    # print c