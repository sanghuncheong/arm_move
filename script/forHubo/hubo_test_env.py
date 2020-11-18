import client_function as CLF
from tf.transformations import quaternion_from_euler
import math

# make environment
box_name = 'object1'
box_xyz = [1.6+0.25, -0.3, 0.8 + 0.2/2 + 0.01]
box_xyzw = [0, 0, 0, 0]
box_wdh = [0.06, 0.06, 0.2]
box_color = 'red'
CLF.add_box_client(box_name, box_xyz, box_xyzw, box_wdh, box_color)

box_name = 'table'
box_xyz = [1.6+0.25, -0.3, 0.8/2]
box_xyzw = [0, 0, 0, 0]
box_wdh = [0.4, 0.4, 0.8]
box_color = 'red'
CLF.add_box_client(box_name, box_xyz, box_xyzw, box_wdh, box_color)

# plan for the arm
arm_name = 'R_arm'
goal_pos = [0.4, -0.4, 0.60]
goal_roll = math.radians(-90)
goal_pitches = []
goal_pitch = math.radians(-90)
# goal_pitches.append(goal_pitch)
# for i in range(2):
#     goal_pitches.append(goal_pitch + i*math.radians(2.5))
#     goal_pitches.append(goal_pitch - i*math.radians(2.5))

goal_yaws = []
goal_yaw = math.radians(0)
goal_yaws.append(goal_yaw)
for i in range(5):
    goal_yaws.append(goal_yaw + i*math.radians(2.5))
    goal_yaws.append(goal_yaw - i*math.radians(2.5))

goal_orientations = []
for i in goal_yaws:
    goal_orientations.append(quaternion_from_euler(goal_roll, goal_pitch, i, axes='rxyz'))

# goal_ori = quaternion_from_euler(math.radians(-90), math.radians(-90), 0, axes='rxyz')
# goal_ori = [0.0, -0.707, 0.0, 0.707]
planner_name = 'RRTConnect'
n_attempt = 100
c_time = 0.1
n_repeat = 10
hand_name = 'gripper'
start_state = [-1, -1, -1, -1, -1, -1]
obj = '-1'
for i in range(len(goal_orientations)):
    CLF.feasible_check_obj_joint_client(arm_name, hand_name, start_state, goal_pos, goal_orientations[i], obj, planner_name, n_attempt, c_time, n_repeat)

# CLF.feasible_check_obj_joint_client(arm_name, hand_name, start_state, goal_pos, goal_ori, obj, planner_name, n_attempt, c_time, n_repeat)
# CLF.move_goalpose_client(arm_name, hand_name, start_state, goal_pos, goal_ori, obj, planner_name, n_attempt, c_time, n_repeat)
