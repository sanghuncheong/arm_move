import math
from tf.transformations import quaternion_from_euler, quaternion_multiply

goal_pitch =0
goal_pitches = []
goal_pitches.append(goal_pitch)

for i in range(3):
    goal_pitches.append(goal_pitch + (i + 1) * math.radians(8.0))
    goal_pitches.append(goal_pitch - (i + 1) * math.radians(8.0))
# Get the grasp orientation (currently the front direction)
goal_orientations = []
for i in goal_pitches:
    no_pitch_ori = quaternion_from_euler(math.radians(-90.0), math.radians(-90.0) + i, 0, axes='rxyz')
    print i, "rad", no_pitch_ori
    pitch_ori = quaternion_from_euler(math.radians(-20), 0.0, 0.0, axes='rxyz')
    print i, "rad", pitch_ori
    fin_ori = quaternion_multiply(no_pitch_ori, pitch_ori)
    print i, "rad", fin_ori, "\n"
    # goal_orientations.append()
