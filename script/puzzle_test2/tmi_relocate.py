#!/usr/bin/env python

#import sys
#import rospy
#import numpy as np
#import tf
#import matplotlib.pyplot as plt
#import copy
import time
import math
from numpy import deg2rad

import D_20_1020_S_client_function as CLF
import D_20_1020_gazebo_client_function as GCF

from arm_move.srv._arm_move_srv import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Header
from sensor_msgs.msg import JointState


PRE_GRASP_ANGLE_UNIT   = 8.0 # Degree
GRASP_ANGLE_UNIT       = 1.5 
POST_GRASP_ANGLE_UNIT  = 1.0
    
PRE_GRASP_MAX_STEP     =  55
GRASP_MAX_STEP         =  10
POST_GRASP_MAX_STEP    =   5
GO_HOME_MAX_STEP       =  55 

PRE_PLACE_MAX_STEP     =  60
PLACE_MAX_STEP         =   8

PRE_PLACE_ANGLE_UNIT   = 8.0 
PLACE_ANGLE_UNIT       = 1.0
POST_PLACE_ANGLE_UNIT  = 1.0


def table_AL_test():
    z_link_offset = 0.13228
        
    base_position = [0.8637, 0, 0.0]
    base_quaternion = [0, 0, 0, 1]
    base_scale = [0.001, 0.001, 0.001]

    CLF.add_mesh_client('shelf_gazebo', base_position, base_quaternion, base_scale)
    

def add_objects(all_objects_info, object_ids, R):
    object_name = [str(i).zfill(2) for i in object_ids]
    object_info = []
    
    for i in object_ids:
        object_info.append(all_objects_info[i])

    # obj_q = [-0.49911631, 0.49999083, -0.50088369, 0.5000076]
    obj_q = [0.0, 0.0, 0.0, 0.0]
    z_link_offset = 0.13228

    #z = 0.36772 + z_link_offset + 0.15/2.0 + 0.1
    z = 0.605

    for i in range(len(object_info)):
        #CLF.add_box_client(object_name[i], [object_info[i][1], -object_info[i][0], z], obj_q, [2 * R[i], 2 * R[i], 0.15], 'red')
        
        #print "object name : ", object_name[i]
        #print "x : ",  object_info[i][1] 
        #print "y : ", -object_info[i][0]

        CLF.add_mesh_client(object_name[i], [object_info[i][1], -object_info[i][0], z], obj_q, [0.001,0.001,0.001])
        GCF.change_gazebo_model_position(object_name[i],[object_info[i][1], -object_info[i][0], z])


def del_objects(object_ids):
    object_name = [str(i).zfill(2) for i in object_ids]
    for obj in object_name:
        CLF.del_box_client(obj)
    for i in range(len(object_ids)):
        GCF.change_gazebo_model_position(object_name[i],[i, 5.0, 0])


def del_all_objects():
    CLF.del_all_client()


def motion_planning_picking(all_objects_info, object_ids, R, check_ID):
    # 2020.08.03 SH
    # 1. move the hand from the home pose to the object at "objects_expand[object_index_done]",
    import numpy as np
    import math
    from tf.transformations import quaternion_from_euler
    import time
    add_objects(all_objects_info, object_ids, R)
    tic_feasible = time.time()
    # print 'Motion planning start'
    planner_name = 'RRTConnect'
    # planner_name = 'BiTRRT'
    n_attempt = 5
    c_time = 0.5
    n_repeat = 1
    start_state = moveit_msgs.msg.RobotState()
    joint_state = sensor_msgs.msg.JointState()
    joint_state.header = std_msgs.msg.Header()
    joint_state.name = ['RSP', 'RSR', 'RSY', 'REB', 'RWY', 'RWP', 'RWY2']

    pre_app_angle = 0
    pre_goal_pitches = []
    pre_goal_pitch = np.deg2rad(pre_app_angle)
    pre_goal_pitches.append(pre_goal_pitch)
   


    for i in range(5):
        pre_goal_pitches.append(pre_goal_pitch + (i + 1) * math.radians(PRE_GRASP_ANGLE_UNIT))
        pre_goal_pitches.append(pre_goal_pitch - (i + 1) * math.radians(PRE_GRASP_ANGLE_UNIT))

    pre_goal_orientations = []
    
    for i in pre_goal_pitches:
        pre_goal_orientations.append(quaternion_from_euler(math.radians(110.0), i, math.radians(45.0), axes='ryxz'))
    

    object_offset = 0.02
    object_r     = 0.025
    pre_grasp_l  = (0.145 + object_offset) / math.cos(deg2rad(20))
    grasp_l      = 0.12 / math.cos(deg2rad(20))

    object_name = [str(i).zfill(2) for i in object_ids]
    object_info = []

    #print all_objects_info, object_ids
    for i in object_ids:
        object_info.append(all_objects_info[i])
    
    table_heigth  = 0.605
    object_height = 0.12  
    robot_offset  = 0.13228

    pre_grasp_z  = table_heigth + (object_height/2) + grasp_l * math.sin(deg2rad(20)) - robot_offset
    grasp_z      = pre_grasp_z 
    post_grasp_z = grasp_z + 0.05 

    pre_goal_poses = []
    goal_poses = []
    post_goal_poses = []

    for i in pre_goal_pitches:
        pre_dx = math.cos(i) * pre_grasp_l
        pre_dy = math.sin(i) * pre_grasp_l
        dx = math.cos(i) * grasp_l 
        dy = math.sin(i) * grasp_l
        
        pre_goal_poses.append([object_info[check_ID][1] - pre_dx, -object_info[check_ID][0] + pre_dy, pre_grasp_z])
        goal_poses.append([object_info[check_ID][1] - dx, -object_info[check_ID][0] + dy, grasp_z])
        post_goal_poses.append([object_info[check_ID][1] - dx, -object_info[check_ID][0] + dy, post_grasp_z])

    

    pre_grasp_feasibility  = 0
    grasp_feasibility      = 0
    post_grasp_feasibility = 0

    i = 0

    while not(pre_grasp_feasibility) :
        print "try", i, "th pre-grasp" 
        [pre_grasp_feasibility, trajectory1] = CLF.move_goalpose_client('panda_arm', 'hand', start_state, pre_goal_poses[i], pre_goal_orientations[i], [], planner_name, n_attempt, c_time, n_repeat,PRE_GRASP_MAX_STEP)
        
        i=i+1          
        if pre_grasp_feasibility == 1 :
            idx = i-1
            break
        elif i == len(pre_goal_pitches) :
            print "pre-grasp failed"
            del_objects(object_ids)
            return 0

    
    # 2. attach the object (ID: objects_index_done),
    if pre_grasp_feasibility == 1:
      
        #print "~~~~~~~~~~~", pre_goal_pitches[idx]
        goal_pitch = pre_goal_pitches[idx]
        goal_pitches = []
        goal_pitches.append(goal_pitch)

        for i in goal_pitches:
            dx = math.cos(i) * grasp_l 
            dy = math.sin(i) * grasp_l
               
            goal_poses.append([object_info[check_ID][1] - dx, -object_info[check_ID][0] + dy, grasp_z])
        

        
        for i in range(5):
            goal_pitches.append(goal_pitch + (i + 1) * math.radians(GRASP_ANGLE_UNIT))
            goal_pitches.append(goal_pitch - (i + 1) * math.radians(GRASP_ANGLE_UNIT))
           
       
        #print len(goal_pitches)
        goal_orientations = []
        for i in goal_pitches:
            goal_orientations.append(quaternion_from_euler(math.radians(110.0), i, math.radians(45.0), axes='ryxz'))
 
        i = 0
        idx = 0
        
        while not(grasp_feasibility) :
         
            print  "try grasp ", i, "th"
            [grasp_feasibility, trajectory1] = CLF.move_goalpose_client('panda_arm', 'hand', start_state, goal_poses[i], goal_orientations[i], [], planner_name, n_attempt, c_time, n_repeat, GRASP_MAX_STEP)
            i = i+1       

            if grasp_feasibility == 1:
                print check_ID, "accessible",
                GCF.attach_object_on_gazebo_gripper("husky_panda","panda_link7",object_name[check_ID],"link")   
                CLF.att_box_client('hand', object_name[check_ID])
                print "pick up"
                idx = i-1                     
            elif i == len(goal_pitches) :
                print "grasp failed"
                del_objects(object_ids)
                return 0
                
    
        if grasp_feasibility == 1 :

            #print "~~~~~~~~~~~", goal_pitches[idx]
            post_goal_pitch = goal_pitches[idx]
            post_goal_pitches = []
            post_goal_pitches.append(post_goal_pitch)

            for i in range(5):
                post_goal_pitches.append(post_goal_pitch + (i + 1) * math.radians(POST_GRASP_ANGLE_UNIT))
                post_goal_pitches.append(post_goal_pitch - (i + 1) * math.radians(POST_GRASP_ANGLE_UNIT))

            post_goal_orientations = []
            
            for i in post_goal_pitches:
                post_goal_orientations.append(quaternion_from_euler(math.radians(110.0), i, math.radians(45.0), axes='ryxz'))
 
            i = 0
       
            while not(post_grasp_feasibility) :
         
                print i, "th post_grasp try", post_goal_pitches[i], "added"
                [post_grasp_feasibility, trajectory1] = CLF.move_goalpose_client('panda_arm', 'hand', start_state, post_goal_poses[i], post_goal_orientations[i], [], planner_name, n_attempt, c_time, n_repeat, POST_GRASP_MAX_STEP)
                i = i+1

                if post_grasp_feasibility :
                    break
        
                if i == len(post_goal_pitches) :
                    print "post grasp failed"
                    CLF.det_box_client(object_name[check_ID], [0, 0, 0], [0, 0, 0, 0], [0, 0, 0], 'red')
                    GCF.detach_object_on_gazebo_gripper("husky_panda","panda_link7",object_name[check_ID],"link")
                    del_objects(object_ids)
                    return 0
                    

            if post_grasp_feasibility == 1:
                print "post grasp completed"
            else : 
                print "post grasp failed"
                CLF.det_box_client(object_name[check_ID], [0, 0, 0], [0, 0, 0, 0], [0, 0, 0], 'red')
                GCF.detach_object_on_gazebo_gripper("husky_panda","panda_link7",object_name[check_ID],"link")
                del_objects(object_ids)
                return 0

        elif grasp_feasibility == 0 : 
            print "grasp failed"
            del_objects(object_ids)
            return 0
               
    # 3. back to the home pose

        app_angle = 0
        goal_pitches = []
        goal_pitch = np.deg2rad(app_angle)
        goal_pitches.append(goal_pitch)

        i=0
        for i in range(5):
            goal_pitches.append(goal_pitch + (i + 1) * math.radians(8.0))
            goal_pitches.append(goal_pitch - (i + 1) * math.radians(8.0))
        # Get the grasp orientation (currently the front direction)
        goal_orientations = []
        for i in goal_pitches:
            goal_orientations.append(quaternion_from_euler(math.radians(110.0), i, math.radians(45.0), axes='ryxz'))
           

        l = 0.12 / math.cos(deg2rad(20))

        object_name = [str(i).zfill(2) for i in object_ids]
        object_info = []
        #print all_objects_info, object_ids
        for i in object_ids:
            object_info.append(all_objects_info[i])
        #z = 0.36772 + 0.15 / 2.0 + 0.1
        z = 0.77 
        goal_poses = []
        for i in goal_pitches:
            dx = math.cos(i) * l
            dy = math.sin(i) * l

            # print "goal pose check", object_info[check_ID][1] - dx, -object_info[check_ID][0] + dy, z
            goal_poses.append([0.70 - dx, -0.02 + dy, z])
        # [object_info[i][1], -object_info[i][0], z]
        go_home_feasibility = 0
        i = 0
        while not go_home_feasibility and i < len(goal_pitches):
            # print i, "th try", goal_pitches[i], "added"
            [go_home_feasibility, trajectory1] = CLF.move_goalpose_client('panda_arm', 'hand', start_state, goal_poses[i], goal_orientations[i], [], planner_name, n_attempt, c_time, n_repeat,GO_HOME_MAX_STEP)
            idx = i
            i = i + 1
        if go_home_feasibility == 1:
            print "come back home OK"
            return go_home_feasibility
        else : 
            print "come back failed"
            CLF.det_box_client(object_name[check_ID], [0, 0, 0], [0, 0, 0, 0], [0, 0, 0], 'red')
            GCF.detach_object_on_gazebo_gripper("husky_panda","panda_link7",object_name[check_ID],"link")
            del_objects(object_ids)
            return 0    
    
    elif (pre_grasp_feasibility == 0) :
        print "grasp failed"
        del_objects(object_ids)
        return 0
    else :
        print "System Error"
        del_objects(object_ids)
        return 0


def motion_planning_placing(all_objects_info, object_ids, R, check_ID, combined_space):
    # 2020.08.05 SH
    # 1. move the hand from the home pose to the object at "objects_expand[object_index_done]",
    import numpy as np
    import math
    from tf.transformations import quaternion_from_euler
    import time
    
    print "place start!!!!"
    tic_feasible = time.time()
    # print 'Motion planning start'
    planner_name = 'RRTConnect'
    # planner_name = 'BiTRRT'

 
    n_attempt = 5
    c_time = 0.5
    n_repeat = 1
    start_state = moveit_msgs.msg.RobotState()
    joint_state = moveit_msgs.msg.RobotState()
    joint_state = sensor_msgs.msg.JointState()
    joint_state.header = std_msgs.msg.Header()
    joint_state.name = ['RSP', 'RSR', 'RSY', 'REB', 'RWY', 'RWP', 'RWY2']

    object_name = [str(i).zfill(2) for i in object_ids]
    object_info = []
    #print all_objects_info, object_ids
    for i in object_ids:
        object_info.append(all_objects_info[i])
    

    pre_place_l  = 0.12 / math.cos(deg2rad(20))
    place_l      = pre_place_l
    post_place_l = pre_place_l
 
    table_heigth  = 0.605
    object_height = 0.12  
    robot_offset  = 0.13228
    
    pre_place_offset      = 0.15
    place_offset          = 0.048
    post_place_offset     = 0.15
    
    pre_place_z   = table_heigth + (object_height/2) + pre_place_offset - robot_offset 
    place_z       = table_heigth + (object_height/2) + place_offset - robot_offset 
    post_place_z  = table_heigth + (object_height/2) + post_place_offset - robot_offset 

    pre_place_feasibility = 0
    place_feasibility = 0
    post_place_feasibility = 0
    
    # Set the place pose: 
    app_angle = 0
    pre_place_goal_pitches = []
    pre_place_goal_pitch = np.deg2rad(app_angle)
    pre_place_goal_pitches.append(pre_place_goal_pitch)
    
    i=0
    for i in range(5):
        pre_place_goal_pitches.append(pre_place_goal_pitch + (i + 1) * math.radians(PRE_PLACE_ANGLE_UNIT))
        pre_place_goal_pitches.append(pre_place_goal_pitch - (i + 1) * math.radians(PRE_PLACE_ANGLE_UNIT))
    
    # Get the place orientation (currently the front direction)
    pre_place_goal_orientations = []
    for i in pre_place_goal_pitches:
        pre_place_goal_orientations.append(quaternion_from_euler(math.radians(110.0), i, math.radians(45.0), axes='ryxz'))
      
    pre_place_goal_poses = []
    
    for i in pre_place_goal_pitches:
        pre_dx = math.cos(i) * pre_place_l
        pre_dy = math.sin(i) * pre_place_l
        pre_place_goal_poses.append([combined_space[1] - pre_dx, -combined_space[0] + pre_dy, pre_place_z])
    i=0
    while not pre_place_feasibility :
        print "try pre-place ",i,"th"  
        [pre_place_feasibility, trajectory1] = CLF.move_goalpose_client('panda_arm', 'hand', start_state, pre_place_goal_poses[i], pre_place_goal_orientations[i], [], planner_name, n_attempt, c_time, n_repeat, PRE_PLACE_MAX_STEP)
        idx = i
        i = i + 1
        #print "~~~~~~~~~~~~~~~~~~!!!!", pre_place_feasibility 
        if pre_place_feasibility == 1:
            time.sleep(0.1)    
            place_goal_pitches = []
            place_goal_pitch = pre_place_goal_pitches[idx]
            place_goal_pitches.append(place_goal_pitch)
    
            i=0
         
            for i in range(5):
                place_goal_pitches.append(place_goal_pitch + (i + 1) * math.radians(PLACE_ANGLE_UNIT))
                place_goal_pitches.append(place_goal_pitch - (i + 1) * math.radians(PLACE_ANGLE_UNIT))
    
            # Get the place orientation (currently the front direction)
            place_goal_orientations = []
            for i in place_goal_pitches:
                place_goal_orientations.append(quaternion_from_euler(math.radians(110.0), i, math.radians(45.0), axes='ryxz'))
      
            place_goal_poses = []
    
            for i in place_goal_pitches:
                dx = math.cos(i) * place_l
                dy = math.sin(i) * place_l
                place_goal_poses.append([combined_space[1] - dx, -combined_space[0] + dy, place_z])
    
            i=0 
            while not place_feasibility :
                print "try place", i , "th"
                [place_feasibility, trajectory1] = CLF.move_goalpose_client('panda_arm', 'hand', start_state, place_goal_poses[i], place_goal_orientations[i], [], planner_name, n_attempt, c_time, n_repeat, PLACE_MAX_STEP)
                idx = i
                i = i + 1
 
                if i == len(place_goal_pitches) :
                    print "place failed"
                    CLF.det_box_client(object_name[check_ID], [0, 0, 0], [0, 0, 0, 0], [0, 0, 0], 'red')
                    GCF.detach_object_on_gazebo_gripper("husky_panda","panda_link7",object_name[check_ID],"link")
                    del_objects(object_ids)
                    return 0


            if place_feasibility :
                CLF.det_box_client(object_name[check_ID], [0, 0, 0], [0, 0, 0, 0], [0, 0, 0], 'red')
                GCF.detach_object_on_gazebo_gripper("husky_panda","panda_link7",object_name[check_ID],"link")
                
                post_place_goal_pitches = []
                post_place_goal_pitch = place_goal_pitches[idx]
                post_place_goal_pitches.append(post_place_goal_pitch)
    
                i=0
                for i in range(5):
                    post_place_goal_pitches.append(post_place_goal_pitch + (i + 1) * math.radians(POST_PLACE_ANGLE_UNIT))
                    post_place_goal_pitches.append(post_place_goal_pitch - (i + 1) * math.radians(POST_PLACE_ANGLE_UNIT))
    
     
                post_place_goal_orientations = []
                for i in post_place_goal_pitches:
                    post_place_goal_orientations.append(quaternion_from_euler(math.radians(110.0), i, math.radians(45.0), axes='ryxz'))
      
                post_place_goal_poses = []
    
                for i in post_place_goal_pitches:
                    post_dx = math.cos(i) * post_place_l
                    post_dy = math.sin(i) * post_place_l
                    post_place_goal_poses.append([combined_space[1] - post_dx, -combined_space[0] + post_dy, post_place_z])
    
                i=0 
                while not post_place_feasibility :
                    print "post-place", i,"th"
                    [post_place_feasibility, trajectory1] = CLF.move_goalpose_client('panda_arm', 'hand', start_state, post_place_goal_poses[i], post_place_goal_orientations[i], [], planner_name, n_attempt, c_time, n_repeat, PLACE_MAX_STEP)
                    idx = i
                    i = i + 1
                   
                    if i == len(post_place_goal_pitches) :
                        print "post place failed"
                        CLF.det_box_client(object_name[check_ID], [0, 0, 0], [0, 0, 0, 0], [0, 0, 0], 'red')
                        GCF.detach_object_on_gazebo_gripper("husky_panda","panda_link7",object_name[check_ID],"link")
                        del_objects(object_ids)
                        return 0
                    if post_place_feasibility :
                        break
        
      
        if i == len(pre_place_goal_pitches) :
            print "pre place failed"
            CLF.det_box_client(object_name[check_ID], [0, 0, 0], [0, 0, 0, 0], [0, 0, 0], 'red')
            GCF.detach_object_on_gazebo_gripper("husky_panda","panda_link7",object_name[check_ID],"link")
            del_objects(object_ids)
            return 0
    
    app_angle = 0
    goal_pitches = []
    goal_pitch = np.deg2rad(app_angle)
    goal_pitches.append(goal_pitch)
    for i in range(5):
        goal_pitches.append(goal_pitch + (i + 1) * math.radians(8.0))
        goal_pitches.append(goal_pitch - (i + 1) * math.radians(8.0))
    # Get the grasp orientation (currently the front direction)
    goal_orientations = []
    for i in goal_pitches:
        goal_orientations.append(quaternion_from_euler(math.radians(90.0), i, math.radians(45.0), axes='ryxz'))
        # goal_orientations.append(quaternion_from_euler(math.radians(-90.0), math.radians(-90.0) + i, 0, axes='rxyz'))

    l = 0.12

    object_name = [str(i).zfill(2) for i in object_ids]
    object_info = []
    #print all_objects_info, object_ids
    for i in object_ids:
        object_info.append(all_objects_info[i])
    z = 0.36772 + 0.15 / 2.0 + 0.1

    goal_poses = []
    for i in goal_pitches:
        dx = math.cos(i) * l
        dy = math.sin(i) * l

        # print "goal pose check", object_info[check_ID][1] - dx, -object_info[check_ID][0] + dy, z
        goal_poses.append([0.70 - dx, -0.02 + dy, 0.77])
    # [object_info[i][1], -object_info[i][0], z]
    feasibility1 = 0
    i = 0
    while not feasibility1 and i < len(goal_pitches):
        # print i, "th try", goal_pitches[i], "added"
        [feasibility1, trajectory1] = CLF.move_goalpose_client('panda_arm', 'hand', start_state, goal_poses[i], goal_orientations[i], [], planner_name, n_attempt, c_time, n_repeat, GO_HOME_MAX_STEP )
        idx = i
        i = i + 1
    if feasibility1 == 1:
        print "come back home OK"
        # CLF.det_box_client(object_name[check_ID], [0, 0, 0], [0, 0, 0, 0], [0, 0, 0], 'red')
        del_objects(object_ids)
        # CLF.remove_all()
        return feasibility1


def go_home():
    #2020.08.05 SH
    move_group_name = 'panda_arm'
    home_joint = [-0.7912285295667355, -1.7449968666946676, 1.6255344777637362, -2.9980328554805484, 1.552371742049853, 1.345932931635115, 0.8050298552807971]
    CLF.move_joints_client_rad(move_group_name, home_joint)


def go_ready():
    #2020.08.05 SH
    move_group_name = 'panda_arm'
    home_joint = [-1.6238, -1.6078, -0.2229, -2.6057, 1.4646, 1.4325, -0.2159]
    CLF.move_joints_client_rad(move_group_name, home_joint)


def hand_open():
    #2020.08.05 SH
    CLF.panda_gripper_open()


def hand_close():
    #2020.08.05 SH
    CLF.panda_gripper_close()


def action_execution(goal_pose, goal_id, goal_pitch):#object_ids includes all existing objects currently, goal id is the node id
    vrep_env = mv.vrep_env_jaco()
    result = False
    planner_name = 'RRTConnect'
    #planner_name = 'BiTRRT'
    n_attempt = 1
    c_time = 0.5
    n_repeat = 1
    z = 0.082-0.1

    joint_names = ['Jaco_joint1', 'Jaco_joint2', 'Jaco_joint3' 'Jaco_joint4' 'Jaco_joint5', 'Jaco_joint6']

    home_pose = [0.13372, -0.036096, 0.64127]  # [0.02689, -0.05, 0.58]#[0.020, -0.0046, 0.673]
    home_orientation = [ 0, 0, 0.8509035, 0.525322 ]#[-0.00675, 0.044888, 0.70534, 0.70742]
    home_joint = [3.1415927410125732, 4.537856101989746, 5.93411922454834, -0.6108652353286743, 1.7453292608261108, -0.5235987901687622]
    # home_pose = [0.02689, -0.05, 0.58]
    # home_orientation = [-0.00145713772037, -0.998970756926, 0.0364956710831, 0.0268955302573]
    # home_joint = [3.1415927410125732, 4.537856101989746, 5.93411922454834, -0.6108652353286743, 1.7453292608261108, -0.5235987901687622]

    place_pose = [0.02689, 0.6, 0.15]
    place_orientation = [0.8660254, 0.0, 0.0, 0.5]

    start_state = moveit_msgs.msg.RobotState()
    joint_state = JointState()
    joint_state.header = Header()
    joint_state.position = home_joint

    start_state.joint_state = joint_state
    CLF.move_joints_client_rad('panda_arm', home_joint)
    CLF.move_hand_joints_client(0.5, 0.5, 0.5)

    # Objects
    goal_name = str(goal_id).zfill(2)

    ### Feasibility check ###
    # Get the target pose (x, y, z)
    goal_pose = [z, goal_pose[0], goal_pose[1]]

    # Set the grasp pose: substract 17cm from the z value of the object centroid
    goal_pitches = []

    goal_pitches.append(goal_pitch)
    for i in range(6):
        goal_pitches.append(goal_pitch + (i + 1) * (math.pi / 36))
        goal_pitches.append(goal_pitch - (i + 1) * (math.pi / 36))

    # Get the grasp orientation (currently the front direction)
    goal_orientations = []
    for i in goal_pitches:
        #goal_orientations.append(quaternion_from_euler(i, math.radians(-5.0), 0.0))
        goal_orientations.append(quaternion_from_euler(i, math.radians(-5.0), math.radians(90.0)))  # , axes='rxyz'))

    l = 0.03#0.000001#0.17
    goal_poses = []
    for i in goal_pitches:
        dx = math.sin(i - math.pi) * l
        dy = math.cos(i - math.pi) * l
        #goal_poses.append([z + 0.07, goal_pose[1] + dx, goal_pose[2] - dy])
        goal_poses.append([z+0.02, goal_pose[1] + dx, goal_pose[2] - dy])

    feasibility1 = 0
    feasibility2 = 0
    trajectory1 = []
    trajectory2 = []
    i = 0
    #start_joint = vrep_env.get_current_joint(joint_names)  # trajectory1.joint_trajectory.points[-1].positions
    #joint_state.position = start_joint
    #start_state.joint_state = joint_state
    while not feasibility1 and i < len(goal_pitches):
        #[feasibility1, trajectory1] = CLF.move_goalpose_execute_client('panda_arm', 'hand', start_state, goal_poses[i], goal_orientations[i], [], planner_name, n_attempt, c_time, n_repeat)
        [feasibility1, trajectory1] = CLF.move_goalpose_execute_client('panda_arm', 'hand', start_state, goal_poses[i],
                                                                       goal_orientations[i], [], planner_name,
                                                                       n_attempt, c_time, n_repeat)
        final_pose = goal_poses[i]
        final_ori = goal_orientations[i]
        i = i + 1
    #time.sleep(2)
    if not feasibility1:
        print("Execution failure for picking", goal_name)
    else:
        #CLF.move_hand_joints_client(0.5, 0.5, 0.5)
        vrep_env.close_hand(1)
        #time.sleep(1)
        start_joint = vrep_env.get_current_joint(joint_names)  #
        #start_joint = trajectory1.joint_trajectory.points[-1].positions
        joint_state.position = start_joint
        start_state.joint_state = joint_state
        #[feasibility2, trajectory2] = CLF.move_goalpose_execute_client('panda_arm', 'hand', start_state, place_pose, place_orientation, goal_name, planner_name, n_attempt, c_time, n_repeat)
        [feasibility2, trajectory2] = CLF.move_goalpose_execute_client('panda_arm', 'hand', start_state, home_pose,
                                                                       home_orientation, goal_name, planner_name,
                                                                       n_attempt, c_time, n_repeat)
        #time.sleep(2)
        CLF.del_box_client(goal_name)
        # CLF.move_hand_joints_client(0.1, 0.1, 0.1)
        vrep_env.open_hand(1)
        # time.sleep(1)
        # if not feasibility2:
        #     print "Execution failure for placing", goal_name
        # else:
        #     start_joint = vrep_env.get_current_joint(joint_names)
        #     # start_joint = trajectory2.joint_trajectory.points[-1].positions
        #     joint_state.position = start_joint
        #     start_state.joint_state = joint_state
        #     # [feasibility3, trajectory3] = CLF.move_goalpose_execute_client('panda_arm', 'hand', start_state, home_pose,
        #     #                                                                home_orientation, [], planner_name,
        #     #                                                                n_attempt, c_time, n_repeat)
        #     [feasibility3, trajectory3] = CLF.move_goalpose_execute_client('panda_arm', 'hand', start_state, home_pose,
        #                                                                    home_orientation, [], planner_name,
        #                                                                    n_attempt, c_time, n_repeat)
        #     #time.sleep(2)
        #     CLF.del_box_client(goal_name)
        #     #CLF.move_hand_joints_client(0.1, 0.1, 0.1)
        #     vrep_env.open_hand(1)
        #     #time.sleep(1)

    if feasibility1 and feasibility2:# and feasibility3:
        result = True
        print("Execution success for picking and placing " + goal_name + ".\n")


    return result, [final_pose, final_ori]



if __name__ == "__main__":
    print "This file has list of custom made functions"
    table_AL_test()
    # go_ready()