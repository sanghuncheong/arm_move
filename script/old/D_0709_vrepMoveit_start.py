#!/usr/bin/env python
try:
    import vrep
except:
    print '--------------------------------------------------------------'
    print '"vrep.py" could not be imported. This means very probably that'
    print 'either "vrep.py" or the remoteApi library could not be found.'
    print 'Make sure both are in the same folder as this file,'
    print 'or appropriately adjust the file "vrep.py"'
    print '--------------------------------------------------------------'
    print ''

import rospy
import numpy as np
import tf
from arm_move.srv._box_info_srv import *
from arm_move.srv._arm_move_srv import *
from arm_move.srv._work_start_srv import *
from arm_move.srv._att_hand_box_srv import *
from arm_move.srv._arm_goalJoint_srv import *


def add_box_client(box_name, box_xyz, box_xyzw, box_wdh):
    rospy.wait_for_service('add_box_srv')
    try:
        add_box_srv = rospy.ServiceProxy('add_box_srv', box_info_srv)
        # pub_msg = box_info_msg()
        pub_msg = box_info_srvRequest()
        # pub_srv.header = 0
        pub_msg.object_name.append(box_name)

        pub_msg.object_position.x = box_xyz[0]
        pub_msg.object_position.y = box_xyz[1]
        pub_msg.object_position.z = box_xyz[2]
        pub_msg.object_orientation.x = box_xyzw[0]
        pub_msg.object_orientation.y = box_xyzw[1]
        pub_msg.object_orientation.z = box_xyzw[2]
        pub_msg.object_orientation.w = box_xyzw[3]
        pub_msg.object_scale.x = box_wdh[0]
        pub_msg.object_scale.y = box_wdh[1]
        pub_msg.object_scale.z = box_wdh[2]

        resp1 = add_box_srv(pub_msg)

        print "response", resp1
        if resp1.w_flag == 1:
            print "work done"
            return resp1.w_flag
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
def move_joints_client(arm_name, jointGoal):
    rospy.wait_for_service('arm_goalJoint_srv')
    try:
        m_joints_srv = rospy.ServiceProxy('arm_goalJoint_srv', arm_goalJoint_srv)
        # pub_msg = box_info_msg()
        pub_msg = arm_goalJoint_srvRequest()
        pub_msg.goalPose.name = [arm_name]
        radGoal = np.deg2rad(jointGoal)
        pub_msg.goalPose.position = radGoal

        resp1 = m_joints_srv(pub_msg)

        print "response", resp1
        if resp1.w_flag == 1:
            print "work done"
            return resp1.w_flag
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
class vrep_env_jaco:
    def __init__(self):
        print ('Program started')
        vrep.simxFinish(-1)  # just in case, close all opened connections
        self.clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # Connect to V-REP
        returnCode, self.jaco_base_world = vrep.simxGetObjectHandle(self.clientID, 'jaco_base_world', vrep.simx_opmode_blocking)

        if self.clientID!=-1:
            print"let's start vrep environment for jaco"

    def obstacle_info(self, obs_name):
        print"find", obs_name, "in vrep scene"
        returnCode, obstacle = vrep.simxGetObjectHandle(self.clientID, obs_name, vrep.simx_opmode_blocking)
        returnCode, self.obs_pos = vrep.simxGetObjectPosition(self.clientID, obstacle, self.jaco_base_world, vrep.simx_opmode_oneshot_wait)
        returnCode, self.obs_ori = vrep.simxGetObjectOrientation(self.clientID, obstacle, self.jaco_base_world, vrep.simx_opmode_oneshot_wait)
        returnCode, self.obs_ori_q = vrep.simxGetObjectQuaternion(self.clientID, obstacle, self.jaco_base_world, vrep.simx_opmode_oneshot_wait)
        returnCode, bb_min_x = vrep.simxGetObjectFloatParameter(self.clientID, obstacle, 15, vrep.simx_opmode_oneshot_wait)
        returnCode, bb_min_y = vrep.simxGetObjectFloatParameter(self.clientID, obstacle, 16, vrep.simx_opmode_oneshot_wait)
        returnCode, bb_min_z = vrep.simxGetObjectFloatParameter(self.clientID, obstacle, 17, vrep.simx_opmode_oneshot_wait)
        returnCode, bb_max_x = vrep.simxGetObjectFloatParameter(self.clientID, obstacle, 18, vrep.simx_opmode_oneshot_wait)
        returnCode, bb_max_y = vrep.simxGetObjectFloatParameter(self.clientID, obstacle, 19, vrep.simx_opmode_oneshot_wait)
        returnCode, bb_max_z = vrep.simxGetObjectFloatParameter(self.clientID, obstacle, 20, vrep.simx_opmode_oneshot_wait)
        obj_sx = bb_max_x - bb_min_x
        obj_sy = bb_max_y - bb_min_y
        obj_sz = bb_max_z - bb_min_z
        self.obs_scale = [obj_sx, obj_sy, obj_sz]

    def target(self, tar_name):
        print"find", tar_name, "in vrep scene"


if __name__ == "__main__":
    move_joints_client('arm', [180, 260, 340, -35, 100, -30])
    env = vrep_env_jaco()
    env.obstacle_info('obstacle1')
    print "obstacle1 pose:", env.obs_pos, env.obs_ori, env.obs_ori_q
    print "obstacle scale", env.obs_scale
    add_box_client('obstacle1', env.obs_pos, env.obs_ori_q, env.obs_scale)

    env.obstacle_info('obstacle2')
    print "obstacle2 pose:", env.obs_pos, env.obs_ori, env.obs_ori_q
    print "obstacle2 scale", env.obs_scale
    add_box_client('obstacle2', env.obs_pos, env.obs_ori_q, env.obs_scale)

    env.obstacle_info('obstacle3')
    print "obstacle3 pose:", env.obs_pos, env.obs_ori, env.obs_ori_q
    print "obstacle3 scale", env.obs_scale
    add_box_client('obstacle3', env.obs_pos, env.obs_ori_q, env.obs_scale)

    env.obstacle_info('obstacle4')
    print "obstacle4 pose:", env.obs_pos, env.obs_ori, env.obs_ori_q
    print "obstacle4 scale", env.obs_scale
    add_box_client('obstacle4', env.obs_pos, env.obs_ori_q, env.obs_scale)

    env.obstacle_info('obstacle5')
    print "obstacle4 pose:", env.obs_pos, env.obs_ori, env.obs_ori_q
    print "obstacle4 scale", env.obs_scale
    add_box_client('obstacle5', env.obs_pos, env.obs_ori_q, env.obs_scale)

    env.obstacle_info('target')
    print "target pose:", env.obs_pos, env.obs_ori, env.obs_ori_q
    print "target scale", env.obs_scale
    add_box_client('target', env.obs_pos, env.obs_ori_q, env.obs_scale)

    env.obstacle_info('customizableTable_forwarder')
    print "target pose:", env.obs_pos, env.obs_ori, env.obs_ori_q
    print "target scale", env.obs_scale
    add_box_client('customizableTable_forwarder', env.obs_pos, env.obs_ori_q, env.obs_scale)
