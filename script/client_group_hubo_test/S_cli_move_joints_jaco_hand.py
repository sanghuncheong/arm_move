#!/usr/bin/env python

import rospy
from arm_move.srv._arm_goalJoint_srv import *

def move_joints_client(j1, j2, j3):
    rospy.wait_for_service('jaco_hand_goalJoint_srv')
    try:
        m_joints_srv = rospy.ServiceProxy('jaco_hand_goalJoint_srv', arm_goalJoint_srv)
        pub_msg = arm_goalJoint_srvRequest()
        pub_msg.arm_name.append('hand')
        pub_msg.goalPose.position = [j1, j2, j3]

        # pub_msg.arm_name.append('r_arm')
        # pub_msg.goalPose.position = [0.6, -0.3, -0.05, -2.25, -1.59, 0.3, 0.01]

        resp1 = m_joints_srv(pub_msg)

        print "response", resp1
        if resp1.w_flag == 1:
            print "work done"
            return resp1.w_flag
        else:
            print "can not close more"
            return resp1.w_flag

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

if __name__ == '__main__':
    print "Requesting for move joints of a robot in moveIT"
    print "Close hand"
    j1, j2, j3 = 0.3, 0.3, 0.3
    end = move_joints_client(j1, j2, j3)