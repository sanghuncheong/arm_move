#!/usr/bin/env python

import rospy
from arm_move.srv._arm_goalJoint_srv import *

def move_joints_client():
    rospy.wait_for_service('arm_goalJoint_srv')
    try:
        m_joints_srv = rospy.ServiceProxy('arm_goalJoint_srv', arm_goalJoint_srv)
        pub_msg = arm_goalJoint_srvRequest()
        pub_msg.arm_name.append('arm')
        # jaco home pose
        pub_msg.goalPose.position = [3.1415927410125732, 4.537856101989746, 5.93411922454834, -0.6108652353286743, 1.7453292608261108, -0.5235987901687622]

        # pub_msg.arm_name.append('r_arm')
        # pub_msg.goalPose.position = [0.6, -0.3, -0.05, -2.25, -1.59, 0.3, 0.01]

        resp1 = m_joints_srv(pub_msg)

        print "response", resp1
        if resp1.w_flag == 1:
            print "work done"
            return resp1.w_flag
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

if __name__ == '__main__':
    print "Requesting for move joints of a robot in moveIT"
    move_joints_client()
