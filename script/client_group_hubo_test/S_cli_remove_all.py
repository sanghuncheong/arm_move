#!/usr/bin/env python

import rospy
from arm_move.srv._work_start_srv import *

def rem_all_client():
    rospy.wait_for_service('remove_all_srv')
    try:
        rem_all_srv = rospy.ServiceProxy('remove_all_srv', work_start_srv)
        # pub_msg = box_info_msg()
        pub_msg = work_start_srvRequest()
        pub_msg.w_start =1

        resp1 = rem_all_srv(pub_msg)

        print "response", resp1
        if resp1.w_flag == 1:
            print "work done"
            return resp1.w_flag
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

if __name__ == '__main__':
    print "Requesting for remove all objects in moveIT"
    rem_all_client()
