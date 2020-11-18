#!/usr/bin/env python

import rospy
from arm_move.srv._box_info_srv import *

def add_box_client():
    rospy.wait_for_service('add_box_srv')
    try:
        add_box_srv = rospy.ServiceProxy('add_box_srv', box_info_srv)
        # pub_msg = box_info_msg()
        pub_msg = box_info_srvRequest()
        # pub_srv.header = 0
        pub_msg.object_name.append('box4')
        pub_msg.object_color.append('pink')
        pub_msg.object_position.x = 0.8637+0.5*0.45
        pub_msg.object_position.y = 0.0
        pub_msg.object_position.z = 0.605
        pub_msg.object_orientation.x = 0.0
        pub_msg.object_orientation.y = 0.0
        pub_msg.object_orientation.z = 0.0
        pub_msg.object_orientation.w = 0.0
        pub_msg.object_scale.x = 0.05
        pub_msg.object_scale.y = 0.05
        pub_msg.object_scale.z = 0.1

        resp1 = add_box_srv(pub_msg)

        print "response", resp1
        if resp1.w_flag == 1:
            print "work done"
            return resp1.w_flag
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

if __name__ == '__main__':
    print "Requesting for adding a box in moveIT"
    add_box_client()
