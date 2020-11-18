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
        ws_zero_pos = [0.73, -0.39]
        grid_i = [4, 40]
        GRID_SIZE = 0.01
        xi, yi = ws_zero_pos[0] + grid_i[0] * GRID_SIZE, ws_zero_pos[1] + grid_i[1] * GRID_SIZE

        pub_msg.object_name.append('can_check')
        pub_msg.object_color.append('pink')
        pub_msg.object_position.x = 0.07
        pub_msg.object_position.y = -yi
        pub_msg.object_position.z = xi
        pub_msg.object_position.x = 0.12
        pub_msg.object_position.y = -0.04
        pub_msg.object_position.z = 0.48 + 0.17
        pub_msg.object_orientation.x = -0.707
        pub_msg.object_orientation.y = 0.0
        pub_msg.object_orientation.z = -0.707
        pub_msg.object_orientation.w = 0.0
        pub_msg.object_scale.x = 0.06
        pub_msg.object_scale.y = 0.06
        pub_msg.object_scale.z = 0.2

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
