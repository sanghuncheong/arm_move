#!/usr/bin/env python

import rospy
from arm_move.srv._box_info_srv import *

def del_box_client():
    rospy.wait_for_service('del_box_srv')
    try:
        del_box_srv = rospy.ServiceProxy('del_box_srv', box_info_srv)
        # pub_msg = box_info_msg()
        pub_msg = box_info_srvRequest()
        # pub_srv.header = 0
        pub_msg.object_name.append('19')
        # pub_msg.object_name.append('ore1')

        # pub_msg.object_position.x = 0.46
        # pub_msg.object_position.y = -0.2
        # pub_msg.object_position.z = 0.55
        # pub_msg.object_orientation.x = 0.0
        # pub_msg.object_orientation.y = 0.0
        # pub_msg.object_orientation.z = 0.0
        # pub_msg.object_orientation.w = 0.0
        # pub_msg.object_scale.x = 0.1
        # pub_msg.object_scale.y = 0.02
        # pub_msg.object_scale.z = 0.1

        resp1 = del_box_srv(pub_msg)

        print "response", resp1
        if resp1.w_flag == 1:
            print "work done"
            return resp1.w_flag
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

if __name__ == '__main__':
    print "Requesting for deleting a box in moveIT"
    del_box_client()
