#!/usr/bin/env python


def add_box_client():
    import rospy
    from arm_move.srv._box_info_srv import *
    rospy.wait_for_service('add_box_srv')
    try:
        add_box_srv = rospy.ServiceProxy('add_box_srv', box_info_srv)
        pub_msg = box_info_srvRequest()
        pub_msg.object_name.append('eepose')
        pub_msg.object_color.append('pink')
        # Home pose check
        # pub_msg.object_position.x = 0.1660 + 0.27
        # pub_msg.object_position.y = -0.2465
        # pub_msg.object_position.z = 0.7629
        # pub_msg.object_orientation.x = 0.0013
        # pub_msg.object_orientation.y = -0.70604
        # pub_msg.object_orientation.z = 0.0486
        # pub_msg.object_orientation.w = 0.7064

        # Approach to an object check
        pub_msg.object_position.x = 0.75
        pub_msg.object_position.y = -0.3
        pub_msg.object_position.z = 0.90 + 0.06
        pub_msg.object_orientation.x = 0.000
        pub_msg.object_orientation.y = 0.0
        pub_msg.object_orientation.z = 0.00
        pub_msg.object_orientation.w = 0.0
        pub_msg.object_scale.x = 0.06
        pub_msg.object_scale.y = 0.06
        pub_msg.object_scale.z = 0.12

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
