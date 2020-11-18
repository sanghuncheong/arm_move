#!/usr/bin/env python


def add_box_client():
    import rospy
    from arm_move.srv._box_info_srv import *
    rospy.wait_for_service('add_box_srv')
    try:
        add_box_srv = rospy.ServiceProxy('add_box_srv', box_info_srv)
        pub_msg = box_info_srvRequest()
        pub_msg.object_name.append('1234')
        pub_msg.object_color.append('pink')
        pub_msg.object_position.x = 0.940
        pub_msg.object_position.y = 0.179
        pub_msg.object_position.z = 0.605
        pub_msg.object_orientation.x = -0.6383
        pub_msg.object_orientation.y = 0.2514
        pub_msg.object_orientation.z = -0.6729
        pub_msg.object_orientation.w = 0.2763
        pub_msg.object_scale.x = 0.01
        pub_msg.object_scale.y = 0.01
        pub_msg.object_scale.z = 0.01

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
