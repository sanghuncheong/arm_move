#!/usr/bin/env python

import rospy
from arm_move.msg._attach_hand_box import attach_hand_box


def talker():
    pub = rospy.Publisher('att_box_info', attach_hand_box, queue_size=10)
    rospy.init_node('box_publisher', anonymous=True)
    pub_msg = attach_hand_box()

    pub_msg.object_name.append('box2')
    pub_msg.hand_name.append('r_hand')
    # pub_msg.object_position.x = 0.5
    # pub_msg.object_position.y = -0.2
    # pub_msg.object_position.z = 0.35
    # pub_msg.object_orientation.x = 0.0
    # pub_msg.object_orientation.y = 0.0
    # pub_msg.object_orientation.z = 0.0
    # pub_msg.object_orientation.w = 0.0
    # pub_msg.object_scale.x = 0.2
    # pub_msg.object_scale.y = 0.5
    # pub_msg.object_scale.z = 0.7

    rospy.loginfo(pub_msg)
    pub.publish(pub_msg)
        # rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass