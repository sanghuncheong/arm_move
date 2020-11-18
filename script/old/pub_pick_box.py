#!/usr/bin/env python

import rospy
from arm_move.msg._pick_box_msg import pick_box_msg


def talker():
    pub = rospy.Publisher('pick_box', pick_box_msg, queue_size=10)
    rospy.init_node('pick_box_publisher', anonymous=True)
    pub_msg = pick_box_msg()

    pub_msg.object_name.append('box2')
    pub_msg.hand_name.append('r_hand')

    # pub_msg.object_position.x = 0.2
    # pub_msg.object_position.y = 0.2
    # pub_msg.object_position.z = 0.2
    # pub_msg.object_orientation.x = 0.0
    # pub_msg.object_orientation.y = 0.0
    # pub_msg.object_orientation.z = 0.0
    # pub_msg.object_orientation.w = 0.0
    # pub_msg.object_scale.x = 0.3
    # pub_msg.object_scale.y = 0.3
    # pub_msg.object_scale.z = 0.3

    rospy.loginfo(pub_msg)
    pub.publish(pub_msg)
        # rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass