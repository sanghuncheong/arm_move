#!/usr/bin/env python

import rospy
from arm_move.msg._arm_move_msg import arm_move_msg


def talker():
    pub = rospy.Publisher('arm_goalPose', arm_move_msg, queue_size=10)
    rospy.init_node('arm_publisher', anonymous=True)
    pub_msg = arm_move_msg()

    pub_msg.arm_name.append('arm')
    #before table
    pub_msg.goal_position.x = 0.3
    pub_msg.goal_position.y = -0.3
    pub_msg.goal_position.z = 0.6
    pub_msg.goal_orientation.x = -0.5
    pub_msg.goal_orientation.y = 0.5
    pub_msg.goal_orientation.z = 0.5
    pub_msg.goal_orientation.w = -0.5
    #
    # # # go to box
    # pub_msg.goal_position.x = 0.4
    # pub_msg.goal_position.y = -0.
    # pub_msg.goal_position.z = 0.6
    # pub_msg.goal_orientation.x = -0.5
    # pub_msg.goal_orientation.y = 0.5
    # pub_msg.goal_orientation.z = 0.5
    # pub_msg.goal_orientation.w = -0.5
    # # # go to box left
    # pub_msg.goal_position.x = 0.42
    # pub_msg.goal_position.y = 0.31
    # pub_msg.goal_position.z = 0.6
    # pub_msg.goal_orientation.x = -0.5
    # pub_msg.goal_orientation.y = 0.5
    # pub_msg.goal_orientation.z = 0.5
    # pub_msg.goal_orientation.w = -0.5

    # rearrange box
    # pub_msg.goal_position.x = 0.4
    # pub_msg.goal_position.y = -0.45
    # pub_msg.goal_position.z = 0.7
    # pub_msg.goal_orientation.x = -0.5
    # pub_msg.goal_orientation.y = 0.5
    # pub_msg.goal_orientation.z = 0.5
    # pub_msg.goal_orientation.w = -0.5

    # # go to box
    # pub_msg.goal_position.x = 0.4
    # pub_msg.goal_position.y = -0.2
    # pub_msg.goal_position.z = 0.6
    # pub_msg.goal_orientation.x = -0.5
    # pub_msg.goal_orientation.y = 0.5
    # pub_msg.goal_orientation.z = 0.5
    # pub_msg.goal_orientation.w = -0.5
    rospy.loginfo(pub_msg)
    pub.publish(pub_msg)
        # rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass