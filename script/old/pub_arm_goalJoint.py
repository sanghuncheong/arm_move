#!/usr/bin/env python

import rospy
import sensor_msgs.msg

def talker():
    pub = rospy.Publisher('arm_goalJoint', sensor_msgs.msg.JointState, queue_size=10)
    rospy.init_node('box_publisher', anonymous=True)
    pub_msg = sensor_msgs.msg.JointState()

    #pub_msg.name.append('l_arm')
    #pub_msg.position.append(0.4)
    #pub_msg.position.append(0.3)
    #pub_msg.position.append(-0.054)
    #pub_msg.position.append(-2.25)
    #pub_msg.position.append(-1.59)
    #pub_msg.position.append(-0.3)
    #pub_msg.position.append(0.01)

    pub_msg.name.append('arm')
    pub_msg.position.append(1.0)
    pub_msg.position.append(3.14)
    pub_msg.position.append(3.14)
    pub_msg.position.append(0)
    pub_msg.position.append(0)
    pub_msg.position.append(0)
    # pub_msg.position.append(0)
    #
    # pub_msg.name.append('r_arm')
    # pub_msg.position.append(0.4)
    # pub_msg.position.append(-0.3)
    # pub_msg.position.append(-0.054)
    # pub_msg.position.append(-2.25)
    # pub_msg.position.append(-1.59)
    # pub_msg.position.append(0.3)
    # pub_msg.position.append(0.01)

    rospy.loginfo(pub_msg)
    pub.publish(pub_msg)
        # rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
