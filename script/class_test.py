#!/usr/bin/env python
import rospy
from std_msgs.msg import *
import time

def listen_topic(data):
    global gripper_encoder
    gripper_encoder = data

if __name__ == "__main__":
    global gripper_encoder  # define global gripper encoder
    gripper_encoder = -999  # init the global gripper encoder

    rospy.init_node('test_class', anonymous=True)

    time_sec = 0
    while(1):
        print "ws:", time_sec, "gripper:", gripper_encoder
        time.sleep(1)
        rospy.Subscriber("topic_test", String, listen_topic)
        time_sec = time_sec + 1