#!/usr/bin/env python
import rospy

class class_listen_test:
    def __init__(self):
        rospy.Subscriber("topic_test", int, self.listen_topic)

    def listen_topic(self):
        print "listen start"

if __name__ == "__main__":

    s1 = class_listen_test
    rospy.spin()