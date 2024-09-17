#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'Xu %s', data.data)

def subs():

    rospy.init_node('subscriber', anonymous=True)

    rospy.Subscriber('thetopic', String, callback)

    rospy.spin()

if __name__ == '__main__':
    subs()
