#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def publi():
    pub = rospy.Publisher('thetopic', String, queue_size=10)
    rospy.init_node('publisher', anonymous=True)
    rate = rospy.Rate(1) 
    while not rospy.is_shutdown():
        h_str = "Henghao %s" % rospy.get_time()
        rospy.loginfo(h_str)
        pub.publish(h_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        publi()
    except rospy.ROSInterruptException:
        pass
