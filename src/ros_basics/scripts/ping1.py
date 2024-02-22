#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def ping():
    rospy.init_node("ping",anonymous=True)
    pub = rospy.Publisher("/ping",String,queue_size=10)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        word = "Ping"
        pub.publish(word)
        rospy.loginfo("I'm the ping node I say : %s", word)
        rate.sleep()

if __name__=='__main__':
    try:
        ping()
    except rospy.ROSInterruptException:
        pass