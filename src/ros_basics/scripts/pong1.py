#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

pub_pong = None

def callback_ping(word):
    global pub_pong
    response = String()
    if word.data == "Ping":
        rospy.loginfo("Im the Pong node i heard: %s", word.data)
        response.data = "Pong"
        pub_pong.publish(response)
    else:
        response.data = "Failed!"
        rospy.loginfo(response.data)
        pub_pong.publish(response)

def pong():
    global pub_pong
    rospy.init_node("pong",anonymous=True)
    rospy.Subscriber("/ping",String,callback_ping)
    pub_pong = rospy.Publisher('/pong',String,queue_size=10)
    rospy.spin()

if __name__=='__main__':
    pong()