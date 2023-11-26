#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16

latest_msg = Int16()


def callback(msg):
    global latest_msg
    latest_msg = msg


if __name__ == '__main__':
    rospy.init_node('state_republisher')
    rospy.Subscriber('/state', Int16, callback)
    pub = rospy.Publisher('/system_state', Int16, queue_size=10)
    rate = rospy.Rate(30)  # 10 Hz
    while not rospy.is_shutdown():
        pub.publish(latest_msg)
        rate.sleep()
