#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16

def publish_numbers():
    rospy.init_node('number_publisher', anonymous=True)
    system_pub = rospy.Publisher('/state', Int16, queue_size=10)
    amiga_pub = rospy.Publisher('/amiga_state', Int16, queue_size=10)
    rate = rospy.Rate(1)  # 0.2 Hz, equivalent to a 5-second delay

    number = 2
    a_num = 0
    while not rospy.is_shutdown():
        sys_msg = Int16()
        sys_msg.data = number
        system_pub.publish(sys_msg)

        ami_msg = Int16()
        ami_msg.data = a_num
        amiga_pub.publish(ami_msg)
        
        number += 1
        if number > 10:
            number = 0
        a_num += 1
        if a_num > 3:
            a_num = 0

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_numbers()
    except rospy.ROSInterruptException:
        pass
