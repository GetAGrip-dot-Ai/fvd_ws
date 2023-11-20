#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16
import random 
import time
ALIGNMENT_THRESH = 0.1 # meters

class AmigaArucoNode:
    
    def __init__(self):

        # initialize values
        rospy.init_node('amiga_aruco_node', anonymous=True)
        self.markers = 1
        self.aligned = 0
        self.x = 0

        # amiga aligned publisher
        self.aligned_pub = rospy.Publisher('/amiga_aligned', Int16, queue_size=1)
        self.start = time.time()


    def determine_aligned(self):
        should_it = time.time() - self.start

        if should_it> 4:
            self.markers = random.choice([0, 22, 24, 26, 28])
            self.start = time.time()

        self.aligned_pub.publish(self.markers)
        print(self.markers)
        

if __name__ == '__main__':
    try:
        amiga_aruco_node = AmigaArucoNode()
        while not rospy.is_shutdown():
            amiga_aruco_node.determine_aligned()
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        pass