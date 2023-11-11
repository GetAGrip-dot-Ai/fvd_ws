#!/usr/bin/env python3

import rospy
from aruco_msgs.msg import Marker
from std_msgs.msg import Int16
from geometry_msgs.msg import PoseStamped

ALIGNMENT_THRESH = 0.07 # meters

class AmigaArucoNode:
    
    def __init__(self):

        # initialize values
        rospy.init_node('amiga_aruco_node', anonymous=True)
        self.aligned = 0
        self.x = 0

        # aruco detection subscriber
        self.aruco_sub = rospy.Subscriber("/aruco_tracker/pose", PoseStamped, self.detection_callback, queue_size=1)
        
        # amiga aligned publisher
        self.aligned_pub = rospy.Publisher('/amiga_aligned', Int16, queue_size=1)

    def detection_callback(self,msg):
        self.x = msg.pose.position.x

    def determine_aligned(self):    
        if abs(self.x) < ALIGNMENT_THRESH and abs(self.x > 0):
            self.aligned = 1
        else:
            self.aligned = 0

        amiga_aruco_node.aligned_pub.publish(amiga_aruco_node.aligned)
        self.x = 0
        # self.pose.position.x = 1000
        

if __name__ == '__main__':
    try:
        amiga_aruco_node = AmigaArucoNode()
        while not rospy.is_shutdown():
            amiga_aruco_node.determine_aligned()
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        pass