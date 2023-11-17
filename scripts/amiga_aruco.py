#!/usr/bin/env python3

import rospy
from aruco_msgs.msg import MarkerArray
from std_msgs.msg import Int16
from geometry_msgs.msg import PoseStamped

ALIGNMENT_THRESH = 0.1 # meters

class AmigaArucoNode:
    
    def __init__(self):

        # initialize values
        rospy.init_node('amiga_aruco_node', anonymous=True)
        self.markers = MarkerArray()
        self.aligned = 0
        self.x = 0

        # aruco detection subscriber
        self.aruco_sub = rospy.Subscriber("/aruco_marker_publisher/markers", MarkerArray, self.detection_callback, queue_size=1)

        # amiga aligned publisher
        self.aligned_pub = rospy.Publisher('/amiga_aligned', Int16, queue_size=1)


    def detection_callback(self,msg):
        self.markers = msg

    def determine_aligned(self):
        self.aligned = 0
        for marker in self.markers.markers:
            x_pos = marker.pose.pose.position.x
            if abs(x_pos) < ALIGNMENT_THRESH:
                self.aligned = marker.id

        self.aligned_pub.publish(self.aligned)
        self.markers = MarkerArray()
        

if __name__ == '__main__':
    try:
        amiga_aruco_node = AmigaArucoNode()
        while not rospy.is_shutdown():
            amiga_aruco_node.determine_aligned()
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        pass