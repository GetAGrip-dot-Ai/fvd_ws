#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class DepthCenterValue:
    def __init__(self):
        # Initialize the node
        rospy.init_node('depth_center_value_node', anonymous=True)

        # Create CV bridge
        self.bridge = CvBridge()

        # Subscribe to the realsense2_camera depth aligned image topic
        self.depth_subscriber = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback)

        rospy.spin()

    def depth_callback(self, data):
        try:
            # Convert ROS Image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
            
            # Fetch the depth value at the center of the image
            center_depth = cv_image[cv_image.shape[0]//2, cv_image.shape[1]//2]
            
            # Print the depth value
            rospy.loginfo("Depth at center: %s mm", center_depth)

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

if __name__ == '__main__':
    try:
        DepthCenterValue()
    except rospy.ROSInterruptException:
        pass
