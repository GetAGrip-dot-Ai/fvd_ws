#!/usr/bin/env python3

import numpy as np
import cv2
from sensor_msgs.msg import Image
import rospy
import cv_bridge

def preprocess_depth_image(depth_image):
    # Convert the depth image to a floating-point format
    float_image = depth_image.astype(np.float16)

    # Replace 'inf' and '0' values with np.nan for processing
    float_image[depth_image == 0] = np.nan
    # No need to replace np.inf, as it should not be present in an integer array

    return float_image


def fill_depth_gaps(depth_image):
    # Ensure depth image is in a 32-bit float format
    depth_image_32f = depth_image.astype(np.float32)

    # Convert NaNs to zero for inpainting and ensure it's in the correct format
    temp_image = np.nan_to_num(depth_image_32f, nan=0.0)

    # Create a mask of zero values (gaps to fill), and ensure it's 8-bit
    mask = (temp_image == 0).astype(np.uint8) * 255

    # Apply inpainting
    inpainted_image = cv2.inpaint(temp_image, mask, 3, cv2.INPAINT_TELEA)

    # Restore NaN values for consistency with the original depth image format
    inpainted_image[depth_image_32f == 0] = np.nan

    return inpainted_image



def depth_callback(msg):
    depth_image_msg = msg
    depth_image = bridge.imgmsg_to_cv2(depth_image_msg)

    preprocessed_image = preprocess_depth_image(depth_image)
    smoothed_image = fill_depth_gaps(preprocessed_image)
    # smoothed_image = smooth_depth_image(preprocessed_image)

    smoothed_image_msg = bridge.cv2_to_imgmsg(
        smoothed_image, "32FC1")  # Corrected encoding
    smooth_pub.publish(smoothed_image_msg)

bridge = cv_bridge.CvBridge()

if __name__ == '__main__':
    try:
        rospy.init_node('depth_smoothing_node', anonymous=True)
        sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, depth_callback)
        smooth_pub = rospy.Publisher("/smooth_depth", Image, queue_size=1)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass