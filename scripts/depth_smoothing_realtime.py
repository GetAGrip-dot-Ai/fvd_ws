#!/usr/bin/env python3

import numpy as np
import cv2
from sensor_msgs.msg import Image
import rospy
import cv_bridge


def preprocess_depth_image(depth_image):
    # Downscale the image for faster processing
    small_image = cv2.resize(depth_image, (0, 0), fx=0.5, fy=0.5)

    # Convert the depth image to a floating-point format
    float_image = small_image.astype(np.float32)

    # Replace 'inf' and '0' values with np.nan for processing
    float_image[small_image == 0] = np.nan

    return float_image


def fill_depth_gaps(depth_image):
    # Convert NaNs to zero for inpainting
    temp_image = np.nan_to_num(depth_image, nan=0.0)

    # Create a mask of zero values
    mask = (temp_image == 0).astype(np.uint8) * 255

    # Apply inpainting (consider using a faster method or limiting area)
    inpainted_image = cv2.inpaint(temp_image, mask, 3, cv2.INPAINT_TELEA)

    # Restore NaN values
    inpainted_image[depth_image == 0] = np.nan

    # Upscale the image back to original size if needed
    # inpainted_image = cv2.resize(inpainted_image, (original_width, original_height))

    return inpainted_image


def depth_callback(msg):
    depth_image_msg = msg
    depth_image = bridge.imgmsg_to_cv2(depth_image_msg)

    preprocessed_image = preprocess_depth_image(depth_image)
    smoothed_image = fill_depth_gaps(preprocessed_image)
    smoothed_image_msg = bridge.cv2_to_imgmsg(
        smoothed_image, "32FC1")
    smooth_pub.publish(smoothed_image_msg)


bridge = cv_bridge.CvBridge()

if __name__ == '__main__':
    try:
        rospy.init_node('depth_smoothing_node', anonymous=True)
        sub = rospy.Subscriber(
            '/camera/aligned_depth_to_color/image_raw', Image, depth_callback)
        smooth_pub = rospy.Publisher("/smooth_depth", Image, queue_size=1)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
