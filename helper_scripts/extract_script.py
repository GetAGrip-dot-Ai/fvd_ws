#!/usr/bin/env python3

import os
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rosbag
import cv2


def extract_images_from_bag(bag_file, topic, output_folder):
    # Check if the output folder exists, and create it if it doesn't
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Initialize CvBridge
    bridge = CvBridge()

    # Counter for image filenames
    count = 0

    # Open the rosbag file
    with rosbag.Bag(bag_file, 'r') as bag:
        # Iterate over the messages in the topic
        for topic, msg, t in bag.read_messages(topics=[topic]):
            try:
                # Convert the image message to an OpenCV image
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except CvBridge.CvBridgeError as e:
                print(e)
                continue

            # Construct the output filename
            output_filename = os.path.join(
                output_folder, f"frame_{count:04d}.png")

            # Save the image
            if count % 15 == 0:
                cv2.imwrite(output_filename, cv_image)
                print(f"Saved: {output_filename}")

            count += 1


if __name__ == '__main__':

    # sys.argv[1]
    folder_path = "/home/amiga/fvd_ws/src/peter/rosbag/"
    output_folder = "/home/amiga/fvd_ws/src/peter/rosbag_img/"
    topic = "/camera/color/image_raw"

    for bag_file in os.listdir(folder_path):
        bag_name = bag_file.split(".")[0]
        print("converting: ", bag_name)
        extract_images_from_bag(folder_path+bag_file, topic, output_folder + bag_name)

