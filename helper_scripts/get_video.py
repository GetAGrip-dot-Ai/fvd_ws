#!/usr/bin/env python3
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


def callback(data):
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        print("Received an image!")
        out.write(cv_image)
        print("Image written to file.")
    except CvBridgeError as e:
        print(e)


rospy.init_node('video_recorder', anonymous=True)
bridge = CvBridge()

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('/root/catkin_ws/output/rosbag4.mp4',
                      fourcc, 20.0, (640, 480))

rospy.Subscriber('/pepper_yolo_results_ppt', Image, callback)
print("Subscriber set up.")

rospy.spin()
out.release()
print("Video recording stopped.")
