#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO
import ultralytics
import torch

import os
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
import torch
from PIL import Image as PILImage
from shapely import Polygon
import cv_bridge
import io
import rospkg
from geometry_msgs.msg import Point


class PerceptionNode:
    def __init__(self):
        ultralytics.checks()

        rospy.init_node('perception_node')
        self.bridge = CvBridge()

        self.curr_path = os.getcwd()

        # Define the YOLO model
        rospack = rospkg.RosPack()
        package_name = 'visual_servo'
        package_path = rospack.get_path(package_name)

        self.yolo_pepper = YOLO(
            package_path+'/weights/pepper_fruit_best_4.pt')
        # self.yolo_pepper.to(0)
        self.yolo_peduncle = YOLO(
            package_path+'/weights/pepper_peduncle_best_4.pt')
        # self.yolo_peduncle.to(0)

        

        # Define the RealSense image subscriber
        self.image_subscriber = rospy.Subscriber(
            '/camera/color/image_raw', Image, self.image_callback, queue_size=1)
        self.image_publisher = rospy.Publisher(
            '/pepper_yolo_results', Image, queue_size=1)
        self.detected_pepper = False
        self.detected_peduncle = False
        self.image_count = 0

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough')
            if cv_image is not None:
                _ = self.run_yolo(cv_image)

        except CvBridgeError as e:
            rospy.logerr("Error converting from image message: {}".format(e))
            return

    def run_yolo(self, image):

        results_pepper = self.yolo_pepper(image)
        results_peduncle = self.yolo_peduncle(image)

        for result in results_pepper:
            boxes = result.boxes  # Boxes object for bbox outputs

            if boxes.xyxy.numpy().size != 0:
                box = boxes.xyxy[0]  # only take the first bb
                self.pepper_center = Point()

                p1 = (int(box[0]), int(box[1]))
                p2 = (int(box[2]), int(box[3]))
                cv2.rectangle(image, p1, p2, (0, 0, 255), 10)
                self.detected_pepper = True

        for result in results_peduncle:
            mask = result.masks
            boxes = result.boxes

            if boxes.xyxy.numpy().size != 0:
                box_peduncle = boxes.xyxy[0]
                box = boxes.xyxy[0]  # only take the first bb

                p3 = (int(box_peduncle[0]), int(box_peduncle[1]))
                p4 = (int(box_peduncle[2]), int(box_peduncle[3]))
                cv2.rectangle(image, p3, p4, (255, 0, 0), 10)
                self.detected_peduncle = True

        try:
            image_msg_bb = self.bridge.cv2_to_imgmsg(image, "rgb8")
            self.image_publisher.publish(image_msg_bb)
            if self.detected_pepper and self.detected_peduncle:
                cv2.imwrite(f"{self.curr_path}/log/both_{self.image_count}.jpg", image)
                self.image_count+=1
                self.detected_pepper = False
                self.detected_peduncle = False

            elif self.detected_peduncle:
                cv2.imwrite(f"{self.curr_path}/log/peduncle_{self.image_count}.jpg", image)
                self.image_count+=1
                self.detected_peduncle = False
            elif self.detected_pepper:
                cv2.imwrite(f"{self.curr_path}/log/pepper_{self.image_count}.jpg", image)
                self.image_count+=1
                self.detected_pepper = False

        except CvBridgeError as e:
            rospy.logerr(
                "Error converting back to image message: {}".format(e))
            return

        return image


if __name__ == '__main__':
    try:
        perception_node = PerceptionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
