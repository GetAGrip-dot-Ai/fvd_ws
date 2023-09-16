#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO
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


def publish_plt_image():
    # Create a ROS publisher for the Image message

    # Create a CV Bridge to convert the NumPy array to a ROS Image message
    bridge = cv_bridge.CvBridge()

    # Create a simple plot (You can replace this with your actual plotting code)
    x = [1, 2, 3, 4, 5]
    y = [10, 8, 6, 4, 2]
    plt.plot(x, y)

    # Convert the Matplotlib image to a NumPy array
    plt_image_np = np.frombuffer(
        plt.gcf().canvas.tostring_rgb(), dtype=np.uint8)
    plt_image_np = plt_image_np.reshape(
        plt.gcf().canvas.get_width_height()[::-1] + (3,))


def draw_all(img, xywh, mask):
    plt.imshow(img)

    # Draw Peduncle
    mask = mask.masks[0]
    polygon = Polygon(mask)
    x, y = polygon.exterior.xy
    plt.fill(x, y, color="blue", alpha=0.5)
    polygon = Polygon(mask)
    plt.plot(*polygon.exterior.xy)

    # draw pepper
    xywh = xywh[0]
    x = int(xywh[0])
    y = int(xywh[1])
    w = int(xywh[2])
    h = int(xywh[3])
    draw_bounding_box(0, x, y, w, h)

    # Save the plot to a BytesIO buffer as a PNG image
    buffer = io.BytesIO()
    plt.savefig(buffer, format='png')
    buffer.seek(0)

    # Convert the buffer to a NumPy array
    buffer_image = np.array(PILImage.open(buffer))

    # Close the plot to free up resources
    plt.close()

    # Convert the NumPy array to a PIL Image
    pil_image = PILImage.fromarray(buffer_image)

    # Return the PIL Image
    return pil_image


def draw_bounding_box(confidence, x, y, w, h, color="blue", fill=False):
    # Get the current reference
    ax = plt.gca()
    # plot the bounding box
    rect = patches.Rectangle((x - w / 2, y - h / 2), w,
                             h, linewidth=2, edgecolor=color, facecolor='none')
    # Add the patch to the Axes
    ax.add_patch(rect)


def draw_bounding_polygon(confidence, mask, img_shape, color="blue", fill=True):
    mask = mask.xy[0]
    polygon = Polygon(mask)
    x, y = polygon.exterior.xy
    if fill:
        plt.fill(x, y, color=color, alpha=0.7)
    polygon = Polygon(mask)
    plt.plot(*polygon.exterior.xy)


class PerceptionNode:
    def __init__(self):
        rospy.init_node('perception_node')
        self.bridge = CvBridge()

        curr_path = os.getcwd()
        print(curr_path)

        # Define the YOLO model
        rospack = rospkg.RosPack()
        package_name = 'visual_servo'
        package_path = rospack.get_path(package_name)

        self.yolo_pepper = YOLO(
            package_path+'/weights/pepper_fruit_best_4.pt')
        self.yolo_peduncle = YOLO(
            package_path+'/weights/pepper_peduncle_best_4.pt')

        # Define the RealSense image subscriber
        self.depth_subscriber = rospy.Subscriber(
            '/camera/depth/image_rect_raw', Image, self.depth_callback, queue_size=1)
        self.image_subscriber = rospy.Subscriber(
            '/camera/color/image_raw', Image, self.image_callback, queue_size=1)
        self.image_publisher = rospy.Publisher(
            '/pepper_yolo_results', Image, queue_size=1)
        self.pepper_center_publisher = rospy.Publisher(
            '/pepper_center', Point, queue_size=1)
        self.peduncle_center_publisher = rospy.Publisher(
            '/peduncle_center', Point, queue_size=1)

        self.pepper_center = None
        self.peduncle_center = None
        self.depth_image = None

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough')
            # cv2.imshow("Image window", cv_image)
            if cv_image is not None:
                _ = self.run_yolo(cv_image)

        except CvBridgeError as e:
            rospy.logerr("Error converting from image message: {}".format(e))
            return

        # Run YOLO on the image
        # detected_image = self.run_yolo(cv_image)

    def run_yolo(self, image):
        results_pepper = self.yolo_pepper(image)
        results_peduncle = self.yolo_peduncle(image)
        # pil_image = None
        # xywh = None
        # mask = None
        for result in results_pepper:
            boxes = result.boxes  # Boxes object for bbox outputs

            if boxes.xyxy.numpy().size != 0:
                box = boxes.xyxy[0]  # only take the first bb
                self.pepper_center = Point()

                # These are in RealSense coordinate system
                self.pepper_center.x = int((box[0] + box[2]) / 2)
                self.pepper_center.y = int((box[1] + box[3]) / 2)

                # Depth image is a numpy array so switch coordinates
                # Depth is converted from mm to m
                self.pepper_center.z = 0.001*self.depth_image[self.pepper_center.y,
                                                              self.pepper_center.x]

                self.pepper_center_publisher.publish(self.pepper_center)

                p1 = (int(box[0]), int(box[1]))
                p2 = (int(box[2]), int(box[3]))
                cv2.rectangle(image, p1, p2, (0, 0, 255), 10)

        for result in results_peduncle:
            mask = result.masks
            boxes = result.boxes
            if boxes.xyxy.numpy().size != 0:
                box_peduncle = boxes.xyxy[0]
                box = boxes.xyxy[0]  # only take the first bb

                self.peduncle_center = Point()

                # These are in RealSense coordinate system
                self.peduncle_center.x = int((box[0] + box[2]) / 2)
                self.peduncle_center.y = int((box[1] + box[3]) / 2)

                # Depth image is a numpy array so switch coordinates
                # Depth is converted from mm to m
                self.peduncle_center.z = 0.001*self.depth_image[self.peduncle_center.y,
                                                                self.peduncle_center.x]

                self.peduncle_center_publisher.publish(self.peduncle_center)

                p3 = (int(box_peduncle[0]), int(box_peduncle[1]))
                p4 = (int(box_peduncle[2]), int(box_peduncle[3]))
                cv2.rectangle(image, p3, p4, (255, 0, 0), 10)

        try:
            image_msg_bb = self.bridge.cv2_to_imgmsg(image, "rgb8")
            self.image_publisher.publish(image_msg_bb)
            print("published image")

        except CvBridgeError as e:
            rospy.logerr(
                "Error converting back to image message: {}".format(e))
            return

        # pil_image = draw_all(image, xywh, mask)
        # np_image = np.array(pil_image)

        # # Convert BGR image to RGB (OpenCV uses BGR by default)
        # rgb_image = np_image
        # # Display the image using cv2.imshow()
        # cv2.imshow('Image', rgb_image)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        return image

    def depth_callback(self, msg):
        try:
            # Convert ROS image message to OpenCV image
            self.depth_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough')

        except CvBridgeError as e:
            rospy.logerr(
                "Error converting from depth image message: {}".format(e))


if __name__ == '__main__':
    try:
        perception_node = PerceptionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
