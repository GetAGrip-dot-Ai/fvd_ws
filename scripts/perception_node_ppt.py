#!/usr/bin/env python3

import time
import rospy
import rospkg
import cv_bridge

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Int16, Bool
from geometry_msgs.msg import Point,  PoseArray
from visualization_msgs.msg import Marker

import message_filters

import cv2
import numpy as np
from ultralytics import YOLO

import tf2_ros
from scipy.spatial.transform import Rotation as R

from pepper_util import PepperPeduncle, PepperFruit, Pepper
from match_peppers_util import match_pepper_fruit_peduncle
from perception_util import *

class PerceptionNode:

    def __init__(self):


        rospy.init_node('perception_node', anonymous=True)
        self.bridge = cv_bridge.CvBridge()
        self.depth_window = 2  # 5 x 5 window (i-2 to i+2)
        self.camera_matrix = None
        self.img_width = 640
        self.img_height = 480

        rospack = rospkg.RosPack()
        package_name = 'peter'
        package_path = rospack.get_path(package_name)

        # Define the YOLO model
        self.yolo = YOLO(
            package_path+'/weights/celery.pt')
        
        # Make marker for visualization
        self.peduncle_marker_rs = make_marker(frame_id="camera_color_optical_frame")
        self.peduncle_marker_base = make_marker(marker_type=8, frame_id='link_base', r= 1, g=0, b=1, a=1, x=0.02, y=0.01)
        self.fruit_marker_rs = make_marker(marker_type=8, frame_id='camera_color_optical_frame', r= 1, g=0, b=0, a=1, x=0.05, y=0.05)
        self.fruit_marker_base = make_marker(marker_type=8, frame_id='link_base', r= 0, g=1, b=0, a=1, x=0.06, y=0.06)

        self.peduncle_poses_base = PoseArray()
        self.peduncle_poses_base.header.frame_id = 'link_base'

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Publishers
        self.peduncle_marker_rs_pub = rospy.Publisher("/visualization_peduncle_marker_rs", Marker, queue_size=1)
        self.peduncle_marker_base_pub = rospy.Publisher("/visualization_peduncle_marker_base", Marker, queue_size=1)

        self.image_pub = rospy.Publisher('/pepper_yolo_results_ppt', Image, queue_size=1)

        # Subscribers
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.img_depth_callback)
        
        # store the results of YOLO
        self.fruit_count = 0
        self.peduncle_count = 0
        self.pepper_count = 0
        self.fruit_detections = dict()
        self.peduncle_detections = dict()
        self.pepper_detections = dict()

        # visualization
        self.image_count = 0

    def img_depth_callback(self, img):
        self.detect_peppers(img)
        

    def detect_peppers(self, img):
        print("detecting peppers")
        try:
            # Convert ROS Image message to OpenCV image
            image = self.bridge.imgmsg_to_cv2(img, desired_encoding='passthrough')

            if image is not None:
                self.run_yolo(image)

                pepper_fruit_peduncle_match = match_pepper_fruit_peduncle(self.fruit_detections, self.peduncle_detections, self.image_count, image)

                for (pfn, ppn), _ in pepper_fruit_peduncle_match:
                    if ppn == -1:
                        continue
                    else:
                        pepper = Pepper(self.pepper_count, pfn, ppn)
                        pepper.pepper_fruit = self.fruit_detections[pfn]
                        pepper.pepper_fruit.parent_pepper = self.pepper_count
                        pepper.pepper_peduncle = self.peduncle_detections[ppn]
                        pepper.pepper_peduncle.parent_pepper = self.pepper_count
                        self.pepper_detections[self.pepper_count] = pepper
                        self.pepper_count += 1


                self.fruit_count, self.peduncle_count = 0, 0
                self.fruit_detections = dict()
                self.peduncle_detections = dict()

                if self.pepper_detections != dict():
                    for i, pepper in self.pepper_detections.items():
                        rand_color = random_color()
                        image = self.visualize_result(image, pepper.pepper_fruit.segment, poi=None, color=rand_color)
                        image = self.visualize_result(image, pepper.pepper_peduncle.segment, poi=pepper.pepper_peduncle.poi_px, color=rand_color)

                try:
                    image_msg_bb = self.bridge.cv2_to_imgmsg(image, "rgb8")
                    self.image_pub.publish(image_msg_bb)

                except cv_bridge.CvBridgeError as e:
                    rospy.logerr(
                        "Error converting back to image message: {}".format(e))
                    
                self.pepper_count = 0
                self.pepper_detections = dict()
    
        except cv_bridge.CvBridgeError as e:
            rospy.logerr("Error converting from image message: {}".format(e))
            return

    def run_yolo(self, image):

        results_both = self.yolo(image, verbose=False)
        result = results_both[0]                       

        peduncle_number = next((key for key, value in result.names.items() if 'peduncle' in value), None)
        if len(result.boxes) != 0:                              # if there is a detection
            for i in range(len(result.masks)):                  # for each mask
                segment = result.masks.xyn[i]                   # only boundary of mask
                mask = result.masks.data[i]                     # mask with 1s and 0s
                cls = result.boxes.cls[i]                       # 0 is pepper, 1 is peduncle

                xywh = result.boxes.xywh[i].cpu().numpy()
                xywh[0], xywh[1] = int(xywh[1]), int(xywh[0])   # Switch from YOLO axes to NumPy axes
                
                if cls == peduncle_number:                      # it is a peduncle
                    peduncle_detection = PepperPeduncle(self.peduncle_count, xywh=xywh, mask=np.array(mask.cpu()), segment=segment)
                    self.peduncle_detections[self.peduncle_count] = peduncle_detection
                    self.peduncle_count += 1

                else:                                           # it is a pepper
                    pepper_detection = PepperFruit(self.fruit_count, xywh=xywh, mask=np.array(mask.cpu()), segment=segment)
                    self.fruit_detections[self.fruit_count] = pepper_detection
                    self.fruit_count+= 1
            self.image_count+=1

    def visualize_result(self, image, segment, poi=None, color=(100, 0, 125, 0.1)):
        mask_coords = (segment @ np.array([[self.img_width, 0], [0, self.img_height]])).astype(int)
        image = cv2.fillPoly(image, pts=[mask_coords], color=color)
        if poi is not None:
            image = cv2.circle(image, (int(poi[1]), int(poi[0])), 5, (0, 0, 255), -1)
        return image

if __name__ == '__main__':
    try:
        perception_node = PerceptionNode()
        print("huh")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
