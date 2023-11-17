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

from perception_util import *

class PerceptionNode:

    def __init__(self):


        rospy.init_node('user_input_node', anonymous=True)
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
            package_path+'/weights/levelb_recent.pt')
        
        # Make marker for visualization
        self.peduncle_marker_rs = make_marker(frame_id="camera_color_optical_frame")
        self.peduncle_marker_base = make_marker(marker_type=8, frame_id='link_base', r= 1, g=0, b=1, a=1, x=0.02, y=0.01)
        self.fruit_marker_rs = make_marker(marker_type=8, frame_id='camera_color_optical_frame', r= 1, g=0, b=0, a=1, x=0.05, y=0.05)
        self.fruit_marker_base = make_marker(marker_type=8, frame_id='link_base', r= 0, g=1, b=0, a=1, x=0.06, y=0.06)

        self.peduncle_poses_base = PoseArray()
        self.peduncle_poses_base.header.frame_id = 'link_base'

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Subscribers
        self.camera_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_callback)
        self.depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
        self.image_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        
        ts = message_filters.TimeSynchronizer([self.image_sub, self.depth_sub], queue_size=1)
        ts.registerCallback(self.img_depth_callback)

        _ = rospy.Subscriber('/user_selected_poi', Point, self.user_input_callback, queue_size=1)

        # user input
        self.user_selected_px = [-1, -1]
        self.user_selected_poi_bs = Pose()
        self.poi_pub = rospy.Publisher('/poi', Pose, queue_size=1)
        
    def user_input_callback(self, msg):
        # the message is a String in the form of (x, y)
        # need to extract x, y and save it to user_selected_poi
        self.user_selected_px = [int(msg.data.split(',')[0]), int(msg.data.split(',')[1])]
        rospy.logwarn("user selected px: ", self.user_selected_px)

    def img_depth_callback(self, img, depth_img):
        
        assert img.header.stamp == depth_img.header.stamp
        
        synced_time = img.header.stamp
        try:
            transformation = self.tfBuffer.lookup_transform("link_base", "camera_color_optical_frame", synced_time, rospy.Duration(0.1))
            rospy.logwarn("user selected: "+str(self.user_selected_px[0]))
            if self.user_selected_px[0] > 0:
                rospy.logwarn("in user mode")
                self.user_select_pepper(img, depth_img, transformation)

                # for 10 seconds, publish to the poi topic
                start_time = time.time()
                while time.time() < start_time + 20:
                    self.poi_pub.publish(self.user_select_poi_bs)
                rospy.logwarn("donme publishing")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Error getting the transform")

    def user_pose(self, depth_img, transformation):
        while self.user_selected_px == [-1, -1]:
            continue

        x, y = self.user_selected_px

        z = self.get_depth(depth_img, x, y) #max(min(self.get_depth(depth_img, x, y), fruit_depth + 0.03), fruit_depth)     

        # RS axes
        X_rs, Y_rs, Z_rs = self.get_3D_coords(x, y, z)

        X_b, Y_b, Z_b = transform_to_base_frame(
            transformation, X_rs, Y_rs, Z_rs)
        self.user_select_poi_bs.position.x = X_b
        self.user_select_poi_bs.position.y = Y_b
        self.user_select_poi_bs.position.z = Z_b

    def user_select_pepper(self, img, depth, transformation):

        try:
            # Convert ROS Image message to OpenCV image
            image = self.bridge.imgmsg_to_cv2(
                img, desired_encoding='passthrough')
            depth_img = self.bridge.imgmsg_to_cv2(
                depth, desired_encoding='passthrough')

            if image is not None:
                self.user_pose(depth_img, transformation)
            self.user_selected_px = [-1, -1]
        except cv_bridge.CvBridgeError as e:
            rospy.logerr("Error converting from image message: {}".format(e))
            return
    
    def empty_visualization_markers(self):
        self.fruit_marker_rs.points = []
        self.fruit_marker_base.points = []
        self.peduncle_marker_rs.points = []
        self.peduncle_marker_base.points = []
        self.peduncle_poses_base.poses = []
    
    def get_depth(self, depth_img, x, y): # TODO move to util?
        x = int(x)
        y = int(y)
        
        top_x = max(0, x - self.depth_window)
        bottom_x = min(self.img_height, x + self.depth_window)
    
        left_y = max(0, y - self.depth_window)
        right_y = min(self.img_width, y + self.depth_window)

        depth_values = depth_img[top_x:bottom_x, left_y:right_y]
        depth_values = depth_values.flatten()
        depth = 0.001*np.median(depth_values)

        depth = 0 if np.isnan(depth) else depth

        return depth
    
    def camera_info_callback(self, msg):
        # Store the camera matrix
        self.camera_matrix = np.array(msg.K, dtype=np.float64).reshape(3, 3)
        self.img_height = msg.height
        self.img_width = msg.width
        self.camera_info_sub.unregister()

    def get_3D_coords(self, x, y, z):
        """
        Input: NumPy frame: (x, y) pixels and z depth in meters
        Output: RealSense frame: X, Y, Z coordinates in meters
        """
        # Switch from NumPy axes to RealSense axes
        x, y = y, x

        Z = z
        X = (x - self.camera_matrix[0, 2]) * Z / self.camera_matrix[0, 0]
        Y = (y - self.camera_matrix[1, 2]) * Z / self.camera_matrix[1, 1]
        return X, Y, Z
    

if __name__ == '__main__':
    try:
        perception_node = PerceptionNode()
        print("huh")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
