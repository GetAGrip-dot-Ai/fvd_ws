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

        self.peduncle_poses_base_pub = rospy.Publisher("/visualization_peduncle_poses_base", PoseArray, queue_size=1)

        self.fruit_marker_rs_pub = rospy.Publisher("/visualization_pepper_marker_rs", Marker, queue_size=1)
        self.fruit_marker_base_pub = rospy.Publisher("/visualization_pepper_marker_base", Marker, queue_size=1)

        self.image_pub = rospy.Publisher('/pepper_yolo_results', Image, queue_size=1)

        # Subscribers
        self.camera_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_callback)
        self.depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
        self.image_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        
        ts = message_filters.TimeSynchronizer([self.image_sub, self.depth_sub], queue_size=1)
        ts.registerCallback(self.img_depth_callback)

        self.user_input_sub = rospy.Subscriber('/user_selected_points', String, self.user_input_callback, queue_size=1)

        # store the results of YOLO
        self.fruit_count = 0
        self.peduncle_count = 0
        self.pepper_count = 0

        self.fruit_detections = dict()
        self.peduncle_detections = dict()
        self.pepper_detections = dict()

        # visualization
        self.image_count = 0

        # user input
        self.user_selected_px = [-1, -1]
        self.user_select_poi_bs = Pose()
        self.poi_pub = rospy.Publisher('/poi', Pose, queue_size=1)
        self.user_poi_calculated = False
        
    def user_input_callback(self, msg):
        # the message is a String in the form of (x, y)
        # need to extract x, y and save it to user_selected_poi
        

        self.user_selected_px = [int(msg.data.split(',')[1]), int(msg.data.split(',')[0])]

    def img_depth_callback(self, img, depth_img):
        
        assert img.header.stamp == depth_img.header.stamp
        
        synced_time = img.header.stamp
        try:
            transformation = self.tfBuffer.lookup_transform("link_base", "camera_color_optical_frame", synced_time, rospy.Duration(0.1))
            if self.user_selected_px[0] > 0:
                self.user_select_pepper(img, depth_img, transformation)
                self.start_time = time.time()
                # for 10 seconds, publish to the poi topic
                while time.time() < self.start_time + 5:
                    self.poi_pub.publish(self.user_select_poi_bs)
                    rospy.logwarn(f"{self.user_select_poi_bs.position.x}, {self.user_select_poi_bs.position.y}, {self.user_select_poi_bs.position.z}")

                rospy.logwarn("donme publishing")
            else:
                # rospy.logwarn("in normal mode")
                self.detect_peppers(img, depth_img, transformation)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Error getting the transform")
        

    def detect_peppers(self, img, depth, transformation):
        try:
            # Convert ROS Image message to OpenCV image
            image = self.bridge.imgmsg_to_cv2(img, desired_encoding='passthrough')
            depth_img = self.bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough')

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

                self.empty_visualization_markers()
                self.calculate_pepper_poses(depth_img, transformation)
                self.publish_visualization_markers()

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

    def user_pose(self, depth_img, transformation):
        while self.user_selected_px == [-1, -1]:
            continue
        if not self.user_poi_calculated:
            x, y = self.user_selected_px

            z = self.get_depth(depth_img, x, y) #max(min(self.get_depth(depth_img, x, y), fruit_depth + 0.03), fruit_depth)     

            # RS axes
            X_rs, Y_rs, Z_rs = self.get_3D_coords(x, y, z)

            X_b, Y_b, Z_b = transform_to_base_frame(
                transformation, X_rs, Y_rs, Z_rs)
            self.user_select_poi_bs.position.x = X_b
            self.user_select_poi_bs.position.y = Y_b
            self.user_select_poi_bs.position.z = Z_b

            self.user_poi_calculated = True
        else:
            pass


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

    def empty_visualization_markers(self):
        self.fruit_marker_rs.points = []
        self.fruit_marker_base.points = []
        self.peduncle_marker_rs.points = []
        self.peduncle_marker_base.points = []
        self.peduncle_poses_base.poses = []
    
    # calculates the fruit/peduncle poses in the base frame
    # output: self.fruit_marker_rs and self.fruit_marker_base etc
    def calculate_pepper_poses(self, depth_img, transformation):
        delete_keys = []

        for i, pepper in self.pepper_detections.items():
            fruit = pepper.pepper_fruit
            peduncle = pepper.pepper_peduncle

            fruit.xyz_rs, fruit.xyz_base = self.fruit_pose(fruit, depth_img, transformation)

            if fruit.xyz_rs is None or fruit.xyz_base is None:
                delete_keys.append(i)
                continue

            peduncle.xyz_rs, peduncle.xyz_base, peduncle.orientation_base = self.peduncle_pose(peduncle, fruit.xywh, fruit.xyz_rs[2], depth_img, transformation)

            if peduncle.xyz_rs is None or peduncle.xyz_base is None:
                delete_keys.append(i)
                continue
            
            self.fruit_marker_rs.points.append(Point(fruit.xyz_rs[0], fruit.xyz_rs[1], fruit.xyz_rs[2]))
            self.fruit_marker_base.points.append(Point(fruit.xyz_base[0], fruit.xyz_base[1], fruit.xyz_base[2]))
            self.peduncle_marker_rs.points.append(Point(peduncle.xyz_rs[0], peduncle.xyz_rs[1], peduncle.xyz_rs[2]))
            self.peduncle_marker_base.points.append(Point(peduncle.xyz_base[0], peduncle.xyz_base[1], peduncle.xyz_base[2]))
            self.peduncle_poses_base.poses.append(get_pose_object(peduncle.xyz_base, peduncle.orientation_base))

        for key in delete_keys:
            del self.pepper_detections[key]

    def publish_visualization_markers(self):
        if self.fruit_marker_rs.points == []:
            self.fruit_marker_rs.points.append(Point(0, 0, 0))
            self.fruit_marker_base.points.append(Point(0, 0, 0))
            
        self.fruit_marker_rs.header.stamp = rospy.Time.now()
        self.fruit_marker_rs_pub.publish(self.fruit_marker_rs)
        self.fruit_marker_base.header.stamp = rospy.Time.now()
        self.fruit_marker_base_pub.publish(self.fruit_marker_base)

        if self.peduncle_marker_rs.points == []:
            self.peduncle_marker_rs.points.append(Point(0, 0, 0))
            self.peduncle_marker_base.points.append(Point(0, 0, 0))
            self.peduncle_poses_base.poses.append(get_pose_object([0, 0, 0], [0, 0, 1]))

        self.peduncle_marker_rs.header.stamp = rospy.Time.now()
        self.peduncle_marker_rs_pub.publish(self.peduncle_marker_rs)
        self.peduncle_marker_base.header.stamp = rospy.Time.now()
        self.peduncle_marker_base_pub.publish(self.peduncle_marker_base)
        self.peduncle_poses_base.header.stamp = rospy.Time.now()
        self.peduncle_poses_base_pub.publish(self.peduncle_poses_base)

    # calculates the fruit center in the base_frame
    def fruit_pose(self, fruit, depth_img, transformation):
        x, y, w, h = fruit.xywh
        z = self.get_depth(depth_img, x, y)

        if z == 0 or z > 3:  # TODO must test
            return None, None

        # X, Y, Z in RS frame
        X_rs, Y_rs, Z_rs = self.get_3D_coords(x, y, z)
        
        # X, Y, Z in base frame
        X_b, Y_b, Z_b = transform_to_base_frame(transformation, X_rs, Y_rs, Z_rs)
        
        return (X_rs, Y_rs, Z_rs), (X_b, Y_b, Z_b)
    


    def peduncle_pose(self, peduncle, fruit_xywh, fruit_depth, depth_img, transformation):
        poi_px, next_point_px = peduncle.set_point_of_interaction(fruit_xywh)
        # TODO user input 

        x, y = poi_px
        x_next, y_next = next_point_px

        if x == -1 and y == -1:
            return None, None, None
        
        z = max(min(self.get_depth(depth_img, x, y), fruit_depth + 0.03), fruit_depth)        # TODO tune this
        z_next = max(min(self.get_depth(depth_img, x_next, y_next), z + 0.01), z - 0.01)    # TODO tune this

        # RS axes
        X_rs, Y_rs, Z_rs = self.get_3D_coords(x, y, z)
        X_next_rs, Y_next_rs, Z_next_rs = self.get_3D_coords(x_next, y_next, z_next)
        
        # Base frame
        X_b, Y_b, Z_b = transform_to_base_frame(transformation, X_rs, Y_rs, Z_rs)
        X_next_b, Y_next_b, Z_next_b = transform_to_base_frame(transformation, X_next_rs, Y_next_rs, Z_next_rs)
        
        return (X_rs, Y_rs, Z_rs), (X_b, Y_b, Z_b), (X_next_b - X_b, Y_next_b - Y_b, Z_next_b - Z_b)
    
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
