#!/usr/bin/env python3

import rospy
import rospkg
import cv_bridge

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Int16, Bool
from geometry_msgs.msg import Point, Pose
from visualization_msgs.msg import Marker

import cv2
import torch
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image as PILImage

import message_filters

import tf2_ros
from tf.transformations import quaternion_matrix

from ultralytics import YOLO

import os
import io
import random

from pepper_util import PepperPeduncle, PepperFruit, Pepper
from match_peppers_util import match_pepper_fruit_peduncle

import matplotlib.pyplot as plt

def plot_3d_points(points1, points2, img, filename):
    # Create a new figure
    fig = plt.figure(figsize=(10, 5))  # Adjust the figure size as needed

    # Add subplot for the image
    ax1 = fig.add_subplot(1, 2, 1)
    ax1.imshow(img)
    ax1.axis('off')  # Turn off axis numbers and ticks

    # Add subplot for 3D scatter plot
    ax2 = fig.add_subplot(1, 2, 2, projection='3d')
    
    # Unpack the first set of points into x1, y1, and z1 lists
    x1, y1, z1 = zip(*points1)
    ax2.scatter(z1, x1, y1, color='blue', label='pepper', alpha=0.2, s = 1)
    
    # Unpack the second set of points into x2, y2, and z2 lists
    x2, y2, z2 = zip(*points2)
    ax2.scatter(z2, x2, y2, color='red', label='peduncle', alpha=0.2, s = 1)
    
    ax2.set_xlabel('depth (m)')
    ax2.set_ylabel('x (m)')
    ax2.set_zlabel('y (m)')

    # Set fixed axis limits and invert the necessary axes
    ax2.set_xlim(5, 0)  # x-axis (depth) from 5 to 0 meters (reversed)
    ax2.set_ylim(-1, 1)  # y-axis from -1 to 1
    ax2.set_zlim(1, 0)  # z-axis from 1 to 0 (reversed)
    
    # Add a legend to differentiate the two point sets
    ax2.legend()

    # Adjust layout
    plt.tight_layout()

    # Save the combined plot to a file
    plt.savefig(filename)

    # Close the plot to free up memory
    plt.close(fig)

class PerceptionNode:

    def __init__(self):

        rospy.init_node('perception_node', anonymous=True)
        self.bridge = cv_bridge.CvBridge()
        self.depth_window = 2  # 5 x 5 window (i-2 to i+2)
        self.camera_matrix = None
        self.img_width = 640
        self.img_height = 480

        rospack = rospkg.RosPack()
        package_name = 'fvd_ws'
        package_path = rospack.get_path(package_name)

        # Define the YOLO model
        self.yolo = YOLO(
            package_path+'/weights/red.pt')
        
        # Make marker for visualization
        self.peduncle_marker_rs =  self.make_marker(frame_id="camera_color_optical_frame")
        self.peduncle_marker_base =  self.make_marker(marker_type=8, frame_id='link_base', r= 1, g=0, b=1, a=1, x=0.02, y=0.01)
        self.pepper_marker_rs =  self.make_marker(marker_type=8, frame_id='camera_color_optical_frame', r= 1, g=0, b=0, a=1, x=0.05, y=0.05)
        self.pepper_marker_base =  self.make_marker(marker_type=8, frame_id='link_base', r= 0, g=1, b=0, a=1, x=0.06, y=0.06)

        self.peduncle_mask_rs = self.make_marker(marker_type=8, frame_id='camera_color_optical_frame', r= 1, g=0, b=0, a=1, x=0.02, y=0.02)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Publishers
        self.peduncle_marker_rs_pub = rospy.Publisher("/visualization_peduncle_marker_rs", Marker, queue_size=1)
        self.peduncle_marker_base_pub = rospy.Publisher("/visualization_peduncle_marker_base", Marker, queue_size=1)

        self.pepper_marker_rs_pub = rospy.Publisher("/visualization_pepper_marker_rs", Marker, queue_size=1)
        self.pepper_marker_base_pub = rospy.Publisher("/visualization_pepper_marker_base", Marker, queue_size=1)

        self.peduncle_mask_pub = rospy.Publisher("/visualization_peduncle_mask", Marker, queue_size=1)

        self.image_pub = rospy.Publisher('/pepper_yolo_results', Image, queue_size=1)
        self.pepper_center_pub = rospy.Publisher('/pepper_center', Point, queue_size=1)
        self.peduncle_center_pub = rospy.Publisher('/peduncle_center', Point, queue_size=1)
        self.poi_pub = rospy.Publisher('/poi', Point, queue_size=1)

        self.poi_publisher = rospy.Publisher('/perception/peduncle/poi', Pose, queue_size=10)

        # Subscribers
        self.camera_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_callback)
        self.depth_sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_callback, queue_size=1)
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback, queue_size=1)

        self.state_sub = rospy.Subscriber('/state', Int16, self.state_callback, queue_size=1)

        self.xarm_moving_sub = rospy.Subscriber('/xarm_moving', Bool, self.xarm_moving_callback, queue_size=1)

        self.poi = Point()
        self.state = None
        
        # POI publisher

        self.pepper_center = None
        self.peduncle_center = None
        self.depth_image = None

        self.peduncle_offset = 0.0       
        
        self.detection_void_count = 0
        self.last_peduncle_center = Point()
        self.last_pepper_center = Point()

        # store the results of YOLO
        self.fruit_count = 0
        self.peduncle_count = 0
        self.pepper_count = 0
        self.peduncle_detections = dict()
        self.pepper_detections = dict()
        self.fruit_detections = dict()


        # visualization
        self.image_count = 0
        self.image = np.zeros((480, 640, 3), dtype=np.uint8)
        self.vis_pepper_list = []
        self.vis_peduncle_list = []

        self.xarm_moving = False
    
    def xarm_moving_callback(self, msg):
        self.xarm_moving = msg.data
        print(self.xarm_moving)

    def image_callback(self, msg):

        try:

            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            if cv_image is not None:
                self.run_yolo(cv_image)

                pepper_fruit_peduncle_match = match_pepper_fruit_peduncle(self.fruit_detections, self.peduncle_detections)

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

                self.fruit_detections = dict()
                self.peduncle_detections = dict()

                self.image = cv_image

                if self.pepper_detections != dict():
                    for i, pepper in self.pepper_detections.items():
                        rand_color = self.random_color()
                        self.image = self.visualize_result(self.image, pepper.pepper_fruit.segment, poi=None, color=rand_color)
                        self.image = self.visualize_result(self.image, pepper.pepper_peduncle.segment, poi=pepper.pepper_peduncle.poi_px, color=rand_color)

                try:
                    image_msg_bb = self.bridge.cv2_to_imgmsg(self.image, "rgb8")
                    self.image_pub.publish(image_msg_bb)

                except cv_bridge.CvBridgeError as e:
                    rospy.logerr(
                        "Error converting back to image message: {}".format(e))
                    
                self.pepper_detections = dict()
    
        except cv_bridge.CvBridgeError as e:
            rospy.logerr("Error converting from image message: {}".format(e))
            return
        

    def run_yolo(self, image):

        results_both = self.yolo(image, verbose=False)

        self.peduncle_dict = dict()
        peduncle_count = 0

        self.pepper_center = Point() # TODO can we just do Point(x=-1, y=-1, z=0)?
        self.pepper_center.x = -1 # Assume there are no detections initially
        self.pepper_center.y = -1
        self.pepper_center.z = 0

        self.peduncle_center = Point() # TODO can we just do Point(x=-1, y=-1, z=0)?
        self.peduncle_center.x = -1 # self.img_width/2
        self.peduncle_center.y = -1 # self.img_height/2
        self.peduncle_center.z = 0

        self.pepper_marker_rs.points = []
        self.pepper_marker_base.points = []
        self.peduncle_marker_rs.points = []
        self.peduncle_marker_base.points = []
        self.peduncle_mask_rs.points = []
        result = results_both[0] # only take the first image because there is only one image
        # import ipdb;
        # ipdb.set_trace()
        if len(result.boxes) != 0: # if there is a detection

            for i in range(len(result.masks)): # for each mask
                segment = result.masks.xyn[i] # only boundary of mask
                mask = result.masks.data[i]       # mask with 1s and 0s
                box = result.boxes.xyxy[i]   
                cls = result.boxes.cls[i] # 0 is pepper, 1 is peduncle

                if cls == 1: # it is a pepper
                    pepper_detection = PepperFruit(self.fruit_count, segment=segment)

                    # TODO change (this is just a placeholder)
                    xywh = result.boxes.xywh 
                    xywh = xywh[0].cpu().numpy()
                    # Switch from YOLO axes to NumPy axes
                    xywh[0], xywh[1] = xywh[1], xywh[0]

                    pepper_detection.xywh = xywh
                    self.fruit_detections[self.fruit_count] = pepper_detection
                    self.fruit_count+= 1

                    # These are in RealSense coordinate system
                    self.pepper_center.x = int((box[1] + box[3]) / 2)
                    self.pepper_center.y = int((box[0] + box[2]) / 2)

                    # self.peduncle_offset = int((box[3] - box[1]) / 3)
                    # self.pepper_center.x -= self.peduncle_offset

                    self.pepper_center.z = self.get_depth(int(self.pepper_center.x), int(self.pepper_center.y))

                    # X, Y, Z in RS frame
                    X, Y, Z = self.get_3D_coords(
                        self.pepper_center.x, self.pepper_center.y, self.pepper_center.z)
                    
                    self.pepper_marker_rs.points.append(Point(X, Y, Z))
                    self.pepper_marker_rs.header.stamp = rospy.Time.now()
                    self.pepper_marker_rs_pub.publish(self.pepper_marker_rs)

                    # X, Y, Z in base frame
                    X_b, Y_b, Z_b = self.transform_to_base_frame(X, Y, Z)

                    self.pepper_marker_base.points.append(Point(X_b, Y_b, Z_b))
                    self.pepper_marker_base.header.stamp = rospy.Time.now()
                    self.pepper_marker_base_pub.publish(self.pepper_marker_base)

                    # if self.state != 5:
                    #     self.poi.x = X_b
                    #     self.poi.y = Y_b
                    #     self.poi.z = Z_b

                    self.last_pepper_center = self.pepper_center

                    x, y = np.where(np.array(mask.cpu()) == 1)
                    # print("there are {} points in the mask".format(len(x)))
                    xys = list(zip(x, y))
                    for x, y in xys:
                        z = self.depth_image[x, y] * 0.001
                        if z == 0:
                            continue
                        X, Y, Z = self.get_3D_coords(
                            x, y, z)
                        point = Point(X, Y, Z)
                        self.vis_pepper_list.append((X, Y, Z))

                else: # it is a peduncle
                    
                    # TODO add pepper xywh
                    peduncle_detection = PepperPeduncle(self.peduncle_count, np.array(mask.cpu()), segment=segment)
                    xywh = result.boxes.xywh[i].cpu().numpy()
                    # Switch from YOLO axes to NumPy axes
                    xywh[0], xywh[1] = xywh[1], xywh[0]

                    # visualize the peduncle mask in rviz 
                    self.peduncle_mask_points = []

                    x, y = np.where(np.array(mask.cpu()) == 1)
                    # print("there are {} points in the mask".format(len(x)))
                    xys = list(zip(x, y))
                    for x, y in xys:
                        z = self.depth_image[x, y] * 0.001
                        if z == 0:
                            continue
                        X, Y, Z = self.get_3D_coords(
                            x, y, z)
                        point = Point(X, Y, Z)
                        self.peduncle_mask_points.append(point)
                        self.vis_peduncle_list.append((X, Y, Z))
                    for point in self.peduncle_mask_points:
                        self.peduncle_mask_rs.points.append(point)
                    self.peduncle_mask_pub.publish(self.peduncle_mask_rs)
                    self.peduncle_mask_rs.points = []

                    peduncle_detection.xywh = xywh
                    poi_x, poi_y = peduncle_detection.set_point_of_interaction()

                    if poi_x == -1 and poi_y == -1:
                        continue

                    self.peduncle_detections[peduncle_count] = peduncle_detection
                    peduncle_count += 1
                                        
                    # These are in NumPy axes
                    self.peduncle_center.x = poi_x
                    self.peduncle_center.y = poi_y

                    self.peduncle_center.z = self.get_depth(int(self.peduncle_center.x), int(self.peduncle_center.y))
                    if self.peduncle_center.z < 0.2:
                        continue
                    
                    # X, Y, Z in RS axes
                    X, Y, Z = self.get_3D_coords(
                        self.peduncle_center.x, self.peduncle_center.y, self.peduncle_center.z)

                    self.peduncle_marker_rs.points.append(Point(X, Y, Z))
                    self.peduncle_marker_rs.header.stamp = rospy.Time.now()
                    self.peduncle_marker_rs_pub.publish(self.peduncle_marker_rs)

                    # Base frame
                    X_b, Y_b, Z_b = self.transform_to_base_frame(X, Y, Z)

                    self.peduncle_marker_base.points.append(Point(X_b, Y_b, Z_b))
                    self.peduncle_marker_base.header.stamp = rospy.Time.now()
                    self.peduncle_marker_base_pub.publish(self.peduncle_marker_base)
                    
                    if self.state != 5:
                        self.poi.x = X_b
                        self.poi.y = Y_b
                        self.poi.z = Z_b

                    self.box_size = (box[2] - box[0]) * (box[3] - box[1])

                    self.last_peduncle_center = self.peduncle_center

            # plot_3d_points(self.vis_pepper_list, self.vis_peduncle_list, self.image, "/root/catkin_ws/both"+str(self.image_count)+".png")
            self.vis_peduncle_list = []
            self.vis_pepper_list = []

            self.image_count+=1
        else: 
            # TODO probably don't need this anymore 
            self.detection_void_count += 1
            if self.detection_void_count > 100:
                self.peduncle_center.y = self.last_peduncle_center.y
                self.peduncle_center.x = 240
                self.pepper_center.y = self.last_pepper_center.y
                self.pepper_center.x = 240
                self.detection_void_count = 0
        self.poi_pub.publish(self.poi)
        self.pepper_center_pub.publish(self.pepper_center)
        self.peduncle_center_pub.publish(self.peduncle_center)


    def depth_callback(self, msg):
        try:
            # Convert ROS image message to OpenCV image
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        except cv_bridge.CvBridgeError as e:
            rospy.logerr("Error converting from depth image message: {}".format(e))
            
    def get_depth(self, x, y):
        top_x = max(0, x - self.depth_window)
        bottom_x = min(self.img_height, x + self.depth_window)
    
        left_y = max(0, y - self.depth_window)
        right_y = min(self.img_width, y + self.depth_window)

        depth_values = self.depth_image[top_x:bottom_x, left_y:right_y]
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
    
    def transform_to_base_frame(self, X, Y, Z):
        # Get transform
        try:
            transformation = self.tfBuffer.lookup_transform("link_base", "camera_color_optical_frame", rospy.Time(), rospy.Duration(0.1))
            
            # Get translation and rotation
            trans, quat = transformation.transform.translation, transformation.transform.rotation

            # Create homogeneous matrix
            homo_matrix = np.asarray(quaternion_matrix([quat.x, quat.y, quat.z, quat.w]))
            homo_matrix[:3, 3] = np.array([trans.x, trans.y, trans.z])

            # Transform to base frame
            point_camera_frame = np.array([X, Y, Z, 1])
            point_base_frame = np.matmul(homo_matrix, point_camera_frame) 

            return point_base_frame[0], point_base_frame[1], point_base_frame[2]
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("Error getting the transform")

    def state_callback(self, msg):
        self.state = msg.data

    def visualize_result(self, image, segment, poi=None, color=(100, 0, 125, 0.1)):
        mask_coords = (segment @ np.array([[self.img_width, 0], [0, self.img_height]])).astype(int)
        image = cv2.fillPoly(image, pts=[mask_coords], color=color)
        if poi is not None:
            image = cv2.circle(image, (int(poi[1]), int(poi[0])), 5, (0, 0, 255), -1)
        return image
    
    def make_marker(self, marker_type=8, frame_id='camera_color_optical_frame', r= 1, g=0, b=0, a=1, x=0.05, y=0.05):
        marker = Marker()
        marker.type = marker_type
        marker.header.frame_id = frame_id
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a
        marker.scale.x = x
        marker.scale.y = y

        return marker
    
    def random_color(self):
        r = random.randint(0, 255)
        g = random.randint(0, 255)
        b = random.randint(0, 255)
        a = random.random()  # returns a float between 0 and 1
        return (r, g, b, a)

if __name__ == '__main__':
    try:
        perception_node = PerceptionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
