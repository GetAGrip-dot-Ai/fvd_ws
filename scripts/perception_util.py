#!/usr/bin/env python3

import rospy
import rospkg
import cv_bridge

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Int16, Bool
from geometry_msgs.msg import Point, Pose
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion

import message_filters

import cv2
import torch
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image as PILImage

import message_filters

import tf2_ros
from tf.transformations import quaternion_matrix
from scipy.spatial.transform import Rotation as R

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


    
def make_marker(marker_type=8, frame_id='camera_color_optical_frame', r= 1, g=0, b=0, a=1, x=0.05, y=0.05):
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

def random_color():
    r = random.randint(0, 255)
    g = random.randint(0, 255)
    b = random.randint(0, 255)
    a = random.random()  # returns a float between 0 and 1
    return (r, g, b, a)

def get_pose_object( position, orientation):
    pose = Pose()
    pose.position = Point(x=position[0], y=position[1], z=position[2])
    orientation = np.array(
        [orientation[0], orientation[1], orientation[2]])
    orientation = orientation/np.linalg.norm(orientation)

    if orientation[0] == 0 and orientation[1] == 0 and orientation[2] == 1:
        cross_vector = np.array([0, 1, 0])
    else:
        cross_vector = np.array([0, 0, 1])

    cross1 = np.cross(orientation, cross_vector)
    cross2 = np.cross(orientation, cross1)
    rotation = np.array([orientation, cross1, cross2]).T

    r = R.from_matrix(rotation)
    quat = r.as_quat()
    pose.orientation = Quaternion(
        x=quat[0], y=quat[1], z=quat[2], w=quat[3])

    return pose

def transform_to_base_frame(transformation, X, Y, Z):

    # Get translation and rotation
    trans, quat = transformation.transform.translation, transformation.transform.rotation

    # Create homogeneous matrix
    homo_matrix = np.asarray(quaternion_matrix(
        [quat.x, quat.y, quat.z, quat.w]))
    homo_matrix[:3, 3] = np.array([trans.x, trans.y, trans.z])

    # Transform to base frame
    point_camera_frame = np.array([X, Y, Z, 1])
    point_base_frame = np.matmul(homo_matrix, point_camera_frame)

    return point_base_frame[0], point_base_frame[1], point_base_frame[2]

# Functions for smoothing depth image
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
