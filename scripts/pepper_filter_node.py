#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray
import tf2_ros
from std_msgs.msg import Int16

from filterpy.kalman import KalmanFilter
from scipy.linalg import norm
from scipy.spatial.transform import Rotation as R
import time
import numpy as np
import matplotlib.pyplot as plt

import rospkg
import os
rospack = rospkg.RosPack()
package_name = 'peter'
package_path = rospack.get_path(package_name)

# if the clusters has been around for a while and 
# we have too few observations get rid of it

TIME_SINCE_BIRTH_METRIC = 100 # [s]
OBSERVATIONS_METRIC = 20 # unitless

# OBSERVATIONS THRESHOLD
# the amount of detections you need to publish a POI
OBSERVATIONS_THRESHOLD = 1

# if the cluster hasn't been seen in a while get rid of it
TIME_SINCE_LAST_OB_METRIC = 10

# nearest neighbtime_since_last_obor metric
NEAREST_NEIGHBOR_METRIC = 0.03 # [m]

# explicit rejection of peppers happens above this value
RADIAL_REJECTION = 100.0 # [cm]

RADIAL_REJECTION /= 100.0 # [m]


class PepperFilterNode:
    
    def __init__(self):

        # initialize values
        rospy.init_node('pepper_filter_node', anonymous=True)
        
        # initialize list of peppers
        self.clusters = []
        
        # subs
        self.pepper_base_sub = rospy.Subscriber("/visualization_peduncle_poses_base", PoseArray, self.pep_callback, queue_size=1)
        
        # filtered poi publisher
        self.poi_pub = rospy.Publisher('/poi', Pose, queue_size=1)
        self.poi_viz = rospy.Publisher("/poi_viz", Marker, queue_size=1)
        # not including the chosen poi
        self.filtered_pois = rospy.Publisher("/filtered_pois", PoseArray, queue_size=1)
        self.filtered_pois_array = PoseArray()
        self.filtered_pois_array.header.frame_id = "link_base"

    def pep_callback(self, data):
        
        potential_peps = data.poses

        for pep in potential_peps:

            if pep.position.x == pep.position.y == pep.position.z == 0:
                break
            
            new_cluster = Cluster(pep.position, pep.orientation, len(self.clusters) + 1)
            # see if pepper belongs to a pre-existing cluster
            
            # reject clusters too far from base frame
            if norm(new_cluster.center) > RADIAL_REJECTION: 
                break
            
            if self.clusters:
                
                dists = [(i, new_cluster.dist(c)) for i, c in enumerate(self.clusters)]

                # if it does  kalman filter it
                min_ind, min_dist = min(dists, key=lambda d: d[1])

                # rospy.logwarn(len(self.clusters) ) 
                
                if min_dist < NEAREST_NEIGHBOR_METRIC:
                    self.clusters[min_ind].filter(new_cluster.center, new_cluster.quat)               
                # if it doesn't, make a new cluster
                else:
                    self.clusters.append(new_cluster)
                
                sorting_criteria = lambda p: p.dist_from_ee
                self.clusters = sorted(self.clusters, key=sorting_criteria)
            else:
                self.clusters.append(new_cluster)
        

                
    def run(self):
        
        self.filtered_pois_array.poses = []
        self.filtered_pois_array.header.stamp = rospy.Time.now()
        self.filtered_pois.publish(self.filtered_pois_array)

        # cleanup cluster based on our criteria
        delete_indices = [i for i, c in enumerate(self.clusters) if c.cleanup()]
        for i in sorted(delete_indices, reverse=True):
            del self.clusters[i]

        poi_marker = Marker()
        poi_marker.type = 0
        poi_marker.header.frame_id = "link_base"
        poi_marker.color.r = 1
        poi_marker.color.g = 1
        poi_marker.color.b = 0
        poi_marker.color.a = 1
        poi_marker.scale.x = 0.08
        poi_marker.scale.y = 0.005
        poi_marker.scale.z = 0.005

        if self.clusters:
            
            poses_list = []
            for i in range(1, len(self.clusters)):  
                poi_pose = self.clusters[i].get_pose_object()
                poses_list.append(poi_pose)
            
            # poi of interest
            poi_pose = self.clusters[0].get_pose_object()
            poi_marker.pose = poi_pose
            
            if self.clusters[0].observations > OBSERVATIONS_THRESHOLD:
                self.poi_pub.publish(poi_pose)
            else:
                poi_marker.color.a = 0.3
                poi_marker.color.g = 0.8
                poi_marker.color.r = 0.8

            self.poi_viz.publish(poi_marker)

            
            self.filtered_pois_array.poses = poses_list
            self.filtered_pois_array.header.stamp = rospy.Time.now()
            self.filtered_pois.publish(self.filtered_pois_array)
        else: 
            poi_marker.pose = Pose()
            self.poi_viz.publish(poi_marker)

    
            
    def make_marker(self, x, y, z, marker_type=2, frame_id='link_base', 
                    r= 1, g=0, b=0, a=1, scale=0.025):
        marker = Marker()
        marker.type = marker_type
        marker.header.frame_id = frame_id
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale

        return marker
        
    def static_visualize(self):
        
        x = [p.x for p in self.clusters]
        y = [p.y for p in self.clusters]
        z = [p.z for p in self.clusters]
        
        fig = plt.figure()  # Adjust the figure size as needed

        # Add subplot for the image
        ax1 = fig.add_subplot(projection='3d')

        # Unpack the first set of points into x1, y1, and z1 lists
        ax1.scatter(x[1:], y[1:], z[1:], color='grey', label='Others', alpha=0.2, s = 5)
        ax1.scatter(x[0], y[0], z[0], color='green', label='Chosen Pepper', s = 8)

        # Unpack the second set of points into x2, y2, and z2 lists
        ax1.set_xlabel('depth (m)')
        ax1.set_ylabel('x (m)')
        ax1.set_zlabel('y (m)')
        
        lim = max(max(x), max(y), max(z))
        ax1.set_xlim(0, lim)
        ax1.set_ylim(0, lim)
        ax1.set_zlim(0, lim)

        # Add a legend to differentiate the two point sets
        # Adjust layout
        plt.tight_layout()
        
        plt.savefig(os.path.join(package_path + "/kalman_filter.png"))

        # Close the plot to free up memory
        plt.close(fig)
    
class Cluster:
    
    def __init__(self, position, orientation, id):
        
        # define the cluster center
        self.center = np.array([position.x, position.y, position.z])
        self.quat = np.array([orientation.x, orientation.y, orientation.z, orientation.w])
        self.vec = self.quat_to_vec(self.quat)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        
        self.calc_dist_from_ee()
                
        # time creation: used to reject infrequent observations
        self.birth = time.time()
        
        # total counts: used to reject low outcomes
        self.observations = 1
        self.last_ob_time = self.birth
        
        # make a filter for the cluster
        self.kf = KalmanFilter(dim_x=7, dim_z=7)
        self.kf.x = np.reshape(np.concatenate([self.center, self.quat]), newshape=(-1, 1))
        self.kf.F = np.eye(7)  # State transition matrix set to identity
        self.kf.H = np.eye(7)  # Measurement matrix
        self.kf.P *= 1e-4  # Initial uncertainty
        
        # take a bunch of observations from live data
        # Note: This cov was taken from actual cluster covariance
        
        cov = np.zeros(shape=(7, 7))
        cov[:3, :3] = np.array(
                    [[2.49044789e-06, 1.73915322e-07, 4.48913473e-07],
                     [1.73915322e-07, 2.42423576e-06, 1.17792215e-06],
                     [4.48913473e-07, 1.17792215e-06, 1.12009796e-05]])
        cov[3:, 3:] = np.eye(4)
        
        self.kf.R = cov
        
        self.kf.Q = np.zeros((7, 7))  # Process noise set to zero
        
        self.id = id
        
    def __str__(self):
        return f"Cluster {self.id} | Center: {self.center} | " + \
        f"Time Alive: {self.time_since_birth()} | Time Since Ob: {self.time_since_last_ob()}"
        
        
    def dist(self, new_cluster):
        n = norm(self.center - new_cluster.center)

        return n
    
    def quat_to_vec(self, quat):
        
        quat_ = R.from_quat(quat)
        rot = quat_.as_matrix()
        
        # project take y component 
        return np.array([0, rot[0, 1], rot[0, 2]])
    
    def vec_to_quat(self, vec):
        
        cross_vector = np.zeros(3)
        if np.array_equal(vec, np.array([0, 0, 1])):
            cross_vector = np.array([0, 1, 0])  
        else: 
            cross_vector = np.array([0, 0, 1])


        cross1 = np.cross(np.squeeze(vec), cross_vector)
        cross2 = np.cross(np.squeeze(vec), cross1)

        rotation = np.vstack([np.squeeze(vec), cross1, cross2]).T

        r = R.from_matrix(rotation)
        quat = r.as_quat()

        return quat
        
    
    def calc_dist_from_ee(self):
        
        rospy.sleep(0.1)
        transformation = self.tfBuffer.lookup_transform("link_base", "camera_color_optical_frame", 
                                                        rospy.Time.now(), rospy.Duration(0.5))
        ee_loc = np.array(
            [transformation.transform.translation.x, 
             transformation.transform.translation.y, 
             transformation.transform.translation.z])
        
        self.dist_from_ee = norm(self.center - ee_loc)
    
    def time_since_birth(self):
        return time.time() - self.birth
    
    def time_since_last_ob(self):
        return time.time() - self.last_ob_time
    
    def filter(self, center, orient):

        measurement = np.concatenate([center, orient])
        
        self.observations += 1
        self.last_ob_time = time.time()
        
        self.kf.predict()
        self.kf.update(measurement)
        
        self.center = np.squeeze(self.kf.x[:3])
        self.quat = np.squeeze(self.kf.x[3:])
        # self.quat = self.vec_to_quat(self.vec)

    
    def cleanup(self):
        
        if self.time_since_birth() > TIME_SINCE_BIRTH_METRIC and self.observations < OBSERVATIONS_METRIC:
            rospy.logwarn("removing cluster")
            return True 
        
        if self.time_since_last_ob() > TIME_SINCE_LAST_OB_METRIC:
            rospy.logwarn("removing cluster")
            return True
        
        return False
        
        
    def get_pose_object(self):
        
        pose = Pose()
        pose.position = Point(*self.center)
        
        pose.orientation = Quaternion(*self.quat)
        
        return pose
        
        

if __name__ == '__main__':

    try:
        pepper_filter_node = PepperFilterNode()
        while not rospy.is_shutdown():
            pepper_filter_node.run()
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        pass
