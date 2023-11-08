#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import tf2_ros
from std_msgs.msg import Int16

from filterpy.kalman import KalmanFilter
from scipy.linalg import norm
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

# if the cluster hasn't been seen in a while get rid of it
TIME_SINCE_LAST_OB_METRIC = 10

# nearest neighbor metric
NEAREST_NEIGHBOR_METRIC = 0.05 # [m]


class PepperFilterNode:
    
    def __init__(self):

        # initialize values
        rospy.init_node('pepper_filter_node', anonymous=True)
        
        # initialize list of peppers
        self.clusters = []
        
        # subs
        self.pepper_base_sub = rospy.Subscriber("/visualization_peduncle_marker_base", Marker, self.pep_callback, queue_size=1)
        
        # filtered poi publisher
        self.poi_pub = rospy.Publisher('/poi', Point, queue_size=1)
        
        self.poi_viz = rospy.Publisher("/poi_viz", Marker, queue_size=1)

    def pep_callback(self, data):
        
        potential_peps = data.points
        
        for pep in potential_peps:
            
            if pep.x == pep.y == pep.z == 0:
                break
            
            new_cluster = Cluster(pep.x, pep.y, pep.z, len(self.clusters) + 1)
            # see if pepper belongs to a pre-existing cluster
            
            if not self.clusters:
                self.clusters.append(new_cluster)
            
            dists = [(i, c.dist(new_cluster)) for i, c in enumerate(self.clusters)]
            # if it does  kalman filter it
            min_ind, min_dist = min(dists, key=lambda d: d[1])
            
            if min_dist < NEAREST_NEIGHBOR_METRIC:
                self.clusters[min_ind].filter(new_cluster.center)               
            # if it doesn't, make a new cluster
            else:
                self.clusters.append(new_cluster)
                
            # cleanup cluster based on our criteria
            for c in self.clusters:
                if c.cleanup(): 
                    self.clusters.remove(c)
            
            sorting_criteria = lambda p: p.dist_from_ee
            self.clusters = sorted(self.clusters, key=sorting_criteria)
            
            # if self.clusters:
                # self.visualize()
                
            # rospy.loginfo("===============================================")
            # for c in self.clusters: rospy.loginfo(c)


                
    def run(self):
        if self.clusters:
            
            poi = Point(
                self.clusters[0].x, 
                self.clusters[0].y, 
                self.clusters[0].z
                )
            
            poi_marker = self.make_marker(
                self.clusters[0].x, 
                self.clusters[0].y, 
                self.clusters[0].z
            )
            
            self.poi_pub.publish(poi)
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
        
    def visualize(self):
        
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
    
    def __init__(self, x, y, z, id):
        
        # define the cluster center
        self.x, self.y, self.z = x, y, z
        self.center = np.array([x, y, z])
        
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        
        self.calc_dist_from_ee()
                
        # time creation: used to reject infrequent observations
        self.birth = time.time()
        
        # total counts: used to reject low outcomes
        self.observations = 1
        self.last_ob_time = self.birth
        
        # make a filter for the cluster
        self.kf = KalmanFilter(dim_x=3, dim_z=3)
        self.kf.x = self.center
        self.kf.F = np.eye(3)  # State transition matrix set to identity
        self.kf.H = np.eye(3)  # Measurement matrix
        self.kf.P *= 1e-4  # Initial uncertainty
        
        # self.kf.R = 0.01 * np.eye(3)  # Measurement noise #TODO Tune
        
        # take a bunch of observations from live data
        # Note: This cov was taken from actual cluster covariance
        self.kf.R = np.array(
                    [[2.49044789e-06, 1.73915322e-07, 4.48913473e-07],
                     [1.73915322e-07, 2.42423576e-06, 1.17792215e-06],
                     [4.48913473e-07, 1.17792215e-06, 1.12009796e-05]])
        
        self.kf.Q = np.zeros((3, 3))  # Process noise set to zero
        
        self.id = id
        
    def __str__(self):
        return f"Cluster {self.id} | Center: {self.center} | " + \
        f"Time Alive: {self.time_since_birth()} | Time Since Ob: {self.time_since_last_ob()}"
        
        
    def dist(self, cluster):
        return norm(self.center - cluster.center)
    
    def calc_dist_from_ee(self):
        
        rospy.sleep(0.1)
        transformation = self.tfBuffer.lookup_transform("link_base", "camera_color_optical_frame", 
                                                        rospy.Time.now(), rospy.Duration(0.5))
        ee_loc = np.array(
            [transformation.transform.translation.x, 
             transformation.transform.translation.y, 
             transformation.transform.translation.z])
        
        # ee_loc = np.array(
        #     [0, 
        #      0, 
        #      0])
        
        self.dist_from_ee = norm(self.center - ee_loc)
    
    def time_since_birth(self):
        return time.time() - self.birth
    
    def time_since_last_ob(self):
        return time.time() - self.last_ob_time
    
    def filter(self, measurement):
        
        self.observations += 1
        self.last_ob_time = time.time()
        
        self.kf.predict()
        self.kf.update(measurement)
        
        self.center = self.kf.x
    
    def cleanup(self):
        
        if self.time_since_birth() > TIME_SINCE_BIRTH_METRIC and self.observations < OBSERVATIONS_METRIC:
            return True 
        
        if self.time_since_last_ob() > TIME_SINCE_LAST_OB_METRIC:
            return True
        
        
    
        
        

if __name__ == '__main__':

    try:
        pepper_filter_node = PepperFilterNode()
        while not rospy.is_shutdown():
            pepper_filter_node.run()
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        pass
