#!/usr/bin/env python3

import os
import rospy
import numpy as np
import rospkg 
import tf2_ros
import geometry_msgs.msg
import yaml
# import tf_conversions
from scipy.spatial.transform import Rotation


"""
broadcase yaml
"""

class TfBroadcaster:

    def __init__(self):
        # initialize values
        rospy.init_node("tf_handler_node")
        tf_yaml = rospy.get_param('/tf_yaml')
        
        rospack = rospkg.RosPack()
        # get the file path for rospy_tutorials
        package_path = rospack.get_path("peter")
        
        self.parseYaml(tf_yaml, package_path)

        
    def broadcast(self):
        rot = self.H[:3, :3]
        # quaternion1 = tf_conversions.transformations.quaternion_from_matrix(self.H)
        r = Rotation.from_matrix(rot) 
        quaternion = r.as_quat()
        trans = self.H[:, 3]
        
        br = tf2_ros.StaticTransformBroadcaster()
        transform = geometry_msgs.msg.TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "link_eef"
        transform.child_frame_id = "camera_link"
        
        transform.transform.translation.x = trans[0]
        transform.transform.translation.y = trans[1]
        transform.transform.translation.z = trans[2]

        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        
        br.sendTransform(transform)
        

    def parseYaml(self, yaml_file, package_path):
        """parse the yaml to reveal all the relevant transforms"""
    
        # parse data 
        data = dict()
        with open(os.path.join(package_path + yaml_file), "r") as file:
            data = yaml.safe_load(file)
        
        # convert the 2d matrices to a list of transforms
        transforms = list()
        for tf_mat in data.values():
            transforms.append(np.array(tf_mat))
                # apply transforms on to the right of each other  
        HTM = np.eye(4)
        for tf in transforms:
            HTM = HTM @ tf   
            
        # rospy.logwarn(HTM)
        self.H = HTM      


if __name__ == '__main__':

    try:
        tfs = TfBroadcaster()
        while not rospy.is_shutdown():
            tfs.broadcast()
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        pass