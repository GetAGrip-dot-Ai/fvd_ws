#!/usr/bin/env python3

import os
import rospkg
import rospy
from sensor_msgs.msg import Joy
from visualization_msgs.msg import Marker

PEDUNCLE_FILE = "/peduncles_10262023_3.csv"
PEPPER_FILE = "/peppers_10262023_3.csv"

class Topic2CSV:
    
    def __init__(self) -> None:
        
        rospy.init_node('Topic_Logger', anonymous=True)
        
        rospack = rospkg.RosPack()
        package_name = 'fvd_ws'
        self.package_path = rospack.get_path(package_name)
        
        self.joystick_callback = rospy.Subscriber('/joy_relay', Joy, self.record_callback)

        self.peduncle_base_sub = rospy.Subscriber("/visualization_peduncle_marker_base", Marker, self.ped_callback, queue_size=1)
        self.pepper_base_sub = rospy.Subscriber("/visualization_pepper_marker_base", Marker, self.pep_callback, queue_size=1)
        
    def record_callback(self, data):    
        
        if data.buttons[5]:      
            self.record_peduncles()
            self.record_peppers()
            rospy.loginfo("Logging...")
        
    def record_peduncles(self):
        with open(os.path.join(self.package_path + PEDUNCLE_FILE), "+a") as file:
            for pose in self.peppers: 
                file.write(f"{pose.x}, {pose.y}, {pose.z}, {len(self.peppers)}\n")
            file.close()
    
    def record_peppers(self):
        with open(os.path.join(self.package_path + PEPPER_FILE), "+a") as file:
            for pose in self.peduncles: 
                file.write(f"{pose.x}, {pose.y}, {pose.z}, {len(self.peduncles)}\n")
            file.close()
    
    def pep_callback(self, data):
        self.peppers = data.points
    
    def ped_callback(self, data):
        self.peduncles = data.points

if __name__ == '__main__':

    try:
        topic_logger = Topic2CSV()
        while not rospy.is_shutdown():
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        pass