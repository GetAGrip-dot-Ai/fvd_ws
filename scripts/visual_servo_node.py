#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, Twist


class VisualServoingNode:
    def __init__(self):

        rospy.init_node('visual_servoing_node', anonymous=True)
        self.rate = rospy.Rate(30)

        # RealSense image dimensions
        self.image_width = 640.0
        self.image_height = 480.0

        # Set target x and y pixels (numpy coordinate system)
        self.target_x = self.image_width/2
        self.target_y = self.image_height/2

        # Set control gains
        self.k_x = 1
        self.k_y = 1
        self.k_z = 0.5

        # pepper center and peduncle center
        self.pepper_center = [0, 0, 0]
        self.peduncle_center = [0, 0, 0]

        # Subscribe to perception poi message
        self.peduncle_center_sub = rospy.Subscriber(
            '/peduncle_center', Point, self.peduncle_center_callback)

        self.pepper_center_sub = rospy.Subscriber(
            '/pepper_center', Point, self.pepper_center_callback)

        # Publish a command velocity
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def pepper_center_callback(self, data):
        self.pepper_center = [data.x, data.y, data.z]

        # calculate the orientation

    def peduncle_center_callback(self, data):
        self.actual_x = data.x  # In pixels
        self.actual_y = data.y  # In pixels
        self.actual_z = data.z  # In meters

        self.peduncle_center = [self.actual_x, self.actual_y, self.actual_z]

        # Calculate error between poi and image center
        target_error_x = self.target_x - self.actual_x
        target_error_y = self.target_y - self.actual_y
        print(target_error_x, target_error_y, self.actual_z)

        # Normalize errors
        target_error_x = target_error_x/self.image_width
        target_error_y = target_error_y/self.image_height

        # Visual servoing control law (velocities set according to RealSense frame)
        vel_cmd = Twist()
        vel_cmd.linear.x = self.k_x * target_error_y
        vel_cmd.linear.y = self.k_y * target_error_x
        vel_cmd.linear.z = self.k_z

        # Publish velocity
        self.cmd_vel_pub.publish(vel_cmd)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()


if __name__ == '__main__':
    try:
        visual_servoing_node = VisualServoingNode()
        visual_servoing_node.run()
    except rospy.ROSInterruptException:
        pass
