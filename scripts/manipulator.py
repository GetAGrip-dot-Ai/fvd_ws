#!/usr/bin/env python3
import os
import yaml
import rospy
import rospkg
import math
from xarm.wrapper import XArmAPI
from xarm.version import __version__
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from scipy.spatial.transform import Rotation as R

"""
Move the xArm using API calls 
"""

class Manipulator:

    def __init__(self):
        """Manipulator class initialization"""
        # initialize values
        self.pregrasp_offset = None
        self.ee_length_offset = None
        self.init_pose = None
        self.orientation = None
        self.to_basket_points = None
        self.from_basket_points = None
        self.cartesian_speed = None
        self.traj_speed = None
        self.encore = False#rospy.get_param('/encore')
        self.ip = rospy.get_param('/xarm_robot_ip')
        arm_yaml = rospy.get_param('/arm_yaml')
        rospack = rospkg.RosPack()
        package_path = rospack.get_path("peter")
        self.parseYaml(arm_yaml, package_path)

        # initialize xArm
        self.arm = XArmAPI(self.ip)
        self.arm.motion_enable(enable=True)
        self.arm.connect()
        self.arm.set_mode(0)
        self.arm.set_state(0)

    def parseYaml(self, yaml_file, package_path):
        """parse the yaml to get parameters"""
        data = dict()
        with open(os.path.join(package_path + yaml_file), "r") as file:
            data = yaml.safe_load(file)
        self.pregrasp_offset = data["pregrasp_offset"]
        self.ee_length_offset = data["ee_offset"]
        self.init_pose = data["init_pose"]
        self.orientation = data["orientation"]
        self.to_basket_points = data["to_basket_points"]
        self.from_basket_points = data["from_basket_points"]
        self.cartesian_speed = int(data["cartesian_speed"])
        self.traj_speed = int(data["traj_speed"])


    def moveToInit(self):
        """move to initial position"""
        print("Moving to initial pose")
        self.arm.set_position(*self.init_pose, wait=True, speed=self.traj_speed)

    def cartesianMove(self,dist,axis): 
        """cartesian move along y"""
        dist = dist*1000  # convert m to mm
        current_pos = self.arm.get_position()[1]
        current_pos[axis] += dist # add dist to specified axis
        print("Executing cartesian move")
        self.arm.set_position(*current_pos, wait=True, speed=self.cartesian_speed)

    def moveToPregrasp(self,poi_pose):
        """move to pregrasp pose"""
        # get the position and orientation
        x = poi_pose.position.x
        y = poi_pose.position.y
        z = poi_pose.position.z

        # add x offsets
        x -= self.pregrasp_offset
        x -= self.ee_length_offset

        # move to new position
        self.arm.set_position(x * 1000 ,y * 1000 ,z * 1000 ,*self.orientation, wait=True, speed=self.traj_speed)

        # update roll angle
        if self.encore:
            # convert quat to rotation matrix
            quat = poi_pose.orientation
            r = R.from_quat([quat.x, quat.y, quat.z, quat.w])
            rot_mat = r.as_matrix()
            
            # take the y and z components
            y_comp = rot_mat[1,0]
            z_comp = rot_mat[2,0]

            # calculate the roll angle
            theta = math.atan2(y_comp, z_comp)
            self.orientation[1] += (math.pi - theta) * (180/math.pi)
            self.arm.set_position(x * 1000 ,y * 1000 ,z * 1000 ,*self.orientation, wait=True, speed=self.traj_speed)

    def moveToPoi(self):
        self.cartesianMove(self.pregrasp_offset,0) # move forward in x

    def orientParallel(self):
        current_pos = self.arm.get_position()[1]
        self.arm.set_position(*current_pos[0:3] ,*self.orientation, wait=True, speed=self.traj_speed)

    def moveToBasket(self):
        """move to basket pose for pepper drop off"""
        self.cartesianMove(-0.05,0) # move back 5 cm
        self.orientParallel() # straighten orientation if needed
        self.cartesianMove(-0.18,0) # move back 15 cm
        rospy.logwarn("Moving to basket pose")
        self.execute_traj(self.to_basket_points)
        rospy.logwarn("Done Traj to basket")

    def moveFromBasket(self):
        """move away from basket pose"""
        rospy.logwarn("Moving from basket pose")
        self.execute_traj(self.from_basket_points)
        rospy.logwarn("Done Traj from basket")

    def multiframe(self):
        """scan down the pepper plant"""
        print("Multiframe: scanning down the plant")
        self.cartesianMove(-0.2,2) # move up 20 cm in z

    def execute_traj(self, points):
        """execute an interpolated trajectory of waypoints"""
        self.arm.move_arc_lines(points, speed=self.traj_speed, times=1, wait=True)

    def disconnect(self):
        """disconnect from xarm"""
        self.arm.disconnect()

    def test(self):
        print("TESTING")
        # current_pose = self.arm.get_position()[1]
        # print(current_pose)
        # self.moveToBasket()
        # rospy.sleep(10)

        # self.moveFromBasket()
        # for traj in self.arm.get_trajectories():
        #     print(traj)

        self.moveToInit()

        # self.execute_traj(self.from_basket_points)
        print("done executing trajectory")

        return


if __name__ == '__main__':

    try:
        xarm = Manipulator()
        xarm.test()

        while not rospy.is_shutdown():
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        pass
