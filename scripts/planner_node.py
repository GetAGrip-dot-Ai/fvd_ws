#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16, Bool
from geometry_msgs.msg import Twist, Point, Pose
from xarm_msgs.msg import RobotMsg
from manipulator import Manipulator
from ag_gripper_driver.srv import Pegasus, PegasusResponse
from visualization_msgs.msg import Marker


"""
Listen to state message. 
Depending on state, call the appropriate callback function and run the corresponding planner. 
"""

class PlannerNode:
    def __init__(self):

        # initialize values
        rospy.init_node('planner_node', anonymous=True)
        self.state = 0
        self.joy_state = Joy()
        self.visual_servoing_state = Twist() 
        self.poi = None
        self.poi_marker = self.make_marker(marker_type=8, frame_id='link_base', r=0, g=1, b=1, a=1, x=0.04, y=0.04)

        # subscribers
        self.state_sub = rospy.Subscriber('/state', Int16, self.state_callback, queue_size=1) # state message
        self.joystick_sub = rospy.Subscriber('/joy', Joy, self.joystick_callback, queue_size=1) # joystick message        
        self.pose_sub = rospy.Subscriber('/poi', Pose, self.poi_callback, queue_size=1) # poi pose

        # publishers
        self.joy_pub = rospy.Publisher('/joy_relay', Joy, queue_size=1) # joystick commands pub
        self.planner_state_pub = rospy.Publisher('/planner_state', Int16, queue_size=1) # planner state pub
        self.state_pub = rospy.Publisher('/state', Int16, queue_size=1) # state pub
        self.poi_from_arm_pub = rospy.Publisher('/poi_from_arm', Marker, queue_size=1) # poi pub


    def state_callback(self, data):
        """Callback for state message"""
        self.state = data.data
    
    def robot_state_callback(self, data):
        """Callback for robot state message"""
        self.ee_pose = data.pose

    def joystick_callback(self, data):
        """Callback for joystick message"""
        self.joy_state = data

    def poi_callback(self, data):
        """Callback for POI message"""
        self.poi = data

    def send_to_ee(self, command):
        """Send commands to open, harvest, or close for end-effector"""

        rospy.wait_for_service('/gripper_service')
        if command == "open":
            try:
                Pegasus_action = rospy.ServiceProxy('/gripper_service',Pegasus)
                Pegasus_action(1)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

        elif command == "harvest":
            try:
                Pegasus_action = rospy.ServiceProxy('/gripper_service',Pegasus)
                Pegasus_action(0)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
        return
        

    def run(self):
        """check what state the robot is in and run the corresponding actions"""

        # ----- MANUAL STATES -----
        # idle / amiga teleop
        if self.state == 0:
            pass

        # xarm teleoperation
        elif self.state == 1:
            self.joy_pub.publish(self.joy_state) # publish joystick commands to arm and ee

        # move to init
        elif self.state == 2:
            xarm = Manipulator()
            xarm.moveToInit()
            rospy.sleep(1)

        # ----- AUTONOMOUS SEQUENCE -----
        # move to init position and open end-effector
        elif self.state == 3:
            try:
                xarm = Manipulator()
                xarm.moveToInit()
                rospy.sleep(.1)
                xarm.disconnect()
                rospy.sleep(.1)
                self.send_to_ee("open")
                rospy.logwarn("Plan Execution: Initalization Complete")
                self.planner_state_pub.publish(3)
            except:
                rospy.logwarn("ERROR: UNABLE TO INITIALIZE AUTONOMOUS PROCEDURE")
                self.state_pub.publish(10)
    
        # multiframe
        elif self.state == 4:
            try:
                xarm = Manipulator()
                xarm.multiframe()
                rospy.sleep(.1)
                xarm.disconnect()
                rospy.sleep(.1)
                rospy.logwarn("Plan Execution: Multiframe Complete")
                self.planner_state_pub.publish(self.state)
            except:
                rospy.logwarn("ERROR: UNABLE TO MULTIFRAME")
                self.state_pub.publish(10)
            
        # move to pre-grasp
        elif self.state == 5:
            try:
                xarm = Manipulator()
                if self.poi:
                    rospy.logwarn(f"POI!!!!!! >>  {self.poi}")
                    xarm.moveToPregrasp(self.poi)
                else:
                    rospy.logwarn("NO POI DETCTED YET!!")
                rospy.sleep(.1)
                xarm.disconnect()
                rospy.sleep(.1)
                self.planner_state_pub.publish(self.state)
            except Exception as e:
                rospy.logwarn(f"ERROR: UNABLE TO MOVE TO PREGRASP POSITION {e}")
                self.state_pub.publish(10)

        # move to poi
        elif self.state == 6:
            try:
                xarm = Manipulator()
                xarm.moveToPoi()
                rospy.sleep(.1)
                xarm.disconnect()
                rospy.sleep(.1)
                rospy.logwarn("Plan Execution: Move to POI Complete")
                self.planner_state_pub.publish(self.state)
            except:
                rospy.logwarn("ERROR: UNABLE TO MOVE TO POI")
                self.state_pub.publish(10)

        # harvest pepper
        elif self.state == 7:
            self.send_to_ee("harvest")
            self.planner_state_pub.publish(self.state)

        # move to basket and drop then go back to init
        elif self.state == 8:
            try:
                rospy.sleep(.1)
                xarm = Manipulator()
                xarm.moveToBasket()
                rospy.sleep(2)
                xarm.disconnect()
                rospy.sleep(.1)
                rospy.logwarn("Plan Execution: Move to Basket Complete")
                self.send_to_ee("open")
                rospy.sleep(.1)
                xarm = Manipulator()
                xarm.moveFromBasket()
                rospy.sleep(2)
                rospy.logwarn("Plan Execution: Move from Basket Complete")
                xarm.disconnect()
                rospy.sleep(.1)
                self.planner_state_pub.publish(self.state)
            except:
                rospy.logwarn("ERROR: UNABLE TO MOVE TO BASKET")
                self.state_pub.publish(10)

        else:
            rospy.logwarn("ERROR: UNRECOGNIZED STATE IN PLANNER NODE")
            self.state_pub.publish(10)
            
        # dispay pregrasp visualization
        if self.poi:
            self.visualizePregrasp(self.poi.position.x, self.poi.position.y, self.poi.position.z)


    def make_marker(self, marker_type=8, frame_id='camera_color_optical_frame', r= 1, g=0, b=0, a=1, x=0.05, y=0.05):
        """make a marker for RViz"""
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
    
            
    def visualizePregrasp(self, x, y, z):
        """publish pregrasp marker"""
        self.poi_marker.points = []
        self.poi_marker.points.append(Point(x - 0.30, y, z))
        self.poi_marker.header.stamp = rospy.Time.now()
        self.poi_from_arm_pub.publish(self.poi_marker)
        


if __name__ == '__main__':

    try:
        planner_node = PlannerNode()
        while not rospy.is_shutdown():
            planner_node.run()
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        pass
