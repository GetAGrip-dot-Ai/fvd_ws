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
from perception_util import make_marker

"""
Listen to state message. 
Depending on state, call the appropriate callback function and run the corresponding planner. 
"""

class PlannerNode:
    def __init__(self):

        # initialize values
        rospy.init_node('planner_node', anonymous=True)
        self.publish_value = -1
        self.state = 0
        self.joy_state = Joy()
        self.visual_servoing_state = Twist() 
        self.poi = None
        self.poi_marker = make_marker(marker_type=8, frame_id='link_base', r=0, g=1, b=1, a=1, x=0.04, y=0.04)

        # subscribers
        self.state_sub = rospy.Subscriber('/state', Int16, self.state_callback, queue_size=1) # state message
        self.joystick_sub = rospy.Subscriber('/joy', Joy, self.joystick_callback, queue_size=1) # joystick message        
        self.poi_sub = rospy.Subscriber('/poi', Pose, self.poi_callback, queue_size=1) # poi pose

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
        # rospy.wait_for_service('/gripper_service')
        if command == "open":
            try:
                rospy.wait_for_service('/gripper_service')
                Pegasus_action = rospy.ServiceProxy('/gripper_service',Pegasus)
                Pegasus_action(1)
                # rospy.logwarn("fake end effector open")
                # rospy.sleep(3)
            except rospy.ServiceException as e:
                rospy.Logwarn("Service call failed: %s"%e)

        elif command == "harvest":
            try:
                rospy.wait_for_service('/gripper_service')
                Pegasus_action = rospy.ServiceProxy('/gripper_service',Pegasus)
                Pegasus_action(0)
                # rospy.logwarn("fake end effector harvest")
                # rospy.sleep(3)
            except rospy.ServiceException as e:
                rospy.Logwarn("Service call failed: %s"%e)
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
            self.publish_value = self.state

        # move to init
        elif self.state == 2:
            xarm = Manipulator()
            xarm.moveToInit()
            rospy.sleep(1)
            self.publish_value = self.state

        # ----- AUTONOMOUS SEQUENCE -----
        # move to init position and open end-effector
        elif self.state == 3:
            try:
                xarm = Manipulator()
                xarm.moveToInit()
                rospy.sleep(.1)
                xarm.disconnect()
                rospy.sleep(.1)
                rospy.logwarn("Plan Execution: Initalization Started")
                self.send_to_ee("open")
                rospy.logwarn("Plan Execution: Initalization Complete")
                self.publish_value = 3
            except:
                rospy.logwarn("ERROR: UNABLE TO INITIALIZE AUTONOMOUS PROCEDURE")
                self.state_pub.publish(10)
                self.publish_value = 10
    
        # multiframe
        elif self.state == 4:
            try:
                xarm = Manipulator()
                xarm.multiframe()
                rospy.sleep(.1)
                xarm.disconnect()
                rospy.sleep(.1)
                rospy.logwarn("Plan Execution: Multiframe Complete")
                self.publish_value = 4
            except:
                rospy.logwarn("ERROR: UNABLE TO MULTIFRAME")
                self.state_pub.publish(10)
                self.publish_value = 10

        # move to pre-grasp
        elif self.state == 5:
            try:
                xarm = Manipulator()
                if self.poi:
                    rospy.logwarn(f"POI!!!!!! >>  {self.poi}")
                    xarm.moveToPregrasp(self.poi)
                else:
                    rospy.logwarn("NO POI DETCTED YET!!") #TODO delete?
                rospy.sleep(.1)
                xarm.disconnect()
                rospy.sleep(.1)
                self.publish_value = 5
            except Exception as e:
                rospy.logwarn(f"ERROR: UNABLE TO MOVE TO PREGRASP POSITION {e}")
                self.state_pub.publish(10)
                self.publish_value = 10

        # move to poi
        elif self.state == 6:
            try:
                xarm = Manipulator()
                xarm.moveToPoi()
                rospy.sleep(.1)
                xarm.disconnect()
                rospy.sleep(.1)
                rospy.logwarn("Plan Execution: Move to POI Complete")
                self.publish_value = 6
            except:
                rospy.logwarn("ERROR: UNABLE TO MOVE TO POI")
                self.state_pub.publish(10)
                self.publish_value = 10

        # harvest pepper
        elif self.state == 7:
            self.send_to_ee("harvest")
            self.publish_value = 7
            
        # move to basket and drop then go back to init
        elif self.state == 8:
            try:
                self.poi = None
                rospy.sleep(.1)
                xarm = Manipulator()
                xarm.moveToBasket()
                rospy.logwarn("Moved to basket")
                rospy.sleep(13)
                # while(xarm.arm.get_is_moving()):
                    # rospy.sleep(.1) 
                xarm.disconnect()
                rospy.logwarn("Plan Execution: Move to Basket Complete")
                self.send_to_ee("open")
                rospy.sleep(2)
                xarm = Manipulator()
                xarm.moveFromBasket()
                rospy.logwarn("Moved from basket")
                rospy.sleep(1)
                rospy.logwarn("Plan Execution: Move from Basket Complete")
                xarm.disconnect()
                rospy.sleep(.1)
                self.publish_value = 8
            except:
                rospy.logwarn("ERROR: UNABLE TO MOVE TO BASKET")
                self.state_pub.publish(10)
                self.publish_value = 10

        # move to basket and drop then go back to init
        elif self.state == 8:
            try:
                self.poi = None
                rospy.sleep(.1)
                xarm = Manipulator()
                xarm.moveToBasket()
                rospy.logwarn("Moved to basket")
                rospy.sleep(1)
                xarm.disconnect()
                rospy.sleep(.1)
                rospy.logwarn("Plan Execution: Move to Basket Complete")
                self.send_to_ee("open")
                rospy.sleep(.1)
                xarm = Manipulator()
                xarm.moveFromBasket()
                rospy.logwarn("Moved from basket")
                rospy.sleep(1)
                rospy.logwarn("Plan Execution: Move from Basket Complete")
                xarm.disconnect()
                rospy.sleep(.1)
                self.publish_value = 8
            except:
                rospy.logwarn("ERROR: UNABLE TO MOVE TO BASKET")
                self.state_pub.publish(10)
                self.publish_value = 10

        else:
            rospy.logwarn("ERROR: UNRECOGNIZED STATE IN PLANNER NODE")
            self.state_pub.publish(10)

        # dispay pregrasp visualization
        if self.poi:
            self.visualizePregrasp(self.poi.position.x, self.poi.position.y, self.poi.position.z)
                      
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
            planner_node.planner_state_pub.publish(planner_node.publish_value)
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        pass
