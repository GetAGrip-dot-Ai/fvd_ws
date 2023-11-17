#!/usr/bin/env python3

### ROS node for state machine
import rospy
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from std_msgs.msg import Int16, Bool

from xarm_msgs.srv import SetInt16, SetInt16Response
from numpy.linalg import norm

"""
---- Possible states -----
--Manual States--
idle: 0
teleop: 1
move to init: 2

--Autonomous Sequence--
move to init: 3
multiframe: 4
move to pregrasp: 5
move to poi: 6
harvest: 7
move to basket: 8
release & reset: 9
ERROR: 10

"""

class StateMachineNode:
    """ROS node for state machine"""
    def __init__(self):
        rospy.init_node('state_machine_node', anonymous=True)

        # --------- JOYSTICK -------------
        self.joy_state = Joy()
        self.joystick_callback = rospy.Subscriber('/joy', Joy, self.joystick_callback, queue_size=1)

        # --------- STATE PUBLISHER INITIALIZATION -------------
        # create publisher that contiously publishes state at specified rate
        self.state_pub = rospy.Publisher('/state', Int16, queue_size=1)
        self.state = 0 # initial state is 0 (idle / amiga teleoperation)
        
        # --------- ROBOT SUBSYSTEM SUBSCRIBERS -------------
        self.manipulator_state_sub = rospy.Subscriber('/xarm/jointState', JointState, self.manipulator_state_callback, queue_size=1)
        self.planner_state_sub = rospy.Subscriber('/planner_state', Int16, self.planner_state_callback, queue_size=1)
        self.poi_sub = rospy.Subscriber('/poi', Pose, self.detection_callback, queue_size=1)


        # --------- ROBOT SUBSYSTEM VARIABLES ---------------
        self.manipulator_moving = None
        self.zero_vel_threshold = 1e-3
        self.plan_executed = None
        self.detection = None


    # --------- PLANNER STATE CALLBACK -------------
    def planner_state_callback(self, data):
        """Callback for planner state message"""
        self.plan_executed = data.data


    # --------- MANIPULATOR STATE CALLBACK -------------
    def manipulator_state_callback(self, joint):
        """Callback for manipulator state message"""

        if norm(joint.velocity) < self.zero_vel_threshold:
            rospy.loginfo(f"STATE MACHINE: Manipulator is still with norm {norm(joint.velocity)}")
            self.manipulator_moving = False
        else:
            rospy.loginfo("STATE MACHINE: Manipulator is moving")
            self.manipulator_moving = True


    # --------- JOYSTICK CALLBACK -------------
    def joystick_callback(self, data):
        """Callback for joystick message"""
        # update joy state
        self.joy_state = data.buttons
        
        '''
        Assumptions:
        - Only one button is pressed at a time
        '''
        # teleop mode
        if self.joy_state[6] and self.state != 1:
            self.state = 1
            rospy.loginfo("Enter Teleop Mode")

        # xbox button -> autonomous sequence
        elif self.joy_state[8] and self.state != 3: 
            self.state = 3
            rospy.loginfo("Enter Autonomous Sequence")
            
        # Y -> enter idle mode
        elif self.joy_state[3] and self.state != 0:
            self.state = 0
            rospy.loginfo("Enter Idle Mode")

        # TODO: add a button for calling move to init - manual (state 2)


    # --------- POI DETECTION CALLBACK -------------
    def detection_callback(self,msg):
        """Determine if there has been a detection in this cycle of states"""
        if self.state == 8: # reset self.detection after moving to basket , TODO spin rate???
            self.detection = None
        else:
            self.detection = 1
            # rospy.logwarn("got a detection")

    # --------- DECIDE STATE -------------s
    def decide_state(self):
         # idle - amiga teleop 
        if self.plan_executed != 10:
            if self.state <=2:
                pass
            
            elif not self.manipulator_moving and self.plan_executed == self.state:
            # elif self.plan_executed == self.state:

                # once basket move is completed, go to init
                if self.state == 8:
                    self.state = 3

                # if no detections are found during multiframe, go to amiga teleop
                elif self.state == 4 and not self.detection:
                    self.state = 0 # did multiframe but no detection 

                # update state once plan is executed
                else:
                    self.state += 1
            else:
                rospy.loginfo_throttle_identical(1,f"moving: {self.manipulator_moving} , planner: {self.plan_executed},  state: {self.state}")

        else:
            rospy.loginfo_throttle_identical(1,"ERROR: PLANNER ERROR, UNABLE TO EXECUTE PLAN")
            self.state == 10


    # --------- MAIN LOOP -------------
    def run(self):
        """Main loop"""

        while not rospy.is_shutdown():
            if self.state != 10:
                # publish state
                self.decide_state()
                self.state_pub.publish(self.state)
            else:
                # sleep for 10 Hz
                rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        fsm = StateMachineNode()
        fsm.run()
    except rospy.ROSInterruptException:
        pass
