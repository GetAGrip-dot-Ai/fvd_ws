#!/usr/bin/env python3

### ROS node for state machine
import rospy
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16, Bool

from xarm_msgs.srv import SetInt16, SetInt16Response
from numpy.linalg import norm

"""
---- Possible states -----
--Manual--
idle: 0
teleop: 1
visual servo: 2
return to init: 3

--Autonomous--
move to init: 4
multiframe: 5
move to pregrasp: 6
move to poi: 7
harvest: 8
move to basket: 9
ERROR: 10

"""

class StateMachineNode:
    """ROS node for state machine"""
    def __init__(self):
        rospy.init_node('state_machine_node', anonymous=True)

        # --------- JOYSTICK -------------
        self.joy_state = Joy()
        # subscribe to joystick message
        self.joystick_callback = rospy.Subscriber('/joy', Joy, self.joystick_callback, queue_size=1)

        # --------- STATE PUBLISHER INITIALIZATION -------------
        # create publisher that contiously publishes state at specified rate
        self.state_pub = rospy.Publisher('/state', Int16, queue_size=1)
        self.state = 0 # initial state is 0 (idle)
        
        # --------- XARM SERVICE INITIALIZATION -------------
        # make a service server to be able to change the arm state
        self.xarm_mode_service = rospy.ServiceProxy("/xarm/set_mode", SetInt16)
        self.xarm_state_service = rospy.ServiceProxy("/xarm/set_state", SetInt16)

        # --------- ROBOT SUBSYSTEM SUBSCRIBERS -------------
        # subscribe to xarm joint state
        self.manipulator_state_sub = rospy.Subscriber('/xarm/jointState', JointState, self.manipulator_state_callback, queue_size=1)
        # subscribe to perception node
        self.perception_state_sub = rospy.Subscriber('/perception_state', Int16, self.perception_state_callback, queue_size=1)
        # subscribe to planner node
        self.planner_state_sub = rospy.Subscriber('/planner_state', Int16, self.planner_state_callback, queue_size=1)

        # --------- ROBOT SUBSYSTEM VARIABLES -------------
        self.joint_velocity = None
        self.manipulator_moving = None
        self.zero_vel_threshold = 1e-3
        self.perception_state = None
        self.planner_state = None
        self.plan_executed = None

    # TODO: Incorperate perception feedback
    # --------- PERCEPTION STATE CALLBACK ------------- #
    # subscribe to topic with image detections

    # subscribe to topic with depth info 
    def perception_state_callback(self, data):
        """Callback for perception state message"""
        self.perception_state = data.data

    # --------- PLANNER STATE CALLBACK -------------
    def planner_state_callback(self, data):
        """Callback for planner state message"""
        if self.plan_executed != data.data:
            print("plan executed - state machine")  #debug
            self.plan_executed = data.data

    # --------- MANIPULATOR STATE CALLBACK -------------
    def manipulator_state_callback(self, joint):
        """Callback for manipulator state message"""

        if norm(joint.velocity) < self.zero_vel_threshold:
            rospy.loginfo_throttle_identical(10,"STATE MACHINE: Manipulator is still")
            self.manipulator_moving = False
        else:
            rospy.loginfo_throttle_identical(10,"STATE MACHINE: Manipulator is moving")
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

        if self.joy_state[6] and self.state != 1:
            self.state = 1
            rospy.loginfo("Enter Manual Mode")

        # xbox button -> vs mode
        elif self.joy_state[8] and self.state != 2: 
            self.state = 4
            rospy.loginfo("Initializing via State 4")
            
        # Y -> enter idle mode
        elif self.joy_state[3] and self.state != 0:
            self.state = 0
            rospy.loginfo("Enter Idle Mode")                
        

    # --------- DECIDE STATE -------------
    def decide_state(self):
         # idle - manual only 
        if self.state == 0:
            pass    # state decided by joystick callback

        # teleop - manual only
        elif self.state == 1:
            pass    # state decided by joystick callback
        
        # visual servo - manual only
        elif self.state == 2:
            pass    # state decided by joystick callback

        # return to init - manual only
        elif self.state == 3:
            pass    # state decided by joystick callback

        # move to init pose
        elif self.state == 4 and self.plan_executed == 4 and not self.manipulator_moving:
            self.state = 5

        # multiframe
        elif self.state == 5 and self.plan_executed == 5 and not self.manipulator_moving:
            self.state = 6
    
        # move to pregrasp
        elif self.state == 6 and self.plan_executed == 6 and not self.manipulator_moving:
            self.state = 7

        # move to poi
        elif self.state == 7 and self.plan_executed == 7 and not self.manipulator_moving:
            self.state = 8

        # harvest pepper
        elif self.state == 8 and self.plan_executed == 8 and not self.manipulator_moving:
            self.state = 9

        # basket drop
        elif self.state == 9 and self.plan_executed == 9 and not self.manipulator_moving:
            # if more pepper seen in pepper detection topic
                self.state = 4
            # else:
                rospy.loginfo_throttle_identical(10,"done with autonomous harvesting sequence")

        else:
            rospy.loginfo_throttle_identical(1,"ERROR: UNRECOGNIZED STATE IN STATE MACHINE NODE")
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
