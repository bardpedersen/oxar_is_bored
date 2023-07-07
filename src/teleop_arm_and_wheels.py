#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerRequest
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray

class TeleopNode:
    def __init__(self):

        # Define the different buttons with their corresponding index

        # Different axes, all between 1 and -1, with 0 as resting position
        # Up = 1, down = -1, Left = 1, right = -1
        self.L_left_right = 0
        self.L_up_down = 1
        self.LT = 2
        self.R_left_right = 3
        self.R_up_down = 4
        self.RT = 5
        self.d_pad_left_right = 6 
        self.d_pad_up_down = 7 

        # Different buttons, 1 when pressed, 0 when not pressed
        self.A = 0
        self.B = 1
        self.X = 2
        self.Y = 3
        self.LB = 4
        self.RB = 5
        self.back = 6
        self.start = 7
        self.xbox = 8
        self.L3 = 9
        self.R3 = 10


        # Simple varibles for checkings statements
        self.T_buttons_initiated_ = False
        self.arm1_initiated = True
        self.X_button_prev_state = False
        self.endeffector1_initiated = True
        self.xbox_button_prev_state = False
        self.L3_R3_button_prev_state = False


        # Variables for storing arm positions
        self.position_x_1 = 0
        self.position_y_1 = 200
        self.position_z_1 = 300
        
        self.position_x_2 = 0
        self.position_y_2 = 200
        self.position_z_2 = 300

        # Variables for home position for the arms 
        self.home_position_x = 0
        self.home_position_y = 200
        self.home_position_z = 300

        # Variables for boundries for the arms
        self.min_x_arm = 0
        self.max_x_arm = 500
        self.min_y_arm = 0
        self.max_y_arm = 500
        self.min_z_arm = 0
        self.max_z_arm = 500

        # Variables for speed control for the arms
        self.arm_speed_control = 1
        self.arm_max_speed = 10
        self.arm_min_speed = 0.5


        # Variables for storing end effector positions
        self.end_effector1_M1 = 90
        self.end_effector1_M2 = 90

        self.end_effector2_M1 = 90
        self.end_effector2_M2 = 90
        
        # Variables for boundries for the end effector
        self.min_x_end_effector = 0
        self.max_x_end_effector = 180

        # Variables for speed control for the end effector
        self.end_effector_speed_control = 1
        self.end_effector_max_speed = 20
        self.end_effector_min_speed = 0.5


        # Variables for speed control for driving
        self.speed_controll = 0.5
        self.max_speed = 100
        self.min_speed = 0.5


        # Initialize the ROS node
        rospy.init_node('teleop_node')

        # Create a publisher for the cmd_vel topic
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Create publisher for the arm controller topic
        self.arm_posit_pub1 = rospy.Publisher('/arm1position', JointState, queue_size=10)
        self.arm_posit_pub2 = rospy.Publisher('/arm2position', JointState, queue_size=10)

        # Create publisher for the end effector controller topic
        self.end_effector_pub1 = rospy.Publisher('/endeffector1', Int32MultiArray, queue_size=10)
        self.end_effector_pub2 = rospy.Publisher('/endeffector2', Int32MultiArray, queue_size=10)

        # Create a subscriber to the "joy" topic with the function "joy_callback" as a callback
        rospy.Subscriber('/joy', Joy, self.joy_callback)

        # Create a service proxy for the "home_steering" service
        self.home_steering_service = rospy.ServiceProxy('home_steering', Trigger)

        # Create a service proxy for the "safety_stop" service
        self.safety_stop_service = rospy.ServiceProxy('safety_stop', Trigger)
        

    # Loop that keeps the ros node running
    def run(self):
        rospy.spin()


    # Call the "home_steering" service
    def home_steering(self):
        trigger_req = TriggerRequest()

        try:
            response = self.home_steering_service(trigger_req)
            if response.success:
                rospy.loginfo('Steering homed successfully!')
            else:
                rospy.logwarn('Failed to home steering.')
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: ' + str(e))


    # Call the "safety_stop" service
    def safety_stop(self):
        trigger_req = TriggerRequest()

        try:
            response = self.safety_stop_service(trigger_req)
            if response.success:
                rospy.loginfo('Safety stop successfully!')
            else:
                rospy.logwarn('Failed to safety stop.')
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: ' + str(e))


    # Function for driving the robot
    def driving(self, data):

        # Adjust the speed of the robot
        if data.buttons[self.RB] == 1:
            self.speed_controll += 0.1
            self.speed_controll = min(max(self.speed_controll, self.min_speed), self.max_speed)
        
        if data.buttons[self.LB] == 1:
            self.speed_controll -= 0.1
            self.speed_controll = min(max(self.speed_controll, self.min_speed), self.max_speed)

        # Create a Twist message and publish it with controls from the joysticks
        twist = Twist()
        twist.linear.x = data.axes[self.L_up_down] * self.speed_controll # Forward/backward motion (left joystick up/down)
        twist.angular.z = data.axes[self.R_left_right] * self.speed_controll # Rotation (left joystick left/right)

        self.cmd_vel_pub.publish(twist)


    # Function for controlling the arms
    def controll_arm(self, data, pos_x, pos_y, pos_z, pub): 

        # Adjust the speed of the arm
        if data.buttons[self.start] == 1:
            self.arm_speed_control += 0.1
            self.arm_speed_control = min(max(self.arm_speed_control, self.arm_min_speed), self.arm_max_speed)
        
        if data.buttons[self.back] == 1:
            self.arm_speed_control -= 0.1
            self.arm_speed_control = min(max(self.arm_speed_control, self.arm_min_speed), self.arm_max_speed)


        # Increasing the values within the limits, the d-pad
        pos_x += data.axes[self.d_pad_up_down] * self.arm_speed_control
        pos_x = min(max(pos_x, self.min_x_arm), self.max_x_arm)

        pos_y += data.axes[self.d_pad_left_right] * self.arm_speed_control
        pos_y = min(max(pos_y, self.min_y_arm), self.max_y_arm)

        # Check if the LT, RT buttons are initiated
        if self.T_buttons_initiated_:
            # Uses the LT and RT buttons to control the z-axis linearly, more pressed bigger increment
            pos_z += (1 - data.axes[self.LT]) / 2 * self.arm_speed_control
            pos_z -= (1 - data.axes[self.RT]) / 2 * self.arm_speed_control
            pos_z = min(max(pos_z, self.min_z_arm), self.max_z_arm)

        # Reset the arm to home position
        if data.buttons[self.B] == 1:
            pos_x = self.home_position_x
            pos_y = self.home_position_y
            pos_z = self.home_position_z

        # Publish the arm position
        joint_state = JointState()
        joint_state.position = [pos_x, pos_y, pos_z]
        joint_state.velocity = [0.0]
        joint_state.effort = [0.0]

        pub.publish(joint_state)

        return pos_x, pos_y, pos_z

    # Function for controlling the end effector
    def end_effector(self, data, M1, M2, pub):
        # Adjust the speed of the end effector
        if data.buttons[self.start] == 1:
            self.end_effector_speed_control += 0.1
            self.end_effector_speed_control = min(max(self.end_effector_speed_control, self.end_effector_min_speed), self.end_effector_max_speed)
        
        if data.buttons[self.back] == 1:
            self.end_effector_speed_control -= 0.1
            self.end_effector_speed_control = min(max(self.end_effector_speed_control, self.end_effector_min_speed), self.end_effector_max_speed)

        # Increasing the values within the limits, the d-pad
        M1 += data.axes[self.d_pad_up_down] * self.end_effector_speed_control
        M1 = min(max(M1, self.min_x_end_effector), self.max_x_end_effector)

        M2 += data.axes[self.d_pad_left_right] * self.end_effector_speed_control
        M2 = min(max(M2, self.min_x_end_effector), self.max_x_end_effector)

        # Reset the end effector to home position
        if data.buttons[self.B] == 1:
            M1 = 90
            M2 = 90

        # Publish the end effector position
        array = Int32MultiArray()
        array.data[0] = int(M1)
        array.data[1] = int(M2)
        pub.publish(array)

        return M1, M2

    # Callback function for the "joy" topic
    def joy_callback(self, data):

        # Bypass that RT and LT starts with 0 as default value and default value changes to 1 when pressed.
        # Check if the RT and LT buttons have been pressed, first then are they in use
        if not self.T_buttons_initiated_ and data.axes[self.LT] == 1 and data.axes[self.RT] == 1:
            self.T_buttons_initiated_ = True

        # Changes only when x button is pressed, not hold down
        if data.buttons[self.X] == 1 and not self.X_button_prev_state:
            self.arm1_initiated = not self.arm1_initiated 
            if self.arm1_initiated:
                rospy.loginfo('Arm 1 enabled')
            elif not self.arm1_initiated:
                rospy.loginfo('Arm 2 enabled')
        self.X_button_prev_state = data.buttons[self.X]
    

        if data.buttons[self.xbox] == 1 and not self.xbox_button_prev_state:
            self.endeffector1_initiated = not self.endeffector1_initiated 
            if self.endeffector1_initiated:
                rospy.loginfo('End effector 1 enabled')
            elif not self.endeffector1_initiated:
                rospy.loginfo('End effector 2 enabled')
        self.xbox_button_prev_state = data.buttons[self.xbox]


        # Call services
        if data.buttons[self.A] == 1:
            self.home_steering()

        if data.buttons[self.R3] == 1 and data.buttons[self.L3] == 1:
            self.safety_stop()

        # Choses witch arm and end effector to control
        if data.buttons[self.Y] == 1:
            if not self.endeffector1_initiated:
                self.end_effector2_M1, self.end_effector2_M2 = self.end_effector(data, self.end_effector2_M1, self.end_effector2_M2, self.end_effector_pub2)
            else:
                self.end_effector1_M1, self.end_effector1_M2 = self.end_effector(data, self.end_effector1_M1, self.end_effector1_M2, self.end_effector_pub1)


        elif not self.arm1_initiated:
            self.position_x_2, self.position_y_2, self.position_z_2 = self.controll_arm(data, self.position_x_2, self.position_y_2, self.position_z_2, self.arm_posit_pub2)
        else:
            self.position_x_1, self.position_y_1, self.position_z_1 = self.controll_arm(data, self.position_x_1, self.position_y_1, self.position_z_1, self.arm_posit_pub1)

        # Call driving function
        self.driving(data)


if __name__ == '__main__':
    Teleop_Node = TeleopNode()
    Teleop_Node.run()


"""
D-pad is used for controlling the arm and end effector
LT and RT is used for controlling the z-axis of the arm
Joysticks are used for driving
Buttons on pad are used for service, switch arms and end effector and reset arm position
Xbox button is used for switching between end effectors
R1 and L1 is used for the speed
Start and back is used for changing the speed of the arm and end effector
"""

"""
Write better code
Write in c++

Fiks controller for whats up and down on end effector and arms.
Global coordinates, front forward etc.

Safety stop on arm
Safety stop on truning

One node for driving
One node for arm -(Write code that uses button to move end effector to nine preset poses)
One for both

Hold button and press a button to switch between cameras

Z- button improve 

active arm one
activate arm two
then both are active

Turn off possitin for arms:
(x 0
Y 200
Z 500)
"""