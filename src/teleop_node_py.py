#!/usr/bin/env python3

"""
Buttons on the joystick are used for services
D-pad is used for switching between controlling the arm and driving
Left joystick is used for driving
Right joystick is used for controlling the arm
R1 and L1 is used for the speed
"""

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerRequest
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray

class TeleopNode:
    def __init__(self):

        # Define the different buttons with their corresponding index
        # For axes, all between 1 and -1, with 0 as resting position
        # Up = 1, down = -1
        # Left = 1, right = -1
        self.L_left_right = 0
        self.L_up_down = 1
        self.LT = 2
        self.R_left_right = 3
        self.R_up_down = 4
        self.RT = 5
        self.d_pad_left_right = 6 
        self.d_pad_up_down = 7 

        # For buttons
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


        self.turning_buttons_initiated_ = False
        self.arm1 = True
        self.arm_list = []


        # For arm
        self.position_x_1 = 0.0
        self.position_y_1 = 0.0
        self.position_z_1 = 0.0
        
        self.position_x_2 = 0.0
        self.position_y_2 = 0.0
        self.position_z_2 = 0.0

        self.min_x_arm = 0
        self.max_x_arm = 500
        self.min_y_arm = 0
        self.max_y_arm = 500
        self.min_z_arm = 0
        self.max_z_arm = 500

        self.arm_speed_control = 1
        self.arm_max_speed = 10
        self.arm_min_speed = 0

        # For end effector
        self.end_effector_first = 90
        self.end_effector_second = 90
        self.min_x_end_effector = 0
        self.max_x_end_effector = 180


        # For driving
        self.speed_controll = 0.5
        self.max_speed = 100
        self.min_speed = 0

        """
        self.controlled_arm = False
        """

        # Initialize the ROS node
        rospy.init_node('teleop_node')

        # Create a publisher for the cmd_vel topic
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Create a publisher for the arm_controller topic
        self.arm_posit_pub1 = rospy.Publisher('/arm1position', JointState, queue_size=10)
        self.arm_posit_pub2 = rospy.Publisher('/arm2position', JointState, queue_size=10)

        # Create a publisher for the end effector topic
        self.end_effector_pub = rospy.Publisher('/joint_angles', Int32MultiArray, queue_size=10)

        # Create a subscriber to the "joy" topic with the function "joy_callback" as a callback
        rospy.Subscriber('/joy', Joy, self.joy_callback)

        # Create a service proxy for the "home_steering" service
        self.home_steering_service = rospy.ServiceProxy('home_steering', Trigger)

        # Create a service proxy for the "safety_stop" service
        self.safety_stop_service = rospy.ServiceProxy('safety_stop', Trigger)
        
    # Loop that keeps the node running
    def run(self):
        rospy.spin()


    # Call the "home_steering" service
    def home_steering(self):
        # Create a Trigger request message
        trigger_req = TriggerRequest()

        # Call the "home_steering" service
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
        # Create a Trigger request message
        trigger_req = TriggerRequest()

        # Call the "safety_stop" service
        try:
            response = self.safety_stop_service(trigger_req)
            if response.success:
                rospy.loginfo('Safety stop successfully!')
            else:
                rospy.logwarn('Failed to safety stop.')
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: ' + str(e))


    # Function for drviving the robot
    def driving(self, data):
        # Create a Twist message and populate it with data from the Joy message

        # Adjust the speed of the robot
        if data.buttons[self.RB] == 1:
            self.speed_controll += 0.1
            self.speed_controll = min(max(self.speed_controll, self.min_speed), self.max_speed)
        
        if data.buttons[self.LB] == 1:
            self.speed_controll -= 0.1
            self.speed_controll = min(max(self.speed_controll, self.min_speed), self.max_speed)


        twist = Twist()
        twist.linear.x = data.axes[self.L_up_down] * self.speed_controll # Forward/backward motion (left joystick up/down)
        twist.angular.z = data.axes[self.R_left_right] * self.speed_controll # Rotation (left joystick left/right)

        self.cmd_vel_pub.publish(twist)


    # Function for controlling the arms
    def controll_arm(self, data, pos_x, pos_y, pos_z, pub): 

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

        # Check if the turning buttons are initiated
        # Uses the L2 and R2 buttons to control the z-axis linearly, more pressed bigger increment
        if self.turning_buttons_initiated_:
            pos_z += (1 - data.axes[self.LT]) / 2 * self.arm_speed_control
            pos_z -= (1 - data.axes[self.RT]) / 2 * self.arm_speed_control
            pos_z = min(max(pos_z, self.min_z_arm), self.max_z_arm)

        joint_state = JointState()
        joint_state.position = [pos_x, pos_y, pos_z]

        pub.publish(joint_state)

        if data.buttons[self.B] == 1:
            self.reset_arm()

        return pos_x, pos_y, pos_z
    
    # Puts arm back to starting position
    def reset_arm(self):

        if self.arm1:
            self.position_x_1 = 0.0
            self.position_y_1 = 0.0
            self.position_z_1 = 0.0
        else:
            self.position_x_2 = 0.0
            self.position_y_2 = 0.0
            self.position_z_2 = 0.0


    def end_effector(self, data):

        array = Int32MultiArray()
        self.end_effector_first += data.axes[self.d_pad_up_down]
        self.end_effector_first = min(max(self.end_effector_first, self.min_x_end_effector), self.max_x_end_effector)

        self.end_effector_second += data.axes[self.d_pad_left_right]
        self.end_effector_second = min(max(self.end_effector_second, self.min_x_end_effector), self.max_x_end_effector)

        array.data = [int(self.end_effector_first), int(self.end_effector_second)]
        self.end_effector_pub.publish(array)

        if data.buttons[self.X] == 1:
            self.end_effector_reset(data)

    # Puts end effector back to starting position
    def end_effector_reset(self, data):
        self.end_effector_second = 90
        self.end_effector_first = 90

    def end_effector_pose(self, data):
        pass


    """
    # Function that sets all value to 0 for safety measure when switching to arm mode
    def speed_zero(self, data):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)
        
        buttons = [0] * len(data.buttons)
        data.buttons = buttons
    """

    def joy_callback(self, data):

        # Bypass that R2 and L2 starts with 0 as default value and changes to 1 when pressed
        # When using these buttons make need to check if they are initiated
        if not self.turning_buttons_initiated_ and data.axes[self.LT] == 1 and data.axes[self.RT] == 1:
            self.turning_buttons_initiated_ = True

        if data.buttons[self.X] == 1:
            self.arm1 = not self.arm1 


        # Check if buttons are pressed
        if data.buttons[self.A] == 1:
            self.home_steering()

        if data.buttons[self.R3] == 1 and data.buttons[self.L3] == 1:
            self.safety_stop()

        if data.buttons[self.Y] == 1:
            self.end_effector(data)
        elif not self.arm1:
            self.position_x_2, self.position_y_2, self.position_z_2 = self.controll_arm(data, self.position_x_2, self.position_y_2, self.position_z_2, self.arm_posit_pub2)
        else:
            self.position_x_1, self.position_y_1, self.position_z_1 = self.controll_arm(data, self.position_x_1, self.position_y_1, self.position_z_1, self.arm_posit_pub1)




        self.driving(data)

        """
        # Switch between controlling the arm and driving,
        # Set all values to 0 for safety measure
        if data.axes[self.d_pad_up_down] == 1:
            self.controlled_arm = False
            rospy.loginfo('Driving enabled')
            self.speed_zero(data)

        if data.axes[self.d_pad_up_down] == -1:
            self.controlled_arm = True
            rospy.loginfo('Arm control enabled')
            self.speed_zero(data)

        if self.controlled_arm:
            self.controll_arm(data)
        else:
            self.driving(data)
        """

if __name__ == '__main__':
    Teleop_Node = TeleopNode()
    Teleop_Node.run()



"""
Write in c++

Write code that uses button to move end effector to nine preset poses

Fiks controller for whats up and down on end effector





Write better code
Safety stop on arm
Safety stop on driving

One node for driving
One node for arm
One for both

Use R1 and L1 for speed up and down
Use buttons for deegrees to arm end


Hold button and press a button to switch between cameras
"""