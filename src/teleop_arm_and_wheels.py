#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerRequest
from std_srvs.srv import SetBool, SetBoolRequest
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import PoseArray

class TeleopNode:
    def __init__(self):

        # Get the button and axes mapping from the parameter server
        self.button_mapping = rospy.get_param('button_map')
        self.axes_mapping = rospy.get_param('axes_map')

        # Get button action and correspondoing button from paramter server
        self.activate_arm_button = rospy.get_param('activate_arm_button')
        self.activate_endef_button = rospy.get_param('activate_endef_button')
        self.use_endef = rospy.get_param('use_endef')

        self.increase_arm_speed = rospy.get_param('increase_arm_speed')
        self.decrease_arm_speed = rospy.get_param('decrease_arm_speed')
        self.increase_drive_speed = rospy.get_param('increase_drive_speed')
        self.decrease_drive_speed = rospy.get_param('decrease_drive_speed')

        self.arm_up = rospy.get_param('arm_up')
        self.arm_down = rospy.get_param('arm_down')
        self.arm_x = rospy.get_param('arm_x')
        self.arm_y = rospy.get_param('arm_y')
        self.endef_up = rospy.get_param('endef_up')
        self.endef_side = rospy.get_param('endef_side')
        self.drive_forward = rospy.get_param('drive_forward')
        self.drive_turning = rospy.get_param('drive_turning')

        self.home_button = rospy.get_param('home_button')
        self.safety_stop_button = rospy.get_param('safety_stop_button')
        self.home_steering_button = rospy.get_param('home_steering_button')

        # Get the restrictions from the parameter server
        self.min_x_arm = rospy.get_param('min_x_arm') 
        self.max_x_arm = rospy.get_param('max_x_arm') 
        self.min_y_arm = rospy.get_param('min_y_arm') 
        self.max_y_arm = rospy.get_param('max_y_arm') 
        self.min_z_arm = rospy.get_param('min_z_arm') 
        self.max_z_arm = rospy.get_param('max_z_arm') 

        self.min_x_end_effector = rospy.get_param('min_x_end_effector')
        self.max_x_end_effector = rospy.get_param('max_x_end_effector')

        self.arm_max_speed = rospy.get_param('arm_max_speed')
        self.arm_min_speed = rospy.get_param('arm_min_speed')

        self.drive_max_speed = rospy.get_param('drive_max_speed')
        self.drive_min_speed = rospy.get_param('drive_min_speed')

        # Variables for home position for the arms 
        self.home_position_x = rospy.get_param('home_position_x')
        self.home_position_y = rospy.get_param('home_position_y')
        self.home_position_z = rospy.get_param('home_position_z')

        # Simple varibles for checkings statements
        self.T_buttons_initiated_ = False
        self.arm1_initiated = True
        self.endeffector1_initiated = True
        self.L3_R3_button_prev_state = False
        self.safety_stop_= False
        self.reset_values = False

        self.previous_button_pressed = [0] * len(self.button_mapping)

        # Variables for storing arm positions
        self.position_x_1 = 0
        self.position_y_1 = 200
        self.position_z_1 = 300
        
        self.position_x_2 = 0
        self.position_y_2 = 200
        self.position_z_2 = 300

        # Variables for storing end effector positions
        self.end_effector1_M1 = 90
        self.end_effector1_M2 = 90

        self.end_effector2_M1 = 90
        self.end_effector2_M2 = 90

        # Variables for speed control for the arms
        self.arm_speed_control = 2

        # Variables for speed control for driving
        self.speed_controll = 0.5

         # Set the ros rate to be the same as the arms
        self.rate = rospy.Rate(40)
        self.joy_data = 0

        # Create a publisher for the cmd_vel topic
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # Create publisher for the arm controller topic
        self.arm_posit_pub1 = rospy.Publisher('arm1position', JointState, queue_size=10)
        self.arm_posit_pub2 = rospy.Publisher('arm2position', JointState, queue_size=10)

        # Create publisher for the end effector controller topic
        self.end_effector_pub1 = rospy.Publisher('endeffector1', Int32MultiArray, queue_size=10)
        self.end_effector_pub2 = rospy.Publisher('endeffector2', Int32MultiArray, queue_size=10)

        # Create a service proxy for the "safety_stop" service
        self.safety_stop_service1 = rospy.ServiceProxy('safety_stop_arm1', SetBool)
        self.safety_stop_service2 = rospy.ServiceProxy('safety_stop_arm2', SetBool)

        # Create a service proxy for the "safety_stop" service, for wheels
        self.safety_stop_service_wheel = rospy.ServiceProxy('safety_stop', SetBool)

        # Create a service proxy for the "home_steering" service
        self.home_steering_service = rospy.ServiceProxy('home_steering', Trigger)

        # Subscribe to actual values
        rospy.Subscriber('arm1_cur_pos', PoseArray, self.arm_pos1_callback)
        rospy.Subscriber('arm2_cur_pos', PoseArray, self.arm_pos2_callback)

        # Create a subscriber to the "joy" topic with the function "joy_callback" as a callback
        rospy.Subscriber('joy_arms_wheels', Joy, self.joy_callback)

        rospy.loginfo('Teleop_node started')
        rospy.loginfo('Arm 1 enabled')
        rospy.loginfo('End effector 1 enabled')


    # Makes position follow the real values when controller is not in use
    def arm_pos1_callback(self, data):
        if self.reset_values:
            for pose in data.poses:
                self.position_x_1 = pose.position.x
                self.position_y_1 = pose.position.y
                self.position_z_1 = pose.position.z

    def arm_pos2_callback(self, data):
        if self.reset_values:
            for pose in data.poses:
                self.position_x_2 = pose.position.x
                self.position_y_2 = pose.position.y
                self.position_z_2 = pose.position.z


    # Calls safety stop service to stop arms
    def safety_stop(self):
        request = SetBoolRequest()
        request.data = self.safety_stop_
        # Arm1
        try:
            response = self.safety_stop_service1(request)
            if response.success:
                rospy.loginfo('Safety stop arm1 successfully!')
            else:
                rospy.logwarn('Failed to safety stop.')
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: ' + str(e))

        # Arm 2
        try:
            response = self.safety_stop_service2(request)
            if response.success:
                rospy.loginfo('Safety stop arm2 successfully!')
            else:
                rospy.logwarn('Failed to safety stop.')
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: ' + str(e))

        # Wheels
        try:
            response = self.safety_stop_service_wheel(request)
            if response.success:
                rospy.loginfo('Safety stop wheels successfully!')
            else:
                rospy.logwarn('Failed to safety stop.')
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: ' + str(e))


    def home_steering(self):
        # home_steering for wheels
        trigger_req = TriggerRequest()
        try:
            response = self.home_steering_service(trigger_req)
            if response.success:
                rospy.loginfo('Safety stop successfully!')
            else:
                rospy.logwarn('Failed to safety stop.')
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: ' + str(e))


    # Function for driving the robot
    def driving_speed(self):

        # Adjust the speed of the robot
        if self.evaluate_button(self.increase_drive_speed):
            self.speed_controll += 0.1
            self.speed_controll = max(min(self.speed_controll, self.drive_max_speed), self.drive_min_speed)
        
        if self.evaluate_button(self.decrease_drive_speed):
            self.speed_controll -= 0.1
            self.speed_controll = max(min(self.speed_controll, self.drive_max_speed), self.drive_min_speed)


    # Function for controlling the arms
    def controll_arm(self, pos_x, pos_y, pos_z, left=False): 

        # Controlls for right arm
        if not left:
            x_nav = self.joy_data.axes[self.axes_mapping[self.arm_x]]
            y_nav = self.joy_data.axes[self.axes_mapping[self.arm_y]]

        # Controlls for left arm
        if left:
            x_nav = self.joy_data.axes[self.axes_mapping[self.arm_x]]
            y_nav = -self.joy_data.axes[self.axes_mapping[self.arm_y]]

        # Increasing the values within the limits
        pos_x += x_nav * self.arm_speed_control
        pos_x = min(max(pos_x, self.min_x_arm), self.max_x_arm)

        pos_y += y_nav * self.arm_speed_control
        pos_y = min(max(pos_y, self.min_y_arm), self.max_y_arm)

        # Check if the LT, RT buttons are initiated
        if self.T_buttons_initiated_:
            # Uses the LT and RT buttons to control the z-axis linearly, more pressed bigger increment
            pos_z += (1 - self.joy_data.axes[self.axes_mapping[self.arm_up]]) / 2 * self.arm_speed_control
            pos_z -= (1 - self.joy_data.axes[self.axes_mapping[self.arm_down]]) / 2 * self.arm_speed_control
            pos_z = min(max(pos_z, self.min_z_arm), self.max_z_arm)

        # Reset the arm to home position
        if self.evaluate_button(self.home_button):
            pos_x = self.home_position_x
            pos_y = self.home_position_y
            pos_z = self.home_position_z
            rospy.loginfo('Arm reset')


        # Returns the updated varibals so it can be stored
        return pos_x, pos_y, pos_z

    # Function for controlling the end effector
    def end_effector(self, M1, M2, left=False):
        # Controlls for right end effector
        if not left:
            m1_nav = -self.joy_data.axes[self.axes_mapping[self.endef_up]]
            m2_nav = -self.joy_data.axes[self.axes_mapping[self.endef_side]]

        # Controlls for left end effector
        if left:
            m1_nav = -self.joy_data.axes[self.axes_mapping[self.endef_up]]
            m2_nav = -self.joy_data.axes[self.axes_mapping[self.endef_side]]

        # Increasing the values within the limits
        M1 += m1_nav * self.arm_speed_control
        M1 = min(max(M1, self.min_x_end_effector), self.max_x_end_effector)

        M2 += m2_nav * self.arm_speed_control
        M2 = min(max(M2, self.min_x_end_effector), self.max_x_end_effector)

        # Reset the end effector to home position
        if self.evaluate_button(self.home_button):
            M1 = 90
            M2 = 90
            rospy.loginfo('End effector reset')

        # Returns the updated varibals so it can be stored
        return M1, M2

    # Callback function for the "joy" topic
    def joy_callback(self, data):
        self.joy_data = data

    # Returns true once when button is held down or pressed once
    def evaluate_button(self, button):
        if self.joy_data.buttons[self.button_mapping[button]] == 1 and self.previous_button_pressed[self.button_mapping[button]] != 1:
            return True
        else:
            return False
        

    # Loop that keeps the ros node running
    def run(self):
        while not rospy.is_shutdown():
            # Checks if the joy_data has been recived
            if self.joy_data!=0:
                # Bypass that RT and LT starts with 0 as default value and default value changes to 1 when pressed.
                # Check if the RT and LT buttons have been pressed, first then are they in use
                if not self.T_buttons_initiated_ and self.joy_data.axes[2] == 1 and self.joy_data.axes[5] == 1:
                    self.T_buttons_initiated_ = True
                    rospy.loginfo("LT and RT are ready to be used")

                    # Changes only when x button is pressed, not hold down
                    # Switches between left and right arm
                if self.evaluate_button(self.activate_arm_button):
                        self.arm1_initiated = not self.arm1_initiated 
                        if self.arm1_initiated:
                            rospy.loginfo('Arm 1 enabled')
                        elif not self.arm1_initiated:
                            rospy.loginfo('Arm 2 enabled')
                
                # Switches between left and right end effector
                if self.evaluate_button(self.activate_endef_button):
                    self.endeffector1_initiated = not self.endeffector1_initiated 
                    if self.endeffector1_initiated:
                        rospy.loginfo('End effector 1 enabled')
                    elif not self.endeffector1_initiated:
                        rospy.loginfo('End effector 2 enabled')

                # Adjust the speed of the arms
                if self.evaluate_button(self.increase_arm_speed):
                    self.arm_speed_control += 0.1
                    self.arm_speed_control = min(max(self.arm_speed_control, self.arm_min_speed), self.arm_max_speed)
                    rospy.loginfo(self.arm_speed_control)

                if self.evaluate_button(self.decrease_arm_speed):
                    self.arm_speed_control -= 0.1
                    self.arm_speed_control = min(max(self.arm_speed_control, self.arm_min_speed), self.arm_max_speed)
                    rospy.loginfo(self.arm_speed_control)

                # Activates the emergency stop 
                if self.joy_data.buttons[self.button_mapping[self.safety_stop_button[0]]] == 1 and self.joy_data.buttons[self.button_mapping[self.safety_stop_button[1]]] == 1 and not self.L3_R3_button_prev_state:
                    self.safety_stop_ = not self.safety_stop_
                    self.safety_stop()
                    if self.safety_stop_:
                        rospy.loginfo("Safety Enabled")
                    else:
                        rospy.loginfo("Safety Disabled")
                        
                if self.joy_data.buttons[self.button_mapping[self.safety_stop_button[0]]] == 1 and self.joy_data.buttons[self.button_mapping[self.safety_stop_button[1]]] == 1:
                    self.L3_R3_button_prev_state = True
                else:
                    self.L3_R3_button_prev_state = False


                # Choses witch arm and end effector to controll with joy
                if not self.safety_stop_:
                    # Can only controll end effector when Y is pressed down
                    if self.joy_data.buttons[self.button_mapping[self.use_endef]]!=0 or self.joy_data.axes[self.axes_mapping[self.arm_down]]!=1 or self.joy_data.axes[self.axes_mapping[self.arm_up]]!=1 or self.joy_data.axes[self.axes_mapping[self.arm_x]]!=0 or self.joy_data.axes[self.axes_mapping[self.arm_y]]!=0 or self.joy_data.buttons[self.button_mapping[self.home_button]]!=0:
                        self.reset_values = False
                        if self.joy_data.buttons[self.button_mapping[self.use_endef]] == 1:
                            if not self.endeffector1_initiated:
                                self.end_effector2_M1, self.end_effector2_M2 = self.end_effector(self.end_effector2_M1, self.end_effector2_M2, left=True)
                            else:
                                self.end_effector1_M1, self.end_effector1_M2 = self.end_effector(self.end_effector1_M1, self.end_effector1_M2)

                            array = Int32MultiArray()
                            array.data = [int(self.end_effector1_M1), int(self.end_effector1_M2)]
                            self.end_effector_pub1.publish(array)

                            array = Int32MultiArray()
                            array.data = [int(self.end_effector2_M1), int(self.end_effector2_M2)]
                            self.end_effector_pub2.publish(array)

                        elif not self.arm1_initiated:
                            self.position_x_2, self.position_y_2, self.position_z_2 = self.controll_arm(self.position_x_2, self.position_y_2, self.position_z_2, left=True)
                        else:
                            self.position_x_1, self.position_y_1, self.position_z_1 = self.controll_arm(self.position_x_1, self.position_y_1, self.position_z_1)
                        
                        joint_state = JointState()
                        joint_state.position = [self.position_x_1, self.position_y_1, self.position_z_1]
                        joint_state.velocity = [0.0]
                        joint_state.effort = [0]
                        self.arm_posit_pub1.publish(joint_state)

                        joint_state = JointState()
                        joint_state.position = [self.position_x_2, self.position_y_2, self.position_z_2]
                        joint_state.velocity = [0.0]
                        joint_state.effort = [0]
                        self.arm_posit_pub2.publish(joint_state)

                    self.reset_values = True

                    # Call services
                    if self.evaluate_button(self.home_steering_button):
                        self.home_steering()


                    # Create a Twist message and publish it with controls from the joysticks
                    self.driving_speed()
                    twist = Twist()
                    twist.linear.x = self.joy_data.axes[self.axes_mapping[self.drive_forward]] * self.speed_controll
                    twist.angular.z = self.joy_data.axes[self.axes_mapping[self.drive_turning]] * self.speed_controll 
                    self.cmd_vel_pub.publish(twist)

                self.previous_button_pressed = self.joy_data.buttons
            self.rate.sleep()


if __name__ == '__main__':
    # Initiate node
    # Initialize the ROS node
    rospy.init_node('teleop_node')
    Teleop_Node = TeleopNode()

    # Keep script running
    Teleop_Node.run()

"""
Safety stop on truning

Hold button and press a button to switch between cameras
"""