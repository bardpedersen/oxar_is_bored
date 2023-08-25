#!/usr/bin/env python3

import numpy as np
import rospy
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool, SetBoolRequest
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray, Float32MultiArray, Bool
from geometry_msgs.msg import PoseArray


class TeleopNode:
    def __init__(self):
        # Load all params from the parameter server
        self.load_params()

        # Initialize variables for storing positions and states
        self.armposition_1 = np.array([0, 200, 300])
        self.armposition_2 = np.array([0, 200, 300])
        self.end_effector1_angles = np.array([90, 90])
        self.end_effector2_angles = np.array([90, 90])

        # Initialize variables for storing states
        self.joy_data = None
        self.previous_button_pressed = [0] * len(self.button_mapping)
        self.safety_stop_ = False
        self.L3_R3_button_prev_state = False
        self.T_buttons_initiated_ = False

        self.arm1_initiated = False
        self.arm2_initiated = False
        self.real_armposition_1 = None
        self.real_armposition_1 = None
        self.endeffector1_initiated = False
        self.endeffector2_initiated = False
        self.arm_speed_control = 2

        self.global_frame_point = True
        self.mirror_movement = False

        self.arm_straight_line = False
        self.active_cosinusodal = False
        self.active_end_effector_move = False
        self.move_arm_from_list = False
        self.arm_right_move_index = 0
        self.arm_left_move_index = 0
        self.end_right_move_index = 0
        self.end_left_move_index = 0
        self.time_right_move_index = 0
        self.time_left_move_index = 0
        self.time_index = 180
        self.sinus_forward = True
        self.cosinus_forward = True
        self.cosinus_upward = True
        self.sinus_x_number = 0
        self.cosinus_x_number = 200
        self.cosinus_z_number = 200
        self.function_state = 0

        # Initialize ROS node with 60 Hz
        self.rate = rospy.Rate(50)

        # Initialize ROS publishers
        self.arm1_position_pub = rospy.Publisher('arm1position', JointState, queue_size=10)
        self.arm2_position_pub = rospy.Publisher('arm2position', JointState, queue_size=10)
        self.arm1_away_position = rospy.Publisher('arm1_calib_stretched_cmd', Bool, queue_size=10)
        self.arm2_away_position = rospy.Publisher('arm2_calib_stretched_cmd', Bool, queue_size=10)
        self.arm1_calib = rospy.Publisher('arm1_calib_cmd', Bool, queue_size=10)
        self.arm2_calib = rospy.Publisher('arm2_calib_cmd', Bool, queue_size=10)
        self.end_effector1_pub = rospy.Publisher('endeffector1', Int32MultiArray, queue_size=10)
        self.end_effector2_pub = rospy.Publisher('endeffector2', Int32MultiArray, queue_size=10)

        # Initialize ROS services
        self.safety_stop_service1 = rospy.ServiceProxy('safety_stop_arm1', SetBool)
        self.safety_stop_service2 = rospy.ServiceProxy('safety_stop_arm2', SetBool)

        # Initialize ROS subscribers
        rospy.Subscriber('arm1_cur_pos', PoseArray, self.arm1_pos_callback)
        rospy.Subscriber('arm2_cur_pos', PoseArray, self.arm2_pos_callback)
        rospy.Subscriber('arm1_angle', Float32MultiArray, self.endeff1_pos_callback)
        rospy.Subscriber('arm2_angle', Float32MultiArray, self.endeff2_pos_callback)
        rospy.Subscriber('joy_arms', Joy, self.joy_callback)

        # Initialize ROS node
        rospy.loginfo('Teleop_node started')
        rospy.loginfo('Press LB and RB to enable the arms')
        rospy.loginfo('Press X and Y to enable the end effectors')
        rospy.loginfo('Press L3 and R3 to enable safety stop')

    def load_params(self):
        # Get the button and axes mapping from the parameter server
        self.button_mapping = rospy.get_param('button_map')
        self.axes_mapping = rospy.get_param('axes_map')

        # Get button action and corresponding button from parameter server
        self.activate_arm1_button = rospy.get_param('activate_arm1_button')
        self.activate_arm2_button = rospy.get_param('activate_arm2_button')
        self.activate_endef1_button = rospy.get_param('activate_endef1_button')
        self.activate_endef2_button = rospy.get_param('activate_endef2_button')
        self.increase_arm_speed = rospy.get_param('increase_arm_speed')
        self.decrease_arm_speed = rospy.get_param('decrease_arm_speed')
        self.arm_up = rospy.get_param('arm_up')
        self.arm_down = rospy.get_param('arm_down')
        self.arm_x = rospy.get_param('arm_x')
        self.arm_y = rospy.get_param('arm_y')
        self.endef_up = rospy.get_param('endef_up')
        self.endef_side = rospy.get_param('endef_side')
        self.frame_change = rospy.get_param('frame_change')
        self.home_button = rospy.get_param('home_button')
        self.do_function_button = rospy.get_param('end_button')
        self.mirror_button = rospy.get_param('mirror_button')
        self.function_button = rospy.get_param('away_button')
        self.safety_stop_button = rospy.get_param('safety_stop_button')

        # Get the restrictions from the parameter server
        self.min_x_arm = rospy.get_param('min_x_arm')
        self.max_x_arm = rospy.get_param('max_x_arm')
        self.min_y_arm = rospy.get_param('min_y_arm')
        self.max_y_arm = rospy.get_param('max_y_arm')
        self.min_z_arm = rospy.get_param('min_z_arm')
        self.max_z_arm = rospy.get_param('max_z_arm')

        self.arm_max_speed = rospy.get_param('arm_max_speed')
        self.arm_min_speed = rospy.get_param('arm_min_speed')

        # Variables for boundaries for the end effector
        self.min_x_end_effector = rospy.get_param('min_x_end_effector')
        self.max_x_end_effector = rospy.get_param('max_x_end_effector')

        # Variables for home position for the arms 
        self.home_position_x = rospy.get_param('home_position_x')
        self.home_position_y = rospy.get_param('home_position_y')
        self.home_position_z = rospy.get_param('home_position_z')

        # Variables for end position for the arms
        self.end_position_x = rospy.get_param('end_position_x')
        self.end_position_y = rospy.get_param('end_position_y')
        self.end_position_z = rospy.get_param('end_position_z')

        # List of positions to move
        self.arm_movement = rospy.get_param('arm_movement')
        self.endeff_movement = rospy.get_param('endeff_movement')

        # Variables for sinusoidal movement
        self.sinus_x_start_end = rospy.get_param('sinus_x_start_end')
        self.sinus_speed_x = rospy.get_param('sinus_speed_x')
        self.sinus_y = rospy.get_param('sinus_y')

        # Variables for cosinusoidal movement
        self.cosinus_x_start_end = rospy.get_param('cosinus_x_start_end')
        self.cosinus_z_start_end = rospy.get_param('cosinus_z_start_end')
        self.cosinus_speed_x = rospy.get_param('cosinus_speed_x')
        self.cosinus_speed_z = rospy.get_param('cosinus_speed_z')
        self.cosinus_y = rospy.get_param('cosinus_y')
        rospy.loginfo('Params loaded')

    # Makes position follow the real values when controller is not in use
    def arm1_pos_callback(self, data):
        self.real_armposition_1 = data.poses[0].position
        if not self.arm1_initiated:
            self.armposition_1 = np.array([self.real_armposition_1.x, self.real_armposition_1.y, self.real_armposition_1.z])

    def arm2_pos_callback(self, data):
        self.real_armposition_1 = data.poses[0].position
        if not self.arm2_initiated:
            self.armposition_2 = np.array([self.real_armposition_1.x, self.real_armposition_1.y, self.real_armposition_1.z])

    def endeff1_pos_callback(self, data):
        if not self.endeffector1_initiated:
            angles = np.degrees(data.data)
            self.end_effector1_angles[1] = 180 - np.clip(angles[0], self.min_x_end_effector,
                                                         self.max_x_end_effector)

    def endeff2_pos_callback(self, data):
        if not self.endeffector2_initiated:
            angles = np.degrees(data.data)
            self.end_effector2_angles[1] = 180 - np.clip(angles[0], self.min_x_end_effector,
                                                         self.max_x_end_effector)

    # Calls safety stop service to stop arms
    def safety_stop(self):
        request = SetBoolRequest()
        request.data = self.safety_stop_
        # Arm 1
        try:
            response = self.safety_stop_service1(request)
            if response.success:
                rospy.loginfo('Safety stop arm1 successfully!')
            else:
                rospy.logwarn('Failed to safety stop arm1.')
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: ' + str(e))

        # Arm 2
        try:
            response = self.safety_stop_service2(request)
            if response.success:
                rospy.loginfo('Safety stop arm2 successfully!')
            else:
                rospy.logwarn('Failed to safety stop arm2.')
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: ' + str(e))


    # Used for checking if the arm has reached the desired position
    def lists_close(self, list1, list2, tolerance=10):
        if abs(list1.x - list2[0]) > tolerance:
            return True
        if abs(list1.y - list2[1]) > tolerance:
            return True
        if abs(list1.z - list2[2]) > tolerance:
            return True
        else:
            return False
        
    # Function for controlling the arms
    def controll_arm(self, pos, mirror=False, left=False):
        # pos[0] # x
        # pos[1] # y
        # pos[2] # z

        # Makes the arm move to a list of positions
        if self.move_arm_from_list:
            if not left:
                # Checks if the arm have moved to all positions
                if self.arm_right_move_index == len(self.arm_movement):
                    self.arm_right_move_index = 0
                    self.move_arm_from_list = False
                    rospy.loginfo("Done moving")
                    return pos
                
                # Itteraes through the list of positions only if the arm is close to the position
                i = self.arm_movement[self.arm_right_move_index]
                if not self.lists_close(self.real_armposition_1, i):
                    self.arm_right_move_index += 1
                
                pos[0] = i[0]
                pos[1] = i[1]
                pos[2] = i[2]
                return pos
            
            if left:
                # Checks if the arm have moved to all positions
                if self.arm_left_move_index == len(self.arm_movement):
                    self.arm_left_move_index = 0
                    self.move_arm_from_list = False
                    rospy.loginfo("Done moving")
                    return pos
                
                # Itteraes through the list of positions only if the arm is close to the position
                i = self.arm_movement[self.arm_left_move_index]
                if not self.lists_close(self.real_armposition_1, i):
                    self.arm_left_move_index += 1

                pos[0] = i[0]
                pos[1] = i[1]
                pos[2] = i[2]
                return pos

        # Makes the arm move in a straight line
        if self.arm_straight_line:
            # Checks if the arm should move forward or backward
            if self.sinus_forward:
                self.sinus_x_number += self.sinus_speed_x
            if not self.sinus_forward:
                self.sinus_x_number -= self.sinus_speed_x

            # Checks if the arm should have reached the end of the movement, if so change direction
            if self.sinus_x_number > self.sinus_x_start_end[1]:
                self.sinus_forward = False

            if self.sinus_x_number < self.sinus_x_start_end[0]:
                self.sinus_forward = True

            pos[0] = self.sinus_x_number
            pos[1] = self.sinus_y
            return pos
        
        # Moves the arm in a co-sinusoidal movement if active
        if self.active_cosinusodal:
            # Checks if the arm should have reached the end of the movement, if so change direction
            if self.cosinus_x_number > self.cosinus_x_start_end[1]:
                self.cosinus_forward = False

            if self.cosinus_x_number < self.cosinus_x_start_end[0]:
                self.cosinus_forward = True

            # Checks if the arm should move forward or backward
            if self.cosinus_forward:
                self.cosinus_x_number += self.cosinus_speed_x

            if not self.cosinus_forward:
                self.cosinus_x_number -= self.cosinus_speed_x
            
            # Normalizes the x value to a value between 0 and 2pi
            x_norm = float(self.cosinus_x_number)/float(self.cosinus_x_start_end[1])*np.pi*2

            # Checks if the arm should move upward or downward
            if self.cosinus_forward:
                self.cosinus_z_number = 200 + 80 * np.sin(x_norm)
            if not self.cosinus_forward:
                self.cosinus_z_number = 200 - 80 * np.sin(x_norm)
        
            pos[0] = self.cosinus_x_number
            pos[1] = self.cosinus_y
            pos[2] = self.cosinus_z_number
            return pos

        # Controls for right arm
        if self.evaluate_button(self.home_button):
            pos[0] = self.home_position_x
            pos[1] = self.home_position_y
            pos[2] = self.home_position_z
            rospy.loginfo('Arm reset ' + ('2' if left else '1'))
            return pos

        # Controls for right arm
        if not left:
            # Check if the global frame is enabled
            if self.global_frame_point:
                # Updates the variables with the values from the controller
                x_nav = self.joy_data.axes[self.axes_mapping[self.arm_x]]
                y_nav = self.joy_data.axes[self.axes_mapping[self.arm_y]]
            else:
                x_nav = -self.joy_data.axes[self.axes_mapping[self.arm_y]]
                y_nav = self.joy_data.axes[self.axes_mapping[self.arm_x]]

        # Controls for left arm
        if left:
            if self.global_frame_point:
                x_nav = -self.joy_data.axes[self.axes_mapping[self.arm_x]]
                y_nav = -self.joy_data.axes[self.axes_mapping[self.arm_y]]
                # Check if mirror is enabled
                if mirror:
                    x_nav = self.joy_data.axes[self.axes_mapping[self.arm_x]]
                    y_nav = self.joy_data.axes[self.axes_mapping[self.arm_y]]
            else:
                x_nav = -self.joy_data.axes[self.axes_mapping[self.arm_y]]
                y_nav = self.joy_data.axes[self.axes_mapping[self.arm_x]]
                if mirror:
                    x_nav = self.joy_data.axes[self.axes_mapping[self.arm_x]]
                    y_nav = self.joy_data.axes[self.axes_mapping[self.arm_y]]

        # Increases the arm movement with the speed and within the boundaries
        pos[0] += x_nav * self.arm_speed_control
        pos[0] = np.clip(pos[0], self.min_x_arm, self.max_x_arm)

        pos[1] += y_nav * self.arm_speed_control
        pos[1] = np.clip(pos[1], self.min_y_arm, self.max_y_arm)

        # Check if the LT, RT buttons are initiated
        if self.T_buttons_initiated_:
            # Uses the LT and RT buttons to control the z-axis linearly
            pos[2] += (1 - self.joy_data.axes[self.axes_mapping[self.arm_up]]
                    ) / 2 * self.arm_speed_control
            pos[2] -= (1 - self.joy_data.axes[self.axes_mapping[self.arm_down]]
                    ) / 2 * self.arm_speed_control
            pos[2] = np.clip(pos[2], self.min_z_arm, self.max_z_arm)

        return pos


    # Function for controlling the end effector
    def controll_endeff(self, angle, mirror=False, left=False):
        
        # Makes the end effector move to a list of positions
        if self.active_end_effector_move:
            if not left:
                self.time_right_move_index += 1
                
                # Iterates through the list of positions only if the time is right
                if self.time_right_move_index > self.time_index:
                    self.end_right_move_index += 1

                    # Checks if the end effector have moved to all positions
                    if self.end_right_move_index == len(self.endeff_movement):
                        self.end_right_move_index = 0
                        self.active_end_effector_move = False
                        rospy.loginfo("Done moving")
                        return angle
                    
                    self.time_right_move_index = 0
                    i = self.endeff_movement[self.end_right_move_index]
                    angle[0] = i[0]
                    angle[1] = i[1]
                    return angle
            
            if left:
                self.time_left_move_index += 1
                # Iterates through the list of positions only if the time is right
                if self.time_left_move_index > self.time_index:
                    self.end_left_move_index += 1
                    self.time_left_move_index = 0

                    # Checks if the end effector have moved to all positions
                    if self.end_left_move_index == len(self.endeff_movement):
                        self.end_left_move_index = 0
                        self.active_end_effector_move = False
                        rospy.loginfo("Done moving")
                        return angle
                    
                    i = self.endeff_movement[self.end_left_move_index]
                    angle[0] = i[0]
                    angle[1] = i[1]
                    return angle

        # Puts the end effector to home position
        if self.evaluate_button(self.home_button):
            angle[0] = 90
            angle[1] = 90
            rospy.loginfo('End effector reset')
            return angle

        # Controls for right end effector
        if not left:
            # Check if the global frame is enabled
            if self.global_frame_point:
                # Updates the variables with the values from the controller
                m1_nav = self.joy_data.axes[self.axes_mapping[self.endef_up]]
                m2_nav = self.joy_data.axes[self.axes_mapping[self.endef_side]]
            else:
                m1_nav = self.joy_data.axes[self.axes_mapping[self.endef_up]]
                m2_nav = self.joy_data.axes[self.axes_mapping[self.endef_side]]

        # Controls for left end effector
        if left:
            if self.global_frame_point:
                m1_nav = self.joy_data.axes[self.axes_mapping[self.endef_up]]
                m2_nav = -self.joy_data.axes[self.axes_mapping[self.endef_side]]
                # Check if mirror is enabled
                if mirror:
                    m1_nav = self.joy_data.axes[self.axes_mapping[self.endef_up]]
                    m2_nav = self.joy_data.axes[self.axes_mapping[self.endef_side]]
            else:
                m1_nav = self.joy_data.axes[self.axes_mapping[self.endef_up]]
                m2_nav = -self.joy_data.axes[self.axes_mapping[self.endef_side]]
                if mirror:
                    m1_nav = self.joy_data.axes[self.axes_mapping[self.endef_up]]
                    m2_nav = self.joy_data.axes[self.axes_mapping[self.endef_side]]

        # Increasing the values within the limits and with the speed
        angle[0] += m1_nav * self.arm_speed_control
        angle[0] = np.clip(angle[0], self.min_x_end_effector, self.max_x_end_effector)

        angle[1] += m2_nav * self.arm_speed_control
        angle[1] = np.clip(angle[1], self.min_x_end_effector, self.max_x_end_effector)

        # Returns the updated variables so it can be stored
        return angle

    # Callback function for the "joy" topic to control hz
    def joy_callback(self, data):
        self.joy_data = data

    # Function for evaluating if a button is pressed or held down
    def evaluate_button(self, button):
        return self.joy_data.buttons[self.button_mapping[button]] == 1 and \
            self.previous_button_pressed[self.button_mapping[button]] != 1
    
    # Sets all functions to false.
    def sett_functions_states_false(self, function_=0):
        if function_==0:
            self.active_end_effector_move = False
            self.arm_straight_line = False
            self.active_cosinusodal = False
            self.move_arm_from_list = False
            rospy.loginfo("Actions disabled")

        if function_==self.arm_straight_line:          
            self.active_cosinusodal = False
            self.move_arm_from_list = False

        if function_==self.active_cosinusodal:          
            self.move_arm_from_list = False
            self.arm_straight_line = False
        
        if function_==self.move_arm_from_list:          
            self.active_cosinusodal = False
            self.arm_straight_line = False

    # Loop that keeps the ros node running
    def run(self):
        # Checks if the joy_data has been received
        while self.joy_data is None:
            pass

        while not rospy.is_shutdown():

            # Bypass that RT and LT starts with 0 as default value and default value changes to 1 when pressed.
            # Check if the RT and LT buttons have been pressed, first then are they in use
            if not self.T_buttons_initiated_ and self.joy_data.axes[2] == 1 and \
                    self.joy_data.axes[5] == 1:
                self.T_buttons_initiated_ = True
                rospy.loginfo("LT and RT are ready to be used")

            # Activates the arms
            if self.evaluate_button(self.activate_arm1_button):
                self.arm1_initiated = not self.arm1_initiated
                rospy.loginfo('Arm 1 ' + ('enabled' if self.arm1_initiated else 'disabled'))

            if self.evaluate_button(self.activate_arm2_button):
                self.arm2_initiated = not self.arm2_initiated
                rospy.loginfo('Arm 2 ' + ('enabled' if self.arm2_initiated else 'disabled'))

            # Activates the end effectors
            if self.evaluate_button(self.activate_endef1_button):
                self.endeffector1_initiated = not self.endeffector1_initiated
                rospy.loginfo('End effector 1 ' + (
                    'enabled' if self.endeffector1_initiated else 'disabled'))

            if self.evaluate_button(self.activate_endef2_button):
                self.endeffector2_initiated = not self.endeffector2_initiated
                rospy.loginfo('End effector 2 ' + (
                    'enabled' if self.endeffector2_initiated else 'disabled'))

            # Activates the global or local frame
            if self.evaluate_button(self.frame_change):
                self.global_frame_point = not self.global_frame_point
                rospy.loginfo(
                    'Global frame ' + ('enabled' if self.global_frame_point else 'disabled'))

            # Activates the mirror movement, so arms move in the same direction
            if self.evaluate_button(self.mirror_button):
                self.mirror_movement = not self.mirror_movement
                rospy.loginfo('Mirror ' + ('enabled' if self.mirror_movement else 'disabled'))

            # Activates the function state of the arms
            if self.evaluate_button(self.function_button):
                self.function_state = (self.function_state + 1) % 7
                if self.function_state == 0:
                    rospy.loginfo("Status: Free")
                elif self.function_state == 1:
                    rospy.loginfo("Status: Calibration")
                elif self.function_state == 2:
                    rospy.loginfo("Status: Arms away")
                elif self.function_state == 3:
                    self.arm_movement = rospy.get_param('/arm_movement')
                    rospy.loginfo("Status: Move to points")
                elif self.function_state == 4:
                    rospy.loginfo("Status: Move sinusodial")
                elif self.function_state == 5:
                    rospy.loginfo("Status: Move cosinusodial")
                elif self.function_state == 6:
                    self.endeff_movement = rospy.get_param('/endeff_movement')
                    rospy.loginfo("Status: Move end_effectors to points")
                else:
                    rospy.logwarn("Unknown status value: {}".format(self.function_state))

            # Activates the functions of the arms from the function state
            if self.evaluate_button(self.do_function_button):
                # Makes the arms calibrate
                if self.function_state == 0:
                     self.sett_functions_states_false()

                if self.function_state == 1:                             
                    # Sets all functions to false before activating new one.
                    self.sett_functions_states_false()

                    if self.arm1_initiated and self.arm2_initiated:
                        rospy.logwarn("Calibrate one arm at a time")
                    elif self.arm1_initiated:
                        self.arm1_calib.publish(True)
                        self.arm1_initiated = False
                        rospy.loginfo('Arm 1 calibrating')
                    elif self.arm2_initiated:
                        self.arm2_calib.publish(True)
                        self.arm2_initiated = False
                        rospy.loginfo('Arm 2 calibrating')

                # Makes the arms go to away position, so to take as little space as possible
                elif self.function_state == 2:
                                    
                    # Sets all functions to false before activating new one.
                    self.sett_functions_states_false()

                    if self.arm1_initiated and self.arm2_initiated:
                        rospy.logwarn("Put one arm away at a time")
                    elif self.arm1_initiated:
                        self.arm1_away_position.publish(True)
                        self.arm1_initiated = False
                        rospy.loginfo('Arm 1 away position')
                    elif self.arm2_initiated:
                        self.arm2_away_position.publish(True)
                        self.arm2_initiated = False
                        rospy.loginfo('Arm 2 away position')

                # Makes the arms move to the points from the list
                elif self.function_state == 3:
                    self.sett_functions_states_false(self.move_arm_from_list)
                    self.move_arm_from_list = not self.move_arm_from_list
                    rospy.loginfo(
                        'Move from list ' + ('enabled' if self.move_arm_from_list else 'disabled'))                

                # Makes the arms move in as straight line back and forth
                elif self.function_state == 4:
                    self.sett_functions_states_false(self.arm_straight_line)
                    self.arm_straight_line = not self.arm_straight_line
                    rospy.loginfo(
                        'Sinusoidal ' + ('enabled' if self.arm_straight_line else 'disabled'))

                # Makes the arms move in a cosinusoidal movement
                elif self.function_state == 5:
                    self.sett_functions_states_false(self.active_cosinusodal)
                    self.active_cosinusodal = not self.active_cosinusodal
                    rospy.loginfo('Co-sinusoidal ' + (
                        'enabled' if self.active_cosinusodal else 'disabled'))
                    
                # Makes the end effectors move to the points from the list
                elif self.function_state == 6:
                    self.active_end_effector_move = not self.active_end_effector_move
                    rospy.loginfo('Endeffector move ' + (
                        'enabled' if self.active_end_effector_move else 'disabled'))
            
                
            # Adjust the speed of the arms and end effectors
            if self.evaluate_button(self.increase_arm_speed):
                self.arm_speed_control += 0.1
                self.arm_speed_control = np.clip(self.arm_speed_control, self.arm_min_speed,
                                                    self.arm_max_speed)
                rospy.loginfo("Arm speed: %s", self.arm_speed_control)

            if self.evaluate_button(self.decrease_arm_speed):
                self.arm_speed_control -= 0.1
                self.arm_speed_control = np.clip(self.arm_speed_control, self.arm_min_speed,
                                                    self.arm_max_speed)
                rospy.loginfo("Arm speed: %s", self.arm_speed_control)

            # Activates the emergency stop 
            if self.joy_data.buttons[self.button_mapping[self.safety_stop_button[0]]] == 1 and \
                    self.joy_data.buttons[self.button_mapping[
                        self.safety_stop_button[1]]] == 1 and not self.L3_R3_button_prev_state:
                self.safety_stop_ = not self.safety_stop_
                self.safety_stop()
                rospy.loginfo('Safety ' + ('enabled' if self.safety_stop_ else 'disabled'))

            # Pushes safety stop only once, when hold down, to make sure safety is not turned on/off multiple times by mistake
            if self.joy_data.buttons[self.button_mapping[self.safety_stop_button[0]]] == 1 and \
                    self.joy_data.buttons[self.button_mapping[self.safety_stop_button[1]]] == 1:
                self.L3_R3_button_prev_state = True
            else:
                self.L3_R3_button_prev_state = False

            # Will not publish data when safety stop is enabled
            if not self.safety_stop_:
                # Controls only the arms that are activated
                if self.arm1_initiated:
                    self.armposition_1 = self.controll_arm(self.armposition_1)

                if self.arm2_initiated:
                    self.armposition_2 = self.controll_arm(self.armposition_2, self.mirror_movement,
                                                            left=True)
                    
                # Controls only the end effectors that are activated
                if self.endeffector1_initiated:
                    self.end_effector1_angles = self.controll_endeff(self.end_effector1_angles)

                if self.endeffector2_initiated:
                    self.end_effector2_angles = self.controll_endeff(self.end_effector2_angles,
                                                                    self.mirror_movement, left=True)

                # Publish only position when arms are activated
                if self.arm1_initiated:
                    joint_state = JointState()
                    joint_state.position = self.armposition_1
                    joint_state.velocity = [0.0]
                    joint_state.effort = [0]
                    self.arm1_position_pub.publish(joint_state)

                if self.arm2_initiated:
                    joint_state = JointState()
                    joint_state.position = self.armposition_2
                    joint_state.velocity = [0.0]
                    joint_state.effort = [0]
                    self.arm2_position_pub.publish(joint_state)

                # Publish only angles to end effector
                array1 = Int32MultiArray()
                array1.data = self.end_effector1_angles
                self.end_effector1_pub.publish(array1)

                array2 = Int32MultiArray()
                array2.data = self.end_effector2_angles
                self.end_effector2_pub.publish(array2)

            # Stores which button is being held down
            self.previous_button_pressed = self.joy_data.buttons
            self.rate.sleep()


if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('teleop_node_arms')
    Teleop_Node = TeleopNode()
    Teleop_Node.run()
