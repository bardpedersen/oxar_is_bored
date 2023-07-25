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

        # Get the button and axes mapping from the parameter server
        self.button_mapping = rospy.get_param('button_map')
        self.axes_mapping = rospy.get_param('axes_map')

        # Get button action and correspondoing button from paramter server
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
        self.end_button = rospy.get_param('end_button')
        self.mirror_button = rospy.get_param('mirror_button')
        self.away_button_ = rospy.get_param('away_button')
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
                
        # Variables for boundries for the end effector
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

        # Simple varibles for checkings statements
        self.T_buttons_initiated_ = False
        self.arm1_initiated = False
        self.arm2_initiated = False
        self.endeffector1_initiated = False
        self.endeffector2_initiated = False
        self.global_frame_point = True
        self.L3_R3_button_prev_state = False
        self.mirror = False
        self.safety_stop_= False
        self.away_button = False
        self.previous_button_pressed = [0] * len(self.button_mapping)

        # Variables for storing arm positions
        self.position_x_1 = 0
        self.position_y_1 = 200
        self.position_z_1 = 300
        
        self.position_x_2 = 0
        self.position_y_2 = 200
        self.position_z_2 = 300

        # Variables for speed control for the arms
        self.arm_speed_control = 2

        # Variables for storing end effector positions
        self.end_effector1_M1 = 90
        self.end_effector1_M2 = 90

        self.end_effector2_M1 = 90
        self.end_effector2_M2 = 90

        # Set the ros rate to be the same as the arms
        self.rate = rospy.Rate(40)
        self.joy_data = 0

        # Create publisher for the arm controller topic
        self.arm_posit_pub1 = rospy.Publisher('arm1position', JointState, queue_size=10)
        self.arm_posit_pub2 = rospy.Publisher('arm2position', JointState, queue_size=10)

        self.arm_away_position1 = rospy.Publisher('arm1_calib_stretched_cmd', Bool, queue_size=10)
        self.arm_away_position2 = rospy.Publisher('arm2_calib_stretched_cmd', Bool, queue_size=10)

        # Create publisher for the end effector controller topic
        self.end_effector_pub1 = rospy.Publisher('endeffector1', Int32MultiArray, queue_size=10)
        self.end_effector_pub2 = rospy.Publisher('endeffector2', Int32MultiArray, queue_size=10)

        # Create a service proxy for the "safety_stop" service
        self.safety_stop_service1 = rospy.ServiceProxy('safety_stop_arm1', SetBool)
        self.safety_stop_service2 = rospy.ServiceProxy('safety_stop_arm2', SetBool)

        # Subscribe to actual values
        rospy.Subscriber('arm1_cur_pos', PoseArray, self.arm_pos1_callback)
        rospy.Subscriber('arm2_cur_pos', PoseArray, self.arm_pos2_callback)

        rospy.Subscriber('arm1_angle', Float32MultiArray, self.arm_endef_angle1)
        rospy.Subscriber('arm2_angle', Float32MultiArray, self.arm_endef_angle2)

        # Create a subscriber to the "joy" topic with the function "joy_callback" as a callback
        rospy.Subscriber('joy_arms', Joy, self.joy_callback)

        rospy.loginfo('Teleop_node started')
        rospy.loginfo('Press LB and RB to enable the arms')
        rospy.loginfo('Press X and Y to enable the end effectors')
        rospy.loginfo('Press L3 and R3 to enable safety stop')


    # Makes position follow the real values when controller is not in use
    def arm_pos1_callback(self, data):
        if not self.arm1_initiated or self.safety_stop_:
            for pose in data.poses:
                self.position_x_1 = pose.position.x
                self.position_y_1 = pose.position.y
                self.position_z_1 = pose.position.z

    def arm_pos2_callback(self, data):
        if not self.arm2_initiated or self.safety_stop_:
            for pose in data.poses:
                self.position_x_2 = pose.position.x
                self.position_y_2 = pose.position.y
                self.position_z_2 = pose.position.z    

    def arm_endef_angle1(self, data):
        if not self.endeffector1_initiated:
            self.end_effector1_M2 = 180 - min(max(np.degrees(data.data[0]), self.min_x_end_effector), self.max_x_end_effector)


    def arm_endef_angle2(self, data):
        if not self.endeffector2_initiated:
            self.end_effector2_M2 = 180 - min(max(np.degrees(data.data[0]), self.min_x_end_effector), self.max_x_end_effector)

    """  Problems with arduino """
    """
    def endef_away1(self):
        n = self.end_effector1_M1
        while n > 0:
            array1 = Int32MultiArray()
            array1.data = [int(n), int(n)]
            self.end_effector_pub1.publish(array1)
            n-=1
    
    def endef_away2(self):
        n = self.end_effector2_M1
        while n > 0:
            array2 = Int32MultiArray()
            array2.data = [int(n), int(n)]
            self.end_effector_pub2.publish(array2)
            n-=1
    """

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


    # Function for controlling the arms
    def controll_arm(self, pos_x, pos_y, pos_z, mirror=False, left=False):   

        # Reset the arm to home position
        if self.evaluate_button(self.home_button):
            pos_x = self.home_position_x
            pos_y = self.home_position_y
            pos_z = self.home_position_z
            rospy.loginfo('Arm reset')
            return pos_x, pos_y, pos_z

        # Reset the arm to end position
        if self.evaluate_button(self.end_button):
            if not self.away_button:
                pos_x = self.end_position_x
                pos_y = self.end_position_y
                pos_z = self.end_position_z
                rospy.loginfo('Arm ready position')
                return pos_x, pos_y, pos_z
            
        # Controlls for right arm
        if not left:
            # Check if the global frame is enabled
            if self.global_frame_point:
                x_nav = self.joy_data.axes[self.axes_mapping[self.arm_x]]
                y_nav = self.joy_data.axes[self.axes_mapping[self.arm_y]]
            else:
                x_nav = -self.joy_data.axes[self.axes_mapping[self.arm_y]]
                y_nav = self.joy_data.axes[self.axes_mapping[self.arm_x]]

        # Controlls for left arm
        if left:
            if self.global_frame_point:
                x_nav = -self.joy_data.axes[self.axes_mapping[self.arm_x]]
                y_nav = -self.joy_data.axes[self.axes_mapping[self.arm_y]]
                # Ceck if mirror is enabled
                if mirror: 
                    x_nav = self.joy_data.axes[self.axes_mapping[self.arm_x]]  
                    y_nav = self.joy_data.axes[self.axes_mapping[self.arm_y]]
            else:
                x_nav = -self.joy_data.axes[self.axes_mapping[self.arm_y]]
                y_nav = self.joy_data.axes[self.axes_mapping[self.arm_x]]
                if mirror:     
                    x_nav = self.joy_data.axes[self.axes_mapping[self.arm_x]] 
                    y_nav = self.joy_data.axes[self.axes_mapping[self.arm_y]]

        # Increeses the arm movement with the speed and whitin the boundries  
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

        # Returns the updated varibals so it can be stored
        return pos_x, pos_y, pos_z

    # Function for controlling the end effector
    def end_effector(self, M1, M2, mirror=False, left=False):
        # Reset the end effector to home position
        if self.evaluate_button(self.home_button):
            M1 = 90
            M2 = 90
            rospy.loginfo('End effector reset')
            return M1, M2 
        
        """
        if self.evaluate_button(self.end_button):
            if self.away_button:
                if left:
                    self.endef_away2()
                else:
                    self.endef_away1()
                rospy.loginfo('End effector awa position')
                return M1, M2
        """
        
        # Controlls for right end effector
        if not left:
            if self.global_frame_point:
                m1_nav = self.joy_data.axes[self.axes_mapping[self.endef_up]]
                m2_nav = self.joy_data.axes[self.axes_mapping[self.endef_side]]
            else:
                m1_nav = self.joy_data.axes[self.axes_mapping[self.endef_up]]
                m2_nav = self.joy_data.axes[self.axes_mapping[self.endef_side]]

        # Controlls for left end effector
        if left:
            if self.global_frame_point:
                m1_nav = self.joy_data.axes[self.axes_mapping[self.endef_up]]
                m2_nav = -self.joy_data.axes[self.axes_mapping[self.endef_side]]
                if mirror: 
                    m1_nav = self.joy_data.axes[self.axes_mapping[self.endef_up]]
                    m2_nav = self.joy_data.axes[self.axes_mapping[self.endef_side]]
            else:
                m1_nav = self.joy_data.axes[self.axes_mapping[self.endef_up]]
                m2_nav = -self.joy_data.axes[self.axes_mapping[self.endef_side]]
                if mirror:
                    m1_nav = self.joy_data.axes[self.axes_mapping[self.endef_up]]
                    m2_nav = self.joy_data.axes[self.axes_mapping[self.endef_side]]

        # Increasing the values within the limits
        M1 += m1_nav * self.arm_speed_control
        M1 = min(max(M1, self.min_x_end_effector), self.max_x_end_effector)

        M2 += m2_nav * self.arm_speed_control
        M2 = min(max(M2, self.min_x_end_effector), self.max_x_end_effector)

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

                if self.evaluate_button(self.activate_arm1_button):
                    self.arm1_initiated = not self.arm1_initiated 
                    if self.arm1_initiated:
                        rospy.loginfo('Arm 1 enabled')
                    elif not self.arm1_initiated:
                        rospy.loginfo('Arm 1 dissabled')

                if self.evaluate_button(self.activate_arm2_button):
                    self.arm2_initiated = not self.arm2_initiated 
                    if self.arm2_initiated:
                        rospy.loginfo('Arm 2 enabled')
                    elif not self.arm2_initiated:
                        rospy.loginfo('Arm 2 dissabled')

                if self.evaluate_button(self.activate_endef1_button):
                    self.endeffector1_initiated = not self.endeffector1_initiated 
                    if self.endeffector1_initiated:
                        rospy.loginfo('End effector 1 enabled')
                    elif not self.endeffector1_initiated:
                        rospy.loginfo('End effector 1 dissabled')

                if self.evaluate_button(self.activate_endef2_button):
                    self.endeffector2_initiated = not self.endeffector2_initiated 
                    if self.endeffector2_initiated:
                        rospy.loginfo('End effector 2 enabled')
                    elif not self.endeffector2_initiated:
                        rospy.loginfo('End effector 2 dissabled')

                if self.evaluate_button(self.frame_change):
                    self.global_frame_point = not self.global_frame_point 
                    if self.global_frame_point:
                        rospy.loginfo('Global frame enabled')
                    elif not self.global_frame_point:
                        rospy.loginfo('Arm frame enabled')

                if self.evaluate_button(self.mirror_button):
                    self.mirror = not self.mirror
                    if self.mirror:
                        rospy.loginfo('Mirror enabled')
                    elif not self.mirror:
                        rospy.loginfo('Mirror disabled')

                if self.evaluate_button(self.away_button_):
                    self.away_button = not self.away_button
                    if self.away_button:
                        rospy.loginfo('Away button enabled')
                    elif not self.away_button:
                        rospy.loginfo('Ready button enabled')
                
                """ Not workin propperly
                if self.evaluate_button(self.end_button):
                    if self.away_button:
                        # Publish to tuck arm  away
                        self.arm_away_position1.publish(True)
                        self.arm_away_position2.publish(True)
                        rospy.loginfo('Arm away position')

                """
                    

                # Adjust the speed of the arms and end effectors
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

                if not self.safety_stop_:
                    # Will not publish data when safety stop is enabled
                    # Controlles only the arms that are activated
                                    
                    if self.arm1_initiated:
                        self.position_x_1, self.position_y_1, self.position_z_1 = self.controll_arm(self.position_x_1, self.position_y_1, self.position_z_1)

                    if self.arm2_initiated:
                        self.position_x_2, self.position_y_2, self.position_z_2 = self.controll_arm(self.position_x_2, self.position_y_2, self.position_z_2, self.mirror, left=True)

                    if self.endeffector1_initiated:
                        self.end_effector1_M1, self.end_effector1_M2 = self.end_effector(self.end_effector1_M1, self.end_effector1_M2)

                    if self.endeffector2_initiated:
                        self.end_effector2_M1, self.end_effector2_M2 = self.end_effector(self.end_effector2_M1, self.end_effector2_M2, self.mirror, left=True)

                # Publish only position when controller is used
                if self.arm1_initiated and not self.away_button:
                    joint_state = JointState()
                    joint_state.position = [self.position_x_1, self.position_y_1, self.position_z_1]
                    joint_state.velocity = [0.0]
                    joint_state.effort = [0]
                    self.arm_posit_pub1.publish(joint_state)

                if self.arm2_initiated and not self.away_button:
                    joint_state = JointState()
                    joint_state.position = [self.position_x_2, self.position_y_2, self.position_z_2]
                    joint_state.velocity = [0.0]
                    joint_state.effort = [0]
                    self.arm_posit_pub2.publish(joint_state)

                array1 = Int32MultiArray()
                array1.data = [int(self.end_effector1_M1), int(self.end_effector1_M2)]
                self.end_effector_pub1.publish(array1)

                array2 = Int32MultiArray()
                array2.data = [int(self.end_effector2_M1), int(self.end_effector2_M2)]
                self.end_effector_pub2.publish(array2)

                # stores wich button is being held down
                self.previous_button_pressed = self.joy_data.buttons
            self.rate.sleep()


if __name__ == '__main__':
    # Initiate node
    # Initialize the ROS node
    rospy.init_node('teleop_node_arms')
    Teleop_Node = TeleopNode()

    # Keep script running
    Teleop_Node.run()

"""
Add a record function. Press a button and it records its movement untill the button is pressed again. 
When annother button is pressed it will move from the recording.
"""