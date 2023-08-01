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
        self.load_params()

        # Initialize variables for storing positions and states
        self.armposition_1 = np.array([0, 200, 300])
        self.armposition_2 = np.array([0, 200, 300])
        self.end_effector1_angles = np.array([90, 90])
        self.end_effector2_angles = np.array([90, 90])

        self.T_buttons_initiated_ = False
        self.arm1_initiated = False
        self.arm2_initiated = False
        self.endeffector1_initiated = False
        self.endeffector2_initiated = False
        self.global_frame_point = True
        self.L3_R3_button_prev_state = False
        self.mirror = False
        self.safety_stop_= False
        self.active_sinusodal = False
        self.active_cosinusodal = False
        self.sinus_forward = True
        self.cosinus_forward = True
        self.cosinus_upward = True
        self.sinus_x_number = 0
        self.cosinus_x_number = 0
        self.cosinus_z_number = 0
        self.away_statement = 0
        self.previous_button_pressed = [0] * len(self.button_mapping)
        self.arm_speed_control = 2
        self.joy_data = None
        self.pose1 = None
        self.pose2 = None

        self.rate = rospy.Rate(60)

        self.arm_posit_pub1 = rospy.Publisher('arm1position', JointState, queue_size=10)
        self.arm_posit_pub2 = rospy.Publisher('arm2position', JointState, queue_size=10)
        self.arm_away_position1 = rospy.Publisher('arm1_calib_stretched_cmd', Bool, queue_size=10)
        self.arm_away_position2 = rospy.Publisher('arm2_calib_stretched_cmd', Bool, queue_size=10)
        self.arm1_calib = rospy.Publisher('arm1_calib_cmd', Bool, queue_size=10)
        self.arm2_calib = rospy.Publisher('arm2_calib_cmd', Bool, queue_size=10)
        self.end_effector_pub1 = rospy.Publisher('endeffector1', Int32MultiArray, queue_size=10)
        self.end_effector_pub2 = rospy.Publisher('endeffector2', Int32MultiArray, queue_size=10)

        self.safety_stop_service1 = rospy.ServiceProxy('safety_stop_arm1', SetBool)
        self.safety_stop_service2 = rospy.ServiceProxy('safety_stop_arm2', SetBool)

        rospy.Subscriber('arm1_cur_pos', PoseArray, self.arm_pos1_callback)
        rospy.Subscriber('arm2_cur_pos', PoseArray, self.arm_pos2_callback)
        rospy.Subscriber('arm1_angle', Float32MultiArray, self.arm_endef_angle1)
        rospy.Subscriber('arm2_angle', Float32MultiArray, self.arm_endef_angle2)
        rospy.Subscriber('joy_arms', Joy, self.joy_callback)

        rospy.loginfo('Teleop_node started')
        rospy.loginfo('Press LB and RB to enable the arms')
        rospy.loginfo('Press X and Y to enable the end effectors')
        rospy.loginfo('Press L3 and R3 to enable safety stop')


    def load_params(self):
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
        self.away_button = rospy.get_param('away_button')
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

        # List of positions to move
        self.arm_movement = rospy.get_param('arm_movement')


    # Makes position follow the real values when controller is not in use
    def arm_pos1_callback(self, data):
        self.pose1 = data.poses[0].position
        if not self.arm1_initiated and not self.safety_stop_ and self.away_statement!=3 and self.away_statement!=4 and self.away_statement!=5:
            self.armposition_1 = np.array([self.pose1.x, self.pose1.y, self.pose1.z])

    def arm_pos2_callback(self, data):
        self.pose2 = data.poses[0].position
        if not self.arm2_initiated and not self.safety_stop_ and self.away_statement!=3 and self.away_statement!=4 and self.away_statement!=5:
            self.armposition_2 = np.array([self.pose2.x, self.pose2.y, self.pose2.z]) 

    def arm_endef_angle1(self, data):
        if not self.endeffector1_initiated:
            angles = np.degrees(data.data)
            self.end_effector1_angles[1] = 180 - np.clip(angles[0], self.min_x_end_effector, self.max_x_end_effector)

    def arm_endef_angle2(self, data):
        if not self.endeffector2_initiated:
            angles = np.degrees(data.data)
            self.end_effector2_angles[1] = 180 - np.clip(angles[0], self.min_x_end_effector, self.max_x_end_effector)


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


    # Function for controlling the arms
    def controll_arm(self, pos, mirror=False, left=False):   
        #pos[0] # x
        #pos[1] # y
        #pos[2] # z

        # Reset the arm to home position
        if self.evaluate_button(self.home_button):
            pos[0] = self.home_position_x
            pos[1] = self.home_position_y
            pos[2] = self.home_position_z
            rospy.loginfo('Arm reset ' + ('2' if left else '1'))
            return pos
            
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
        pos[0] += x_nav * self.arm_speed_control
        pos[0] = np.clip(pos[0], self.min_x_arm, self.max_x_arm)

        pos[1] += y_nav * self.arm_speed_control
        pos[1] = np.clip(pos[1], self.min_y_arm, self.max_y_arm)

        # Check if the LT, RT buttons are initiated
        if self.T_buttons_initiated_:
            # Uses the LT and RT buttons to control the z-axis linearly, more pressed bigger increment
            pos[2] += (1 - self.joy_data.axes[self.axes_mapping[self.arm_up]]) / 2 * self.arm_speed_control
            pos[2] -= (1 - self.joy_data.axes[self.axes_mapping[self.arm_down]]) / 2 * self.arm_speed_control
            pos[2] = np.clip(pos[2], self.min_z_arm, self.max_z_arm)

        # Returns the updated varibals so it can be stored
        return pos
    

    def are_lists_close(self, list1, list2, tolerance=10):
        if abs(list1.x - list2[0]) > tolerance:
            return True
        if abs(list1.y - list2[1]) > tolerance:
            return True
        if abs(list1.z - list2[2]) > tolerance:
            return True
        else:
            return False


    def move_arm(self, left=0): 
        for i in self.arm_movement:
            
            if left==0:
                self.armposition_1[0] = i[0] 
                self.armposition_1[1] = i[1] 
                self.armposition_1[2] = i[2] 

                joint_state = JointState()
                joint_state.position = self.armposition_1
                joint_state.velocity = [0.0]
                joint_state.effort = [0]
                self.arm_posit_pub1.publish(joint_state)
                
                while self.are_lists_close(self.pose1, i):
                    pass
                
            if left==1:
                self.armposition_2[0] = i[0] 
                self.armposition_2[1] = i[1] 
                self.armposition_2[2] = i[2]

                joint_state = JointState()
                joint_state.position = self.armposition_2
                joint_state.velocity = [0.0]
                joint_state.effort = [0]
                self.arm_posit_pub2.publish(joint_state)
                
                while self.are_lists_close(self.pose2, i):
                    pass

            if left==2:
                self.armposition_1[0] = i[0] 
                self.armposition_1[1] = i[1] 
                self.armposition_1[2] = i[2]

                self.armposition_2[0] = i[0] 
                self.armposition_2[1] = i[1] 
                self.armposition_2[2] = i[2]

                joint_state = JointState()
                joint_state.position = self.armposition_1
                joint_state.velocity = [0.0]
                joint_state.effort = [0]
                self.arm_posit_pub1.publish(joint_state)
                

                joint_state = JointState()
                joint_state.position = self.armposition_2
                joint_state.velocity = [0.0]
                joint_state.effort = [0]
                self.arm_posit_pub2.publish(joint_state)
                
                while self.are_lists_close(self.pose2, i) or self.are_lists_close(self.pose1, i):
                    pass

        rospy.loginfo('Done moving')

    def move_arm_sinusodial(self, pos):
        if self.sinus_forward:
            self.sinus_x_number += 1
        if not self.sinus_forward:
            self.sinus_x_number -= 1

        if self.sinus_x_number > 250:
            self.sinus_forward = False

        if self.sinus_x_number < -150:
            self.sinus_forward = True

        pos[0] = self.sinus_x_number
        pos[1] = 200 
        return pos
    
    def move_arm_cosinusodial(self, pos):
        if self.cosinus_forward:
            self.cosinus_x_number += 1
        if not self.cosinus_forward:
            self.cosinus_x_number -= 1

        if self.cosinus_upward:
            self.cosinus_z_number -= 1
        if not self.cosinus_upward:
            self.cosinus_z_number += 1

        if self.cosinus_x_number > 279:
            self.cosinus_forward = False

        if self.cosinus_x_number < -21:
            self.cosinus_forward = True
        
        if self.cosinus_z_number > 399:
            self.cosinus_upward = True

        if self.cosinus_z_number < 99:
            self.cosinus_upward = False

        pos[0] = self.cosinus_x_number
        pos[1] = 300 
        pos[2] = self.cosinus_z_number
        return pos

    # Function for controlling the end effector
    def end_effector(self, angle, mirror=False, left=False):

        # Reset the end effector to home position
        if self.evaluate_button(self.home_button):
            angle[0] = 90
            angle[1] = 90
            rospy.loginfo('End effector reset')
            return angle
        
        if self.evaluate_button(self.end_button):
            angle[0] = 0
            angle[1] = 0  
            rospy.loginfo('End effector away')
            return angle              

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
        angle[0] += m1_nav * self.arm_speed_control
        angle[0] = np.clip(angle[0], self.min_x_end_effector, self.max_x_end_effector)

        angle[1] += m2_nav * self.arm_speed_control
        angle[1] = np.clip(angle[1], self.min_x_end_effector, self.max_x_end_effector)

        # Returns the updated varibals so it can be stored
        return angle


    # Callback function for the "joy" topic
    def joy_callback(self, data):
        self.joy_data = data


    def evaluate_button(self, button):
        return self.joy_data.buttons[self.button_mapping[button]] == 1 and self.previous_button_pressed[self.button_mapping[button]] != 1


    # Loop that keeps the ros node running
    def run(self):
        while not rospy.is_shutdown():
            # Checks if the joy_data has been recived
            if self.joy_data is not None:
                # Bypass that RT and LT starts with 0 as default value and default value changes to 1 when pressed.
                # Check if the RT and LT buttons have been pressed, first then are they in use
                if not self.T_buttons_initiated_ and self.joy_data.axes[2] == 1 and self.joy_data.axes[5] == 1:
                    self.T_buttons_initiated_ = True
                    rospy.loginfo("LT and RT are ready to be used")

                if self.evaluate_button(self.activate_arm1_button):
                    self.arm1_initiated = not self.arm1_initiated 
                    rospy.loginfo('Arm 1 ' + ('enabled' if self.arm1_initiated else 'disabled'))

                if self.evaluate_button(self.activate_arm2_button):
                    self.arm2_initiated = not self.arm2_initiated 
                    rospy.loginfo('Arm 2 ' + ('enabled' if self.arm2_initiated else 'disabled'))

                if self.evaluate_button(self.activate_endef1_button):
                    self.endeffector1_initiated = not self.endeffector1_initiated 
                    rospy.loginfo('End effector 1 ' + ('enabled' if self.endeffector1_initiated else 'disabled'))

                if self.evaluate_button(self.activate_endef2_button):
                    self.endeffector2_initiated = not self.endeffector2_initiated 
                    rospy.loginfo('End effector 2 ' + ('enabled' if self.endeffector2_initiated else 'disabled'))

                if self.evaluate_button(self.frame_change):
                    self.global_frame_point = not self.global_frame_point 
                    rospy.loginfo('Global frame ' + ('enabled' if self.global_frame_point else 'disabled'))

                if self.evaluate_button(self.mirror_button):
                    self.mirror = not self.mirror
                    rospy.loginfo('Mirror ' + ('enabled' if self.mirror else 'disabled'))

                if self.evaluate_button(self.away_button):
                    self.away_statement = (self.away_statement + 1) % 6
                    if self.away_statement == 0:
                        rospy.loginfo("Status: Free")
                    elif self.away_statement == 1:
                        rospy.loginfo("Status: Calibration")
                    elif self.away_statement == 2:
                        rospy.loginfo("Status: Away")
                    elif self.away_statement == 3:
                        rospy.loginfo("Status: Move preset")
                    elif self.away_statement == 4:
                        rospy.loginfo("Status: Move sinusodial")
                    elif self.away_statement == 5:
                        rospy.loginfo("Status: Move cosinusodial")
                    else:
                        rospy.logwarn("Unknown status value: {}".format(self.away_statement))

                if self.evaluate_button(self.end_button):
                    if self.away_statement==1:
                        if self.arm1_initiated and self.arm2_initiated:
                            rospy.loginfo("Calibrate one arm at a time")
                        elif self.arm1_initiated:
                            self.arm1_calib.publish(True)
                            rospy.loginfo('Arm 1 calibrating')
                        elif self.arm2_initiated:
                            self.arm2_calib.publish(True)
                            rospy.loginfo('Arm 2 calibrating')

                    elif self.away_statement==2:
                        if self.arm1_initiated and self.arm2_initiated:
                            rospy.loginfo("Put one arm away at a time")
                        elif self.arm1_initiated:
                            self.arm_away_position1.publish(True)
                            rospy.loginfo('Arm 1 away position')
                        elif self.arm2_initiated:
                            self.arm_away_position2.publish(True)
                            rospy.loginfo('Arm 2 away position')

                    elif self.away_statement==3: 
                        if self.arm1_initiated and self.arm2_initiated:
                            rospy.loginfo('Arm 1 and 2 prevmove')
                            self.move_arm(left=2)                           
                        elif self.arm1_initiated:
                            rospy.loginfo('Arm 1 prevmove')
                            self.move_arm()
                        elif self.arm2_initiated:
                            rospy.loginfo('Arm 2 prevmove')
                            self.move_arm(left=1)
                    
                    elif self.away_statement==4:
                        self.active_sinusodal = not self.active_sinusodal
                        rospy.loginfo('Sinusodial ' + ('enabled' if self.active_sinusodal else 'disabled'))

                    elif self.away_statement==5:
                        self.active_cosinusodal = not self.active_cosinusodal
                        rospy.loginfo('Cosinusodial ' + ('enabled' if self.active_cosinusodal else 'disabled'))

                                 
                # Adjust the speed of the arms and end effectors
                if self.evaluate_button(self.increase_arm_speed):
                    self.arm_speed_control += 0.1
                    self.arm_speed_control = np.clip(self.arm_speed_control, self.arm_min_speed, self.arm_max_speed)
                    rospy.loginfo("Arm speed: %s", self.arm_speed_control)

                if self.evaluate_button(self.decrease_arm_speed):
                    self.arm_speed_control -= 0.1
                    self.arm_speed_control = np.clip(self.arm_speed_control, self.arm_min_speed, self.arm_max_speed)
                    rospy.loginfo("Arm speed: %s", self.arm_speed_control)

                # Activates the emergency stop 
                if self.joy_data.buttons[self.button_mapping[self.safety_stop_button[0]]] == 1 and self.joy_data.buttons[self.button_mapping[self.safety_stop_button[1]]] == 1 and not self.L3_R3_button_prev_state:
                    self.safety_stop_ = not self.safety_stop_
                    self.safety_stop()
                    rospy.loginfo('Safety ' + ('enabled' if self.safety_stop_ else 'disabled'))
                        
                if self.joy_data.buttons[self.button_mapping[self.safety_stop_button[0]]] == 1 and self.joy_data.buttons[self.button_mapping[self.safety_stop_button[1]]] == 1:
                    self.L3_R3_button_prev_state = True
                else:
                    self.L3_R3_button_prev_state = False

                if not self.safety_stop_:
                    # Will not publish data when safety stop is enabled
                    # Controlles only the arms that are activated
                                    
                    if self.arm1_initiated and self.away_statement==0:
                        self.armposition_1 = self.controll_arm(self.armposition_1)

                    if self.arm2_initiated and self.away_statement==0:
                        self.armposition_2 = self.controll_arm(self.armposition_2, self.mirror, left=True)

                    if self.endeffector1_initiated:
                        self.end_effector1_angles = self.end_effector(self.end_effector1_angles)

                    if self.endeffector2_initiated:
                        self.end_effector2_angles = self.end_effector(self.end_effector2_angles, self.mirror, left=True)

                    if self.arm1_initiated and self.active_sinusodal:
                        self.armposition_1 = self.move_arm_sinusodial(self.armposition_1)
                    
                    if self.arm2_initiated and self.active_sinusodal:
                        self.armposition_2 = self.move_arm_sinusodial(self.armposition_2)

                    if self.arm1_initiated and self.active_cosinusodal:
                        self.armposition_1 = self.move_arm_cosinusodial(self.armposition_1)
                    
                    if self.arm2_initiated and self.active_cosinusodal:
                        self.armposition_2 = self.move_arm_cosinusodial(self.armposition_2)

                    # Publish only position when controller is used
                    if self.arm1_initiated and (self.away_statement==0 or self.away_statement==4 or self.away_statement==5):
                        joint_state = JointState()
                        joint_state.position = self.armposition_1
                        joint_state.velocity = [0.0]
                        joint_state.effort = [0]
                        self.arm_posit_pub1.publish(joint_state)

                    if self.arm2_initiated and (self.away_statement==0 or self.away_statement==4 or self.away_statement==5):
                        joint_state = JointState()
                        joint_state.position = self.armposition_2
                        joint_state.velocity = [0.0]
                        joint_state.effort = [0]
                        self.arm_posit_pub2.publish(joint_state)

                    array1 = Int32MultiArray()
                    array1.data = self.end_effector1_angles
                    self.end_effector_pub1.publish(array1)

                    array2 = Int32MultiArray()
                    array2.data = self.end_effector2_angles
                    self.end_effector_pub2.publish(array2)

                # stores wich button is being held down
                self.previous_button_pressed = self.joy_data.buttons
            self.rate.sleep()


if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('teleop_node_arms')
    Teleop_Node = TeleopNode()
    Teleop_Node.run()

"""
Add a record function. Press a button and it records its movement untill the button is pressed again. 
When annother button is pressed it will move from the recording.

Arms are lagging
Can co both sinus and cosin at same time. remove.
Endeffector error if moved to fast
Safety always wokring?
Calibration at the same time, error
"""