#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray
import numpy as np


class Path_follower:
    def __init__(self):
        rospy.init_node('path_follower')

        # Create publishers for arm positions
        self.arm1_move_to_position = rospy.Publisher('arm1position', JointState, queue_size=10)
        self.arm2_move_to_position = rospy.Publisher('arm2position', JointState, queue_size=10)

        # Get params
        self.path = rospy.get_param('arm_movement')

        
    def move_arm(self, arm1=False, arm2=False, path=None):
        if path == None:
            self.path = rospy.get_param('arm_movement')
        else:
            self.path = path

        # Checks one position at a time from list
        last_t = 0
        for x, y, z, t in self.path:
            rospy.loginfo(f"Going to point: {[x, y, z]}")

            # Create message to arm
            joint_state = JointState()
            joint_state.position = [x, y, z]
            joint_state.velocity = [0.0]
            joint_state.effort = [0]

            if arm1: self.arm1_move_to_position.publish(joint_state)
            if arm2: self.arm2_move_to_position.publish(joint_state)

            rospy.sleep(t - last_t);    last_t = t


if __name__ == '__main__':
    # Create path
    from create_path import create_path, create_step
    # points = create_path(t_step=0.1, duration=100, freq=0.2, freq_shift=2, phase_shift=np.pi/2)
    points = create_step(t_step=4, duration=100, step_x=100, step_z=100)
    
    # Follow path
    follower = Path_follower()
    follower.move_arm(arm1=True, path=points)
