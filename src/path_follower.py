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

    def move_arm(self, path, arm1=False, arm2=False):

        # Checks one position at a time from list
        rate = rospy.Rate(100)
        start_time = rospy.Time.now()
        for x, y, z, t in path:
            t = start_time + rospy.Duration(t)
            # rospy.loginfo(f"Going to point: {[x, y, z]}")

            # Create message to arm
            joint_state = JointState()
            joint_state.position = [x, y, z]
            joint_state.velocity = [0.0]
            joint_state.effort = [0]

            while t > rospy.Time.now() and not rospy.is_shutdown():
                if arm1: self.arm1_move_to_position.publish(joint_state)
                if arm2: self.arm2_move_to_position.publish(joint_state)
                rate.sleep()

        rospy.loginfo(f'Completed in {rospy.Time.now().to_sec()-start_time.to_sec():.2f}')
            


if __name__ == '__main__':
    # Create path
    from create_path import create_path, create_step
    points = create_path(t_step=0.1, duration=100, freq=0.5, freq_shift=3/2, phase_shift=0)
    # points = create_step(t_step=4, duration=30, step_x=100, step_z=100)
    
    # Follow path
    follower = Path_follower()
    follower.move_arm(arm1=True, arm2=True, path=points)
