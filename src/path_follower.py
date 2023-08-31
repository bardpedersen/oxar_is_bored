#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float32, Float32MultiArray
import numpy as np


class Path_follower:
    def __init__(self, node_name='path_follower'):
        rospy.init_node(node_name)

        # Create publishers for arm positions
        self.arm1_move_to_position = rospy.Publisher('arm1position', JointState, queue_size=10)
        self.arm2_move_to_position = rospy.Publisher('arm2position', JointState, queue_size=10)

        # Publishers for errors
        self.arm1_error = rospy.Publisher('arm1_error', Float32MultiArray, queue_size=10)
        self.arm2_error = rospy.Publisher('arm2_error', Float32MultiArray, queue_size=10)

        # Subscribe to arm positions
        rospy.Subscriber('arm1_cur_pos', PoseArray, self.arm1_pos_callback)
        rospy.Subscriber('arm2_cur_pos', PoseArray, self.arm2_pos_callback)


    def arm1_pos_callback(self, msg):
        self.arm1_pos = msg.poses[0].position

    def arm2_pos_callback(self, msg):
        self.arm2_pos = msg.poses[0].position

    def publish_error(self, ref, arm1=False, arm2=False):
        if arm1:
            # Calculate error
            error_x = ref[0] - self.arm1_pos.x
            error_y = ref[1] - self.arm1_pos.y
            error_z = ref[2] - self.arm1_pos.z
            # Create message
            msg = Float32MultiArray()
            msg.data = [error_x, error_y, error_z]
            # Publish message
            self.arm1_error.publish(msg)
        if arm2:
            # Calculate error
            error_x = ref[0] - self.arm2_pos.x
            error_y = ref[1] - self.arm2_pos.y
            error_z = ref[2] - self.arm2_pos.z
            # Create message
            msg = Float32MultiArray()
            msg.data = [error_x, error_y, error_z]
            # Publish message
            self.arm2_error.publish(msg)


    def move_arm(self, path, arm1=False, arm2=False):
        rate = rospy.Rate(100)
        start_time = rospy.Time.now()

        for x, y, z, t in path:
            t = start_time + rospy.Duration(t)
            rospy.loginfo(f"Going to point: {[x, y, z]}")

            # Create message to arm
            joint_state = JointState()
            joint_state.position = [x, y, z]
            joint_state.velocity = [0.0]
            joint_state.effort = [0]

            while t > rospy.Time.now() and not rospy.is_shutdown():
                if arm1: self.arm1_move_to_position.publish(joint_state)
                if arm2: self.arm2_move_to_position.publish(joint_state)
                try:    self.publish_error([x, y, z], arm1=arm1, arm2=arm2)
                except: pass
                rate.sleep()
            

        rospy.loginfo(f'Completed in {rospy.Time.now().to_sec()-start_time.to_sec():.2f}')
            


if __name__ == '__main__':
    # Create path
    from create_path import create_path, create_step, write_NM, write_BU
    points = create_path(t_step=0.1, duration=60, freq=0.5, freq_shift=2, phase_shift=np.pi/2)
    # points = create_step(t_step=4, duration=60, step_x=100, step_z=100)
    follower = Path_follower('both follow')
    follower.move_arm(arm1=True, arm2=True, path=points)

    # Follow path
    # follower = Path_follower('arm1_follow')
    # while not rospy.is_shutdown():
    #     follower.move_arm(arm1=True, path=write_NM(width=200, height=150, t_step=1))

    # follower = Path_follower('arm2_follow')
    # while not rospy.is_shutdown():
    #     follower.move_arm(arm2=True, path=write_BU(width=150, height=150, t_step=0.5))
