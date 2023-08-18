#!/usr/bin/env python3

import numpy as np
import rospy
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from sensor_msgs.msg import JointState


class ArmToJointState:
    def __init__(self, arm_number=1):
        rospy.init_node('arm_to_joint_state')

        self.joint_state_pub = rospy.Publisher(f'arm_{arm_number}/joint_states', JointState, queue_size=10)
        self.joint_state = JointState()

        # Set the joint names and initial positions
        self.joint_state.name = [
            f'arm_{arm_number}_base_joint',
            f'arm_{arm_number}_joint_1',
            f'arm_{arm_number}_joint_2',
            f'arm_{arm_number}_joint_3',
            f'arm_{arm_number}_joint_4',
            f'arm_{arm_number}_joint_5'
        ]
        self.joint_state.position = [0.0] * len(self.joint_state.name)

        # Subscribe to topics and update joint positions
        rospy.Subscriber(f'arm{arm_number}_angle', Float32MultiArray, self.arm_callback)
        rospy.Subscriber(f'endeffector{arm_number}', Int32MultiArray, self.end_callback)
        rospy.Subscriber(f'arm{arm_number}position', JointState, self.z_callback)

    def arm_callback(self, msg):
        # Update the corresponding joint position
        self.joint_state.position[1] = msg.data[-1] - np.radians(90)
        self.joint_state.position[2] = msg.data[-2] - np.radians(180)

    def end_callback(self, msg):
        # Update the corresponding joint position
        self.joint_state.position[4] = np.radians(msg.data[-1]-90)
        self.joint_state.position[5] = np.radians(msg.data[-2]-90)

    def z_callback(self, msg):
        z = -msg.position[-1] / 1000
        self.joint_state.position[0] = z 


    def publish_joint_state(self):
        rospy.loginfo(self.joint_state)
        # Publish the joint state
        self.joint_state.header.stamp = rospy.Time.now()
        self.joint_state_pub.publish(self.joint_state)


if __name__ == '__main__':
    arm_1 = ArmToJointState(arm_number=1)
    arm_2 = ArmToJointState(arm_number=2)
    rate = rospy.Rate(10)  # Update rate in Hz

    while not rospy.is_shutdown():
        arm_1.publish_joint_state()
        arm_2.publish_joint_state()
        rate.sleep()
