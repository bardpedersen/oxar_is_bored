#!/usr/bin/env python3

import rospy
import tf
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped


class TFToJointStatePublisher:
    def __init__(self, tf_link_name, tf_link_parent='base_link'):
        self.tf_link_name = tf_link_name
        self.joint_state_pub = rospy.Publisher(tf_link_name, JointState, queue_size=10)
        self.tf_listener = tf.TransformListener()

    def publish_joint_state(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/base_link', self.tf_link_name, rospy.Time(0))

            joint_state_msg = JointState()
            joint_state_msg.header.stamp = rospy.Time.now()
            joint_state_msg.name.append(self.tf_link_name)
            joint_state_msg.position = trans

            self.joint_state_pub.publish(joint_state_msg)

        except:
            pass


def arm1position_publisher(data):
    x = data.position[0]/1000 - abs(0.15011 - 0.10123)
    y = data.position[1]/1000 - abs(0.35032 - 0.3585)
    z = -data.position[2]/1000 + abs(1.006280 - 1.2543)

    broadcaster.sendTransform(
        (x, y, z),
        (0.0, 0.0, 0.0, 1.0),
        rospy.Time.now(),
        'arm1_reference_link',
        'arm1_base_link'
    )

def arm2position_publisher(data):
    x = data.position[0]/1000 + abs(0.13749 - 0.11897)
    y = data.position[1]/1000 - abs(0.35032 - 0.3585)
    z = -data.position[2]/1000 + abs(1.006280 - 1.2543)

    broadcaster.sendTransform(
        (x, y, z),
        (0.0, 0.0, 0.0, 1.0),
        rospy.Time.now(),
        'arm2_reference_link',
        'arm2_base_link'
    )


# Launch node
rospy.init_node('target_publisher')

# Publish reference
broadcaster = tf.TransformBroadcaster()
arm1_subscriber = rospy.Subscriber('arm1position', JointState, arm1position_publisher)
arm1_subscriber = rospy.Subscriber('arm2position', JointState, arm2position_publisher)

# Publish TF as topics
arm1_reference_link = TFToJointStatePublisher('arm1_reference_link', tf_link_parent='arm1_base_link')
arm1_pan_link = TFToJointStatePublisher('arm1_pan_link', tf_link_parent='arm1_base_link')
arm2_reference_link = TFToJointStatePublisher('arm2_reference_link', tf_link_parent='arm2_base_link')
arm2_pan_link = TFToJointStatePublisher('arm2_pan_link', tf_link_parent='arm2_base_link')

while not rospy.is_shutdown():
    arm1_reference_link.publish_joint_state()
    arm1_pan_link.publish_joint_state()
    arm2_reference_link.publish_joint_state()
    arm2_pan_link.publish_joint_state()

rospy.spin()