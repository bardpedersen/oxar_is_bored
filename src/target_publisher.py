#!/usr/bin/env python3

import rospy
import tf
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped


def callback(data):
    position = data.position
    print(position)

    # Create a TransformStamped message
    transform = TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = 'base_link'
    transform.child_frame_id = 'target_link'
    transform.transform.translation.x = position[0]/1000
    transform.transform.translation.y = position[1]/1000
    transform.transform.translation.z = position[2]/1000
    transform.transform.rotation.x = 0.0
    transform.transform.rotation.y = 0.0
    transform.transform.rotation.z = 0.0
    transform.transform.rotation.w = 1.0

    # Publish the transform
    broadcaster.sendTransformMessage(transform)


# Launch node
rospy.init_node('target_publisher')
broadcaster = tf.TransformBroadcaster()
subscriber = rospy.Subscriber('arm1position', JointState, callback)
rospy.spin()