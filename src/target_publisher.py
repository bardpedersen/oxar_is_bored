#!/usr/bin/env python3

import rospy
import tf
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped


def callback(data):
    x = data.position[0]/1000 - abs(0.15011 - 0.10123)
    y = data.position[1]/1000 - abs(0.35032 - 0.3585)
    z = -data.position[2]/1000 + abs(1.006280 - 1.2543)

    # Create a TransformStamped message
    broadcaster.sendTransform(
        (x, y, z),
        (0.0, 0.0, 0.0, 1.0),
        rospy.Time.now(),
        'arm1_reference_link',
        'arm1_base_link'
    )


# Launch node
rospy.init_node('target_publisher')
broadcaster = tf.TransformBroadcaster()
subscriber = rospy.Subscriber('arm1position', JointState, callback)
rospy.spin()