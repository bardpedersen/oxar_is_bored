#!/usr/bin/env python3

import rospy
import tf
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped


def callback(data):
    position = data.position
    print(position)

    # Create a TransformStamped message
    broadcaster.sendTransform(
        (position[0]/1000, position[1]/1000, -position[2]/1000),
        (0.0, 0.0, 0.0, 1.0),
        rospy.Time.now(),
        'target_link',
        'arm1_base_link'
    )


# Launch node
rospy.init_node('target_publisher')
broadcaster = tf.TransformBroadcaster()
subscriber = rospy.Subscriber('arm1position', JointState, callback)
rospy.spin()