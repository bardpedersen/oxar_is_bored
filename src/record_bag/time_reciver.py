#!/usr/bin/env python3

import sys
import rospy
from rosgraph_msgs.msg import Clock

class RosbagSynchronizer:
    def __init__(self, node_name):

        rospy.init_node(node_name)
        topic_sync = rospy.get_param('~topic_sync', '')
        self.sync_pub = rospy.Publisher(topic_sync, Clock, queue_size=10)
        rospy.Subscriber('/clock_sync', Clock, self.sync_callback)

    def sync_callback(self, msg):
        sync_msg = Clock()
        sync_msg.clock = rospy.Time.now()
        self.sync_pub.publish(sync_msg)

if __name__ == '__main__':
    node_name = sys.argv[1]
    try:
        synchronizer = RosbagSynchronizer(node_name)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
