#!/usr/bin/env python3

import rospy
from rosgraph_msgs.msg import Clock
from time import sleep

def publish_time():
    rospy.init_node('time_publisher')
    clock_pub = rospy.Publisher('/clock_sync', Clock, queue_size=10)

    sleep(10)
    current_time = rospy.Time.now()
    clock_msg = Clock()
    clock_msg.clock = current_time

    if not rospy.is_shutdown():
        clock_pub.publish(clock_msg)

if __name__ == '__main__':
    try:
        publish_time()
    except rospy.ROSInterruptException:
        pass