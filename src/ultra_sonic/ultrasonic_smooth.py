#!/usr/bin/env python3

import rospy
from std_msgs.msg import UInt8


class UltrasonicSmoothNode:
    def __init__(self, window_size=5):
        rospy.init_node('ultrasonic_smooth_node', anonymous=True)

        self.window_size = window_size
        self.buffer_l1 = []
        self.buffer_l2 = []

        self.pub_smooth = rospy.Publisher('/ultrasonic_smooth', UInt8, queue_size=10)

        rospy.Subscriber('/ultra_sonic_l1', UInt8, self.ultra_sonic_l1_callback)
        rospy.Subscriber('/ultra_sonic_l2', UInt8, self.ultra_sonic_l2_callback)

    def ultra_sonic_l1_callback(self, data):
        self.buffer_l1.append(data.data)
        if len(self.buffer_l1) > self.window_size:
            self.buffer_l1.pop(0)
        self.publish_smooth_value()

    def ultra_sonic_l2_callback(self, data):
        self.buffer_l2.append(data.data)
        if len(self.buffer_l2) > self.window_size:
            self.buffer_l2.pop(0)
        self.publish_smooth_value()

    def publish_smooth_value(self):
        if len(self.buffer_l1) == self.window_size and len(self.buffer_l2) == self.window_size:
            smooth_value = (sum(self.buffer_l1) / self.window_size + sum(self.buffer_l2) / self.window_size) / 2
            self.pub_smooth.publish(UInt8(int(smooth_value)))
            


if __name__ == '__main__':
    try:
        smooth_ultra_sonic_node = UltrasonicSmoothNode(window_size=5)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
