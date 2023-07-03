#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
import cv2

def camera_info_publisher():
    rospy.init_node('camera_info_publisher', anonymous=True)
    pub = rospy.Publisher('camera_info', CameraInfo, queue_size=10)
    rate = rospy.Rate(1)  # Publish at 1 Hz

    while not rospy.is_shutdown():
        cap = cv2.VideoCapture(2)  # Use the first USB camera (change index as needed)
        if not cap.isOpened():
            rospy.logerr('Failed to open the camera.')
            rate.sleep()
            continue

        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = cap.get(cv2.CAP_PROP_FPS)

        camera_info_msg = CameraInfo()
        camera_info_msg.header.stamp = rospy.Time.now()
        camera_info_msg.width = width
        camera_info_msg.height = height
        camera_info_msg.D = fps

        pub.publish(camera_info_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        camera_info_publisher()
    except rospy.ROSInterruptException:
        pass
