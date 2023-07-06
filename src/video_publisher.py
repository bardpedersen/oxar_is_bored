#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import Trigger, TriggerResponse


def find_video_channels():
    rospy.wait_for_service('video_finder')
    video_finder = rospy.ServiceProxy('video_finder', Trigger)

    # Call the service
    response = video_finder()

    # Process the response
    if response.success:
        return response.message
    else:
        return "Service call failed!"


class VideoPublisher:
    def __init__(self, topic_name, video_channel):
        self.video_channel = video_channel
        self.topic_name = topic_name
        self.bridge = CvBridge()
        self.publisher = rospy.Publisher(topic_name, Image, queue_size=10)
        self.video_capture = cv2.VideoCapture(self.video_channel)


    def publish_frame(self):
        if self.video_capture.isOpened():
            # Get video frame
            ret, frame = self.video_capture.read()

            # Convert the frame to a ROS image message
            try: 
                ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.publisher.publish(ros_image)
            except:
                pass


if __name__ == '__main__':
    rospy.init_node('video_publisher')
    video_channels = eval(find_video_channels())
    publishers = []
    camera_topics = []

    # Creates video publishers
    for video_channel in video_channels:
        topic_name = f'video_stream_{video_channel}'
        camera_topics.append(topic_name)
        publishers.append(VideoPublisher(topic_name, video_channel))
    
    # Set the parameter for the camera topics
    rospy.set_param('/camera_topics', camera_topics)

    # Running the publishers
    while True:
        for publisher in publishers:
            publisher.publish_frame()
        # rospy.sleep(0.033)  # Adjust the delay based on your desired frame rate
