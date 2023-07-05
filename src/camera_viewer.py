#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import Joy
from cv_bridge import CvBridge

class CameraDisplayNode:
    def __init__(self):
        self.camera_subscribers = []
        self.images = []
        self.cv_bridge = CvBridge()
        self.index = 0

    def joy_callback(self, joy_msg):
        
        # Check if the joystick button is pressed
        if joy_msg.axes[6] == -1:
            # Increment the camera index
            self.index = (self.index + 1) % len(self.camera_subscribers)

        elif joy_msg.axes[6] == 1:
            # Decrement the camera index
            self.index = (self.index - 1) % len(self.camera_subscribers)

    def camera_callback(self, image, camera_index):
        # Convert ROS Image to OpenCV format
        cv_image = self.cv_bridge.imgmsg_to_cv2(image, "bgr8")

        # Check if the image has valid dimensions
        if cv_image.shape[0] > 0 and cv_image.shape[1] > 0:
            # Store the image for display
            self.images[camera_index] = cv_image

    def display_images(self):
        # Filter out None values (invalid images)
        valid_images = [image for image in self.images if image is not None]

        # Concatenate valid images side by side
        if valid_images:

            # Display the combined image
            if len(valid_images) == 1:
                self.im_tile_resize = valid_images[0]
            else:
                self.im_tile_resize = self.concat_tile_resize([[valid_images[self.index]],
                                        valid_images])
                
            cv2.imshow("Camera Display", self.im_tile_resize)
            cv2.waitKey(1)


        # Create the image by merging the height and width
    def concat_tile_resize(self, im_list_2d, interpolation=cv2.INTER_CUBIC):
        im_list_v = [self.hconcat_resize_min(im_list_h, interpolation=cv2.INTER_CUBIC) for im_list_h in im_list_2d]
        return self.vconcat_resize_min(im_list_v, interpolation=cv2.INTER_CUBIC)
    
        # Resize all images to the same width and concatenate them vertically
    def vconcat_resize_min(self, im_list, interpolation=cv2.INTER_CUBIC):
        w_min = min(im.shape[1] for im in im_list)
        im_list_resize = [cv2.resize(im, (w_min, int(im.shape[0] * w_min / im.shape[1])), interpolation=interpolation)
                        for im in im_list]
        return cv2.vconcat(im_list_resize)
    
        # Resize the images to the same height and concatenate them horizontally
    def hconcat_resize_min(self, im_list, interpolation=cv2.INTER_CUBIC):
        h_min = min(im.shape[0] for im in im_list)
        im_list_resize = [cv2.resize(im, (int(im.shape[1] * h_min / im.shape[0]), h_min), interpolation=interpolation)
                        for im in im_list]
        return cv2.hconcat(im_list_resize)

    def run(self):
        rospy.init_node("camera_viewer_node")

        # Get the camera topics from ROS parameters
        camera_topics = rospy.get_param("~camera_topics", [])

        # Create a controller subscriber
        controller_subscriber = rospy.Subscriber("/joy", Joy, self.joy_callback)

        # Create camera subscribers and image buffers
        for i, topic in enumerate(camera_topics):
            self.images.append(None)
            subscriber = rospy.Subscriber(topic, Image, self.camera_callback, callback_args=i)
            self.camera_subscribers.append(subscriber)

        rate = rospy.Rate(30)  # Display rate in Hz

        while not rospy.is_shutdown():
            key = cv2.waitKey(1) & 0xFF
            if key >= ord('0') and key < ord(str(len(camera_topics))):
                self.index = key - ord('0')
            elif key == ord('q'):
                break

            self.display_images()
            rate.sleep()

        cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        node = CameraDisplayNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
