#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
# from sensor_msgs.msg import Joy


# This node subscribes to multiple camera topics and displays the images in a single window
class CameraDisplayNode:
    def __init__(self):
        self.images = []
        self.valid_images = []
        self.cv_bridge = CvBridge()
        self.index = 0
        self.window_name = "Camera Display"
        self.im_tile_resize = None

    # Not needed
    # def joy_callback(self, joy_msg):
    #    # Check if the dpad left and right is pressed
    #    if joy_msg.axes[6] == -1:
    #        # Increment the camera index
    #        self.index = (self.index + 1) % len(self.valid_images)
    #    elif joy_msg.axes[6] == 1:
    #        # Decrement the camera index
    #        self.index = (self.index - 1) % len(self.valid_images)
        
    # Switch between cameras with mouse or touch screen
    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            num_cameras = len(self.valid_images)
            # Calculate the width of each image in the display window
            image_width = int(self.im_tile_resize.shape[1] / num_cameras)
            image_height = int(self.im_tile_resize.shape[0] - image_width*0.75)
            # Determine the index of the clicked image
            clicked_index = x // image_width
            if clicked_index < num_cameras and y > image_height:
                self.index = clicked_index
            rospy.loginfo("Clicked on camera %d", self.index + 1)

    def camera_callback(self, image, camera_index):
        # Convert ROS Image to OpenCV format
        cv_image = self.cv_bridge.imgmsg_to_cv2(image, "bgr8")

        # Check if the image has valid dimensions
        if cv_image.shape[0] > 0 and cv_image.shape[1] > 0:
            # Store the image for display
            self.images[camera_index] = cv_image

    def display_images(self):
        # Filter out None values (invalid images)
        self.valid_images = [image for image in self.images if image is not None]

        # Concatenate valid images side by side
        if self.valid_images:
            # Display the combined image
            if len(self.valid_images) == 1:
                self.im_tile_resize = self.valid_images[0]
            else:
                self.im_tile_resize = self.concat_tile_resize([[self.valid_images[self.index]],
                                                               self.valid_images])

            # Add camera number text to each image 
            num_cameras = len(self.valid_images)
            image_width = int(self.im_tile_resize.shape[1] / num_cameras)
            image_height = int(self.im_tile_resize.shape[0] - image_width*0.75)
            for i in range(num_cameras):
                x = i * image_width + 5
                y = image_height + 25
                text = "Camera {}".format(i + 1)
                cv2.putText(self.im_tile_resize, text, (x, y), cv2.FONT_HERSHEY_COMPLEX,
                            2/num_cameras, (0, 0, 0), int(2/num_cameras))

                if i == self.index:
                    x = 25  # Need to define better
                    y = 75  # Need to define better
                    cv2.putText(self.im_tile_resize, text, (x, y), cv2.FONT_HERSHEY_COMPLEX,
                                2, (255, 255, 255), 1)
            
            # Show the combined image in full screen
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.setWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            cv2.imshow(self.window_name, self.im_tile_resize)
            cv2.setMouseCallback(self.window_name, self.mouse_callback)
            # To switch between cameras with mouse

    # Create the image by merging the height and width
    def concat_tile_resize(self, im_list_2d, interpolation=cv2.INTER_CUBIC):
        im_list_v = [self.hconcat_resize_min(im_list_h, interpolation=interpolation)
                     for im_list_h in im_list_2d]
        return self.vconcat_resize_min(im_list_v, interpolation=interpolation)

    # Resize all images to the same width and concatenate them vertically
    # noinspection PyMethodMayBeStatic
    def vconcat_resize_min(self, im_list, interpolation=cv2.INTER_CUBIC):
        w_min = min(im.shape[1] for im in im_list)
        im_list_resize = [cv2.resize(im, (w_min, int(im.shape[0] * w_min / im.shape[1])),
                                     interpolation=interpolation)
                          for im in im_list]
        return cv2.vconcat(im_list_resize)

    # Resize the images to the same height and concatenate them horizontally
    # noinspection PyMethodMayBeStatic
    def hconcat_resize_min(self, im_list, interpolation=cv2.INTER_CUBIC):
        h_min = min(im.shape[0] for im in im_list)
        im_list_resize = [cv2.resize(im, (int(im.shape[1] * h_min / im.shape[0]), h_min),
                                     interpolation=interpolation)
                          for im in im_list]
        return cv2.hconcat(im_list_resize)

    # Main loop
    def run(self):

        # Wait for the parameter to be set
        while not rospy.has_param("camera_topics"):
            rospy.sleep(1)
            # rospy.loginfo("Waiting for parameters")

        camera_topics = rospy.get_param("camera_topics")

        # Create a controller subscriber
        # Not needed
        # rospy.Subscriber("/joy", Joy, self.joy_callback)

        # Create camera subscribers and image buffers
        for i, topic in enumerate(camera_topics):
            self.images.append(None)
            rospy.Subscriber(topic, Image, self.camera_callback, callback_args=i)
        
        rate = rospy.Rate(60)  # Display rate in Hz
        rospy.loginfo("Camera viewer node has been started")
        rospy.loginfo("Camera topics: %s", camera_topics)

        # Use keyboard to switch cameras and quit image display
        while not rospy.is_shutdown():
            key = cv2.waitKey(1) & 0xFF
            if ord('0') <= key < ord(str(len(camera_topics))):
                self.index = key - ord('0')
            elif key == ord('q'):
                break

            self.display_images()
            rate.sleep()

        cv2.destroyAllWindows()


if __name__ == "__main__":
    rospy.init_node("camera_viewer_node")
    Camera_Display_Node = CameraDisplayNode()
    Camera_Display_Node.run()
