import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

"""
Fps for camera or for callback in hz?

Want camera FPS
Or ROS callback HZ
"""

class CameraFPS:
    def __init__(self):
        self.bridge = CvBridge()
        self.frame_count = 0
        self.start_time = None

    def image_callback(self, msg):
        if self.start_time is None:
            self.start_time = time.time()

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.frame_count += 1

        # Calculate FPS every 10 frames
        if self.frame_count % 10 == 0:
            elapsed_time = time.time() - self.start_time
            fps = self.frame_count / elapsed_time
            rospy.loginfo("Camera FPS: %.2f" % fps)
            self.frame_count = 0
            self.start_time = time.time()

def main():
    rospy.init_node('camera_fps_node')
    camera_fps = CameraFPS()
    rospy.Subscriber('/usb_cam_0/image_raw', Image, camera_fps.image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
