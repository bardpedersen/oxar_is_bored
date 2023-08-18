#!/usr/bin/env python3

import rospy
import subprocess
from std_msgs.msg import Empty

recording_process = None
is_recording = False

def start_stop_recording_callback(msg):
    global recording_process, is_recording
    if not is_recording:
        try:
            # Start rosbag recording for left camera
            recording_process = subprocess.Popen(['rosbag', 'record', 'tf_static', 'camera_l1/color/camera_info', 'camera_l2/color/camera_info', 'camera_l1/color/image_raw/compressed', 'camera_l2/color/image_raw/compressed', 'camera_l1/depth/color/points', 'camera_l2/depth/color/points', '/clock_sync', '-o', 'test_left.bag'])
            rospy.loginfo("Recording for left camera started")
            is_recording = True
        except Exception as e:
            rospy.logerr("Error starting recording: %s", str(e))
    else:
        if recording_process is not None:
            try:
                # Stop the rosbag recording process
                recording_process.terminate()
                recording_process.wait()
                recording_process = None
                rospy.loginfo("Recording for left camera stopped")
                is_recording = False
            except Exception as e:
                rospy.logerr("Error stopping recording: %s", str(e))

if __name__ == '__main__':
    rospy.init_node('remote_recorder_left')
    start_stop_subscriber = rospy.Subscriber('/start_recording_bag', Empty, start_stop_recording_callback)
    rospy.loginfo("Remote recorder node for left camera is ready.")
    rospy.spin()
