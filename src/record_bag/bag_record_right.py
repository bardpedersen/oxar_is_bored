#!/usr/bin/env python

import rospy
import subprocess
from std_msgs.msg import Empty

recording_process = None
is_recording = False

def start_stop_recording_callback(msg):
    global recording_process, is_recording
    if not is_recording:
        try:
            # Start rosbag recording for right camera
            recording_process = subprocess.Popen(['rosbag', 'record', 'tf_static', 'camera_r1/color/camera_info', 'camera_r2/color/camera_info', 'camera_r1/color/image_raw', 'camera_r2/color/image_raw', 'camera_r1/depth/color/points', 'camera_r2/depth/color/points', '/clock_sync', '-o', 'test_right.bag'])
            rospy.loginfo("Recording for right camera started")
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
                rospy.loginfo("Recording for right camera stopped")
                is_recording = False
            except Exception as e:
                rospy.logerr("Error stopping recording: %s", str(e))

if __name__ == '__main__':
    rospy.init_node('remote_recorder_right')
    start_stop_subscriber = rospy.Subscriber('/start_recording_bag', Empty, start_stop_recording_callback)
    rospy.loginfo("Remote recorder node for right camera is ready.")
    rospy.spin()
