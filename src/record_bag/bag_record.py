#!/usr/bin/env python3

import sys
import rospy
import subprocess
from std_msgs.msg import Empty

recording_process = None
is_recording = False

def start_stop_recording_callback(msg):
    global recording_process, is_recording
    if not is_recording:
        try:
            # Start rosbag recording for nuc pc
            topics = rospy.get_param('~topics', '')
            save_as = rospy.get_param('~save_as', '')
            command = ['rosbag', 'record'] + topics.split(',') + ['-o'] + save_as.split(',')
            recording_process = subprocess.Popen(command)            
            rospy.loginfo("Recording for started")
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
                rospy.loginfo("Recording stopped")
                is_recording = False
            except Exception as e:
                rospy.logerr("Error stopping recording: %s", str(e))

if __name__ == '__main__':
    node_name = sys.argv[1]
    rospy.init_node(node_name)
    start_stop_subscriber = rospy.Subscriber('/start_recording_bag', Empty, start_stop_recording_callback)
    rospy.loginfo("Remote recorder node is ready.")
    rospy.spin()
