#!/usr/bin/env python3

import sys
import rospy
import subprocess
from std_msgs.msg import String

recording_process = None

def start_recording(topics, save_as):
    global recording_process
    command = ['rosbag', 'record'] + topics.split(',') + ['-o'] + save_as.split(',')
    recording_process = subprocess.Popen(command)
    rospy.loginfo("Recording started")

def stop_recording():
    global recording_process
    if recording_process is not None:
        recording_process.terminate()
        recording_process.wait()
        rospy.loginfo("Recording stopped")
        recording_process = None
    else:
        rospy.logwarn("No recording in progress to stop")

def start_stop_recording_callback(msg):
    global recording_process
    if msg.data == 'stop':
        stop_recording()

    elif recording_process is not None:
        rospy.logerr("Recording allredy in progress, stop with 'stop' before starting a new one")
    
    else:
        if msg.data == 'calib':
            topics = rospy.get_param('~topics_calib', '')
        elif msg.data == 'drive':
            topics = rospy.get_param('~topics_drive', '')
        else:
            topics = rospy.get_param('~topics_record', '')

        save_as = rospy.get_param('~save_as', '') + msg.data
        if topics and save_as:
            start_recording(topics, save_as)

        elif topics and not save_as:
            rospy.logerr("No save_as parameter provided for recording type '%s'", save_as)
        elif save_as and not topics:
            rospy.logerr("No topics parameter provided for recording type '%s'", topics)
        else:
            rospy.logerr("Unknown error '%s'", msg.data)

if __name__ == '__main__':
    node_name = sys.argv[1]
    rospy.init_node(node_name)
    rospy.Subscriber('/start_recording_bag', String, start_stop_recording_callback)
    rospy.loginfo("Remote recorder node is ready.")
    rospy.spin()
