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
        
    elif msg.data in ['record', 'calib']:
        topics = rospy.get_param('~topics_' + msg.data, '')
        save_as = rospy.get_param('~save_as', '')
        if topics and save_as:
            start_recording(topics, save_as)
        else:
            rospy.logerr("No topics or save_as parameter provided for recording type '%s'", msg.data)
    else:
        rospy.logerr("Invalid recording type: '%s'", msg.data)

if __name__ == '__main__':
    node_name = sys.argv[1]
    rospy.init_node(node_name)
    rospy.Subscriber('/start_recording_bag', String, start_stop_recording_callback)
    rospy.loginfo("Remote recorder node is ready.")
    rospy.spin()
