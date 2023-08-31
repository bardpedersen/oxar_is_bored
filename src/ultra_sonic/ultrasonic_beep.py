#!/usr/bin/env python3

import rospy
from std_msgs.msg import UInt8
import sounddevice as sd
import numpy as np


class BeepingNode:
    def __init__(self):
        rospy.init_node('beeping_node', anonymous=True)

        self.sub_smooth = rospy.Subscriber('/ultrasonic_smooth', UInt8, self.ultrasonic_smooth_callback)
        self.last_beep_time = rospy.Time.now()

    def ultrasonic_smooth_callback(self, data):
        value = data.data
        time_since_last_beep = (rospy.Time.now() - self.last_beep_time).to_sec()

        time_between_beeps = value / 100

        if time_since_last_beep > time_between_beeps:
            print(f'Beep! {value}')
            self.play_beep(1000, 0.1)
            self.last_beep_time = rospy.Time.now()


    def play_beep(self, frequency, duration):
        sample_rate = 44100  # You can adjust the sample rate
        t = np.linspace(0, duration, int(sample_rate * duration), False)
        beep = 0.5 * np.sin(2 * np.pi * frequency * t)
        sd.play(beep, sample_rate)


if __name__ == '__main__':
    try:
        beeping_node = BeepingNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
