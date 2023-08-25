#!/usr/bin/env python3

import rospy

angle_lists = []
steps = 9
alpha = 30
beta = 30

for i in steps:
    angle_lists.append([alpha, beta])
    alpha += 30
    beta += 30

rospy.init_node('angle_generator_node')
rospy.set_param('/endeff_movement', angle_lists)
