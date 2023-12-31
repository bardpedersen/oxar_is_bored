#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import rospy

def create_path(t_step=0.1, duration=20, freq=0.1, freq_shift=1, phase_shift=np.pi/2):

    # Parameters
    amplitude = 100

    center_x = 100; center_y = 200; center_z = 200
    omega_x = freq
    omega_y = 0
    omega_z = freq * freq_shift
    phi_x = 0
    phi_y = 0
    phi_z = phase_shift

    x_coords = []; y_coords = []; z_coords = []; t_coords = []
    for t in np.arange(0, duration, t_step) + t_step:
        x = np.sin(omega_x*t + phi_x) * amplitude + center_x
        y = np.sin(omega_y*t + phi_y) * amplitude + center_y
        z = np.cos(omega_z*t + phi_z) * amplitude + center_z
        x_coords.append(x); y_coords.append(y); z_coords.append(z); t_coords.append(t)

    return zip(x_coords, y_coords, z_coords, t_coords)

def create_step(t_step=1, duration=20, step_x=50, step_y=0, step_z=0):
    # Initial position
    start_x = 100; start_y = 200; start_z = 200
    pose_x = True; pose_y = True; pose_z = True

    x_coords = []; y_coords = []; z_coords = []; t_coords = []
    for t in np.arange(0, duration, t_step) + t_step:
        if pose_x and pose_z:
            pose_x = not pose_x
        elif not pose_x and pose_z:
            pose_z = not pose_z
        elif not pose_x and not pose_z:
            pose_x = not pose_x
        elif pose_x and not pose_z:
            pose_z = not pose_z
        
        x = start_x + step_x if pose_x else start_x
        y = start_y + step_y if pose_y else start_y
        z = start_z + step_z if pose_z else start_z

        x_coords.append(x); y_coords.append(y); z_coords.append(z); t_coords.append(t)

    return zip(x_coords, y_coords, z_coords, t_coords)

def write_NM(width=100, height=100, t_step=1):
    start_x = 0
    start_y = 200
    start_z = 300
    path = [
        [start_x, start_y+width*0/4, start_z],
        [start_x, start_y+width*0/4, start_z-height],
        [start_x, start_y+width*0.3, start_z],
        [start_x, start_y+width*0.3, start_z-height],

        [start_x, start_y+width*2/4, start_z-height],
        [start_x, start_y+width*2/4, start_z],
        [start_x, start_y+width*2/4, start_z-height],
        [start_x, start_y+width*3/4, start_z],
        [start_x, start_y+width*4/4, start_z-height],
        [start_x, start_y+width*4/4, start_z],
    ]
    # Add the reverse
    path += reversed(path)
    # Add time to path
    path = [point+[(i+1)*t_step] for i, point in enumerate(path)]
    return path

def write_BU(width=100, height=100, t_step=1):
    start_x = 0
    start_y = 370
    start_z = 300
    B_path = [
        [start_x, start_y-width*0,      start_z-height*1],
        [start_x, start_y-width*0.3,   start_z-height*0.9],
        [start_x, start_y-width*0.4,   start_z-height*0.8],
        [start_x, start_y-width*0.4,   start_z-height*0.7],
        [start_x, start_y-width*0.3,   start_z-height*0.6],
        [start_x, start_y-width*0,      start_z-height*0.5],
        [start_x, start_y-width*0.3,   start_z-height*0.4],
        [start_x, start_y-width*0.4,   start_z-height*0.3],
        [start_x, start_y-width*0.4,   start_z-height*0.2],
        [start_x, start_y-width*0.3,   start_z-height*0.1],
        [start_x, start_y-width*0,      start_z-height*0],
        [start_x, start_y-width*0,      start_z-height*0],
    ]
    U_path = [
        [start_x, start_y-width*0,      start_z-height*1],
        [start_x, start_y-width*0,      start_z-height*1],
        [start_x, start_y-width*0.5,    start_z-height*1],
        [start_x, start_y-width*0.5,    start_z-height*0.5],

        [start_x, start_y-width*0.6,   start_z-height*0.1],
        [start_x, start_y-width*0.7,   start_z-height*0],
        [start_x, start_y-width*0.8,   start_z-height*0],
        [start_x, start_y-width*0.9,   start_z-height*0.1],

        
        [start_x, start_y-width*1,      start_z-height*0.5],
        [start_x, start_y-width*1,      start_z-height*1],
    ]
    path = B_path + U_path
    # Add the reverse
    path += reversed(path)
    # Add time to path
    path = [point+[(i+1)*t_step] for i, point in enumerate(path)]
    return path

if __name__ == "__main__":
    # Create points
    points = create_path(t_step=0.1)

    # Publish the points
    rospy.init_node('circle_publisher')
    rospy.set_param('/arm_movement', points)

