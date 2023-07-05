#!/usr/bin/env python3

import cv2


def get_working_cameras():
    num_cameras = 20  # Update this value to match your system
    working_cameras = []

    for camera_index in range(num_cameras):
        # Create a video capture object for the current camera channel
        vid = cv2.VideoCapture(camera_index)

        # Check if video is available
        if vid.isOpened():
            working_cameras.append(camera_index)
    
    return working_cameras

print(f'Working cameras: {get_working_cameras()}')

