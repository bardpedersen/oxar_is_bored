#!/usr/bin/env python3

import cv2
import rospy
from std_srvs.srv import Trigger, TriggerResponse


class VideoFinderService:
    def __init__(self):
        rospy.init_node('video_finder_server')
        rospy.Service('video_finder', Trigger, self.handle_service_request)
        rospy.spin()


    def handle_service_request(self, request):
        return TriggerResponse(success=True, message=str(self.get_working_cameras()))

    # Remove error message from terminal?
    def try_to_display_video(self, idx):
        cap = cv2.VideoCapture(idx)
        try:
            ret, frame = cap.read()
            cv2.imshow('Camera', frame)
            return True
        except:
            cap.release()
            cv2.destroyAllWindows()
            return False


    def get_working_cameras(self):
        num_cameras = 20  # Update this value to match your system
        working_cameras = []

        for camera_index in range(num_cameras):
            print('Trying to display camera: ', camera_index)
            if self.try_to_display_video(camera_index):
                working_cameras.append(camera_index)
        
        return working_cameras


if __name__ == '__main__':
    service = VideoFinderService()
    # print('Working cameras: ', service.get_working_cameras())

